#include "agv_map_3d/assets.hpp"

#include <boost/process.hpp>
#include <sys/file.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include <cmath>
#include <fstream>
#include <iostream>
#include <set>
#include <sstream>
#include <vector>

namespace fs = std::filesystem;
namespace process = boost::process;
using json = nlohmann::json;

namespace agv_map_3d
{
namespace
{
double epochSeconds()
{
  return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
}

// Check the format and exact XYZ payload size, not just the exporter's exit code.
std::pair<uint64_t, uint64_t> inspectPly(const fs::path & file)
{
  std::ifstream stream(file, std::ios::binary);
  if (!stream) throw std::runtime_error("Cannot open generated PLY");
  std::string line;
  uint64_t count = 0, header = 0;
  std::vector<std::string> properties;
  bool binary = false, ended = false;
  if (!std::getline(stream, line) || line != "ply") throw std::runtime_error("Invalid PLY header");
  header = 4;
  while (std::getline(stream, line)) {
    header += line.size() + 1;
    if (header > 4096) throw std::runtime_error("PLY header too large");
    if (line == "format binary_little_endian 1.0") binary = true;
    if (line.rfind("element vertex ", 0) == 0) {
      std::size_t consumed = 0;
      const auto value = line.substr(15);
      count = std::stoull(value, &consumed);
      if (consumed != value.size()) throw std::runtime_error("Invalid PLY vertex count");
    }
    if (line.rfind("property ", 0) == 0) properties.push_back(line);
    if (line == "end_header") {ended = true; break;}
  }
  const auto size = fs::file_size(file);
  if (!ended || !binary || count == 0 || count > 5000000 ||
    properties != std::vector<std::string>{"property float x", "property float y", "property float z"} ||
    size != header + count * 12)
  {
    throw std::runtime_error("Invalid or truncated binary XYZ PLY");
  }
  return {count, size};
}

std::string tail(const fs::path & file)
{
  std::ifstream stream(file, std::ios::binary | std::ios::ate);
  if (!stream) return {};
  const auto length = stream.tellg();
  stream.seekg(length > 2000 ? length - std::streamoff(2000) : std::streampos(0));
  return {std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>()};
}
}  // namespace

Assets::Assets(Options options) : options_(std::move(options))
{
  fs::create_directories(options_.root);
  options_.root = fs::canonical(options_.root);
  const auto lock = options_.root / ".map3d.server.lock";
  lock_fd_ = ::open(lock.c_str(), O_CREAT | O_RDWR | O_CLOEXEC | O_NOFOLLOW, 0600);
  if (lock_fd_ < 0 || ::flock(lock_fd_, LOCK_EX | LOCK_NB) != 0) {
    if (lock_fd_ >= 0) ::close(lock_fd_);
    lock_fd_ = -1;
    throw std::runtime_error("Cannot lock maps directory; another 3D server may already own it");
  }
}

Assets::~Assets()
{
  stop();
  if (lock_fd_ >= 0) ::close(lock_fd_);
}

void Assets::start()
{
  try {
    worker_ = std::thread([this]() {worker();});
    watcher_ = std::thread([this]() {watch();});
  } catch (...) {
    stop();
    throw;
  }
}

void Assets::stop()
{
  if (stopped_.exchange(true)) return;
  changed_.notify_all();
  if (watcher_.joinable()) watcher_.join();
  if (worker_.joinable()) worker_.join();
}

fs::path Assets::checked(const fs::path & path) const
{
  const auto resolved = fs::weakly_canonical(path);
  auto part = resolved.begin();
  for (const auto & root_part : options_.root) {
    if (part == resolved.end() || *part++ != root_part) throw InvalidPath("Asset escapes maps directory");
  }
  if (part == resolved.end()) throw InvalidPath("Expected a map file, not its root directory");
  return path;
}

std::string Assets::signature(const fs::path & file, const std::string & kind) const
{
  struct stat data {};
  if (::stat(checked(file).c_str(), &data) != 0) throw NotFound("3D map source not found");
  std::ostringstream result;
  result.precision(17);
  result << data.st_dev << ':' << data.st_ino << ':' << data.st_mtim.tv_sec << ':'
         << data.st_mtim.tv_nsec << ':' << data.st_size << ':' << options_.voxel << ':' << kind << ":v2";
  return result.str();
}

Assets::Source Assets::locate(const std::string & name) const
{
  if (name.empty() || name.size() > 128 || name.find("..") != std::string::npos ||
    name.find_first_of("/\\") != std::string::npos || name.find('\0') != std::string::npos)
  {
    throw InvalidPath("Invalid map name");
  }
  const std::vector<fs::path> candidates{
    options_.root / name / (name + ".pbstream"),
    options_.root / (name + ".pbstream"), options_.root / name / "global.pcd"};
  for (const auto & file : candidates) {
    if (options_.source_format == "pcd" && file.extension() != ".pcd") continue;
    if (options_.source_format == "pbstream" && file.extension() != ".pbstream") continue;
    std::error_code error;
    if (!fs::is_regular_file(file, error)) continue;
    checked(file);  // Every source type, including both PBStream layouts.
    Source source;
    source.name = name;
    source.file = file;
    source.kind = file.extension() == ".pcd" ? "pcd" : "pbstream";
    source.ply = file;
    source.ply.replace_extension(".ply");
    source.meta = file;
    source.meta.replace_extension(".3d.json");
    checked(source.ply);
    checked(source.meta);
    source.version = signature(file, source.kind);
    return source;
  }
  throw NotFound("3D map source not found");
}

json Assets::status(const std::string & name)
{
  const auto source = locate(name);
  std::lock_guard<std::mutex> guard(mutex_);
  if (const auto job = jobs_.find(name); job != jobs_.end() && job->second["version"] == source.version) {
    return job->second;
  }
  try {
    json state;
    const auto completed = completed_.find(name);
    if (completed != completed_.end() && completed->second.value("version", "") == source.version) {
      state = completed->second;
    } else {
      std::ifstream input(source.meta);
      if (input) state = json::parse(input);
    }
    if (state.is_object()) {
      if (state.value("version", "") == source.version && state.value("name", "") == name &&
        state.value("frame_id", "") == "map")
      {
        if (state.value("status", "") == "ready") {
          const auto [count, size] = inspectPly(checked(source.ply));
          if (state.value("point_count", uint64_t(0)) == count && state.value("size_bytes", uint64_t(0)) == size) return state;
        }
        if (state.value("status", "") == "failed" && state.value("retry_at", 0.0) > epochSeconds()) return state;
      }
    }
  } catch (const std::exception &) { /* A bad cache is regenerated, never served. */ }
  json state{{"name", name}, {"status", "generating"}, {"frame_id", "map"},
    {"version", source.version}, {"voxel_size", options_.voxel}};
  const auto now = std::chrono::steady_clock::now();
  auto & observation = observations_[name];
  if (observation.version != source.version) observation = {source.version, now};
  if (std::chrono::duration<double>(now - observation.since).count() < options_.settle_seconds) {
    state["message"] = "Waiting for a stable source file";
    return state;
  }
  // The old version is still being cancelled/validated; do not start two jobs.
  if (!jobs_.count(name) && !stopped_) {
    jobs_[name] = state;
    queue_.push_back(source);
    changed_.notify_all();
  }
  return state;
}

fs::path Assets::cloud(const std::string & name, const std::string & version) const
{
  const auto source = locate(name);
  if (source.version != version) throw Changed("Map changed; request status again");
  return checked(source.ply);
}

void Assets::worker()
{
  while (!stopped_) {
    Source source;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      changed_.wait(lock, [this]() {return stopped_ || !queue_.empty();});
      if (stopped_) return;
      source = queue_.front();
      queue_.pop_front();
    }
    try {exportSource(source);}
    catch (const std::exception & error) {std::cerr << "3D export: " << error.what() << '\n';}
    // Always release the job, including path errors, map deletion and disk errors.
    std::lock_guard<std::mutex> guard(mutex_);
    jobs_.erase(source.name);
  }
}

void Assets::exportSource(const Source & source)
{
  const auto suffix = ".export-" + std::to_string(::getpid());
  const fs::path stage = source.ply.string() + suffix;
  const fs::path stdout_path = stage.string() + ".stdout";
  const fs::path stderr_path = stage.string() + ".stderr";
  json state{{"name", source.name}, {"frame_id", "map"}, {"version", source.version},
    {"voxel_size", options_.voxel}};
  try {
    checked(stage);
    checked(stdout_path);
    checked(stderr_path);
    if (signature(source.file, source.kind) != source.version) throw std::runtime_error("Map changed before export");
    const auto exporter = source.kind == "pcd" ? options_.pcd_exporter : options_.pbstream_exporter;
    std::ostringstream voxel;
    voxel.precision(17);
    voxel << options_.voxel;
    process::group group;
    process::child child(exporter.string(),
      process::args(std::vector<std::string>{source.file.string(), stage.string(), voxel.str()}),
      process::std_out > stdout_path.string(), process::std_err > stderr_path.string(), group);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(options_.timeout_seconds);
    while (!child.wait_for(std::chrono::milliseconds(50))) {
      if (stopped_ || std::chrono::steady_clock::now() >= deadline) {
        group.terminate();
        child.wait();
        throw std::runtime_error(stopped_ ? "Export cancelled on shutdown" : "3D export timed out");
      }
    }
    if (child.exit_code() != 0) throw std::runtime_error("Exporter failed: " + tail(stderr_path) + tail(stdout_path));
    if (stopped_) throw std::runtime_error("Export cancelled on shutdown");
    const auto [count, size] = inspectPly(checked(stage));
    if (signature(source.file, source.kind) != source.version) throw std::runtime_error("Map changed while exporting");
    fs::rename(checked(stage), checked(source.ply));
    state.update({{"status", "ready"}, {"point_count", count}, {"size_bytes", size}});
  } catch (const std::exception & error) {
    state.update({{"status", "failed"}, {"message", error.what()},
      {"retry_at", epochSeconds() + options_.retry_seconds}});
  }
  for (const auto & path : {stage, fs::path(stage.string() + ".tmp"), stdout_path, stderr_path}) {
    try {std::error_code error; fs::remove(checked(path), error);} catch (const std::exception &) {}
  }
  if (stopped_) return;
  {
    std::lock_guard<std::mutex> guard(mutex_);
    completed_[source.name] = state;  // Keep errors visible even if the disk is read-only/full.
  }
  const fs::path temporary = source.meta.string() + ".tmp";
  try {
    std::ofstream output(checked(temporary));
    output.exceptions(std::ios::badbit | std::ios::failbit);
    output << state.dump(-1, ' ', false, json::error_handler_t::replace);
    output.close();
    fs::rename(checked(temporary), checked(source.meta));
  } catch (...) {
    try {std::error_code error; fs::remove(checked(temporary), error);} catch (const std::exception &) {}
    throw;
  }
}

void Assets::watch()
{
  while (!stopped_) {
    try {
      std::set<std::string> names;
      for (const auto & entry : fs::directory_iterator(options_.root)) {
        if (entry.is_regular_file() && entry.path().extension() == ".pbstream") names.insert(entry.path().stem().string());
        if (entry.is_directory()) {
          const auto name = entry.path().filename().string();
          if (fs::is_regular_file(entry.path() / (name + ".pbstream")) ||
            fs::is_regular_file(entry.path() / "global.pcd")) names.insert(name);
        }
      }
      for (const auto & name : names) {
        try {status(name);} catch (const std::exception &) {}
      }
    } catch (const std::exception & error) {std::cerr << "3D map scan: " << error.what() << '\n';}
    std::unique_lock<std::mutex> lock(mutex_);
    changed_.wait_for(lock, std::chrono::duration<double>(options_.scan_seconds), [this]() {return stopped_.load();});
  }
}
}  // namespace agv_map_3d
