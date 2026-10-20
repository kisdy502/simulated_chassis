#pragma once

#include <nlohmann/json.hpp>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <filesystem>
#include <map>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

namespace agv_map_3d
{
struct NotFound : std::runtime_error {using std::runtime_error::runtime_error;};
struct InvalidPath : std::runtime_error {using std::runtime_error::runtime_error;};
struct Changed : std::runtime_error {using std::runtime_error::runtime_error;};
struct Options
{
  std::filesystem::path root, pbstream_exporter, pcd_exporter;
  std::string source_format = "auto";
  double voxel = 0.1;
  double settle_seconds = 2.0;
  double scan_seconds = 10.0;
  double timeout_seconds = 300.0;
  double retry_seconds = 30.0;
};

class Assets
{
public:
  explicit Assets(Options options);
  ~Assets();
  void start();
  void stop();
  nlohmann::json status(const std::string & name);
  std::filesystem::path cloud(const std::string & name, const std::string & version) const;

private:
  struct Source
  {
    std::string name, kind, version;
    std::filesystem::path file, ply, meta;
  };
  struct Observation
  {
    std::string version;
    std::chrono::steady_clock::time_point since;
  };
  Source locate(const std::string & name) const;
  std::filesystem::path checked(const std::filesystem::path & path) const;
  std::string signature(const std::filesystem::path & file, const std::string & kind) const;
  void worker();
  void watch();
  void exportSource(const Source & source);
  Options options_;
  int lock_fd_ = -1;
  std::atomic<bool> stopped_{false};
  std::mutex mutex_;
  std::condition_variable changed_;
  std::deque<Source> queue_;
  std::map<std::string, nlohmann::json> jobs_;
  std::map<std::string, Observation> observations_;
  std::map<std::string, nlohmann::json> completed_;
  std::thread worker_, watcher_;
};
}  // namespace agv_map_3d
