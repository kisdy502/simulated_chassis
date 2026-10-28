#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <thread>
#include "core/system/tracked_pose.h"

using namespace lightning;

static void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

static bool close(const SE3& a, const SE3& b) {
    return (a.inverse() * b).log().norm() < 1e-9;
}

int main() {
    NavState initial;
    initial.timestamp_ = 42.25;
    initial.SetPose(SE3(SO3::exp(Vec3d(0.1, -0.2, 0.4)), Vec3d(3, 4, 0.2)));
    auto cloud = std::make_shared<PointCloudType>();
    auto keyframe = std::make_shared<Keyframe>(7, cloud, initial);
    const SE3 correction(SO3::exp(Vec3d(-0.02, 0.05, 0.7)), Vec3d(0.4, -0.8, 0.3));
    keyframe->SetOptPose(correction * initial.GetPose());
    NavState current = initial;
    current.timestamp_ = 43.123456789;
    current.SetPose(initial.GetPose() * SE3(SO3::exp(Vec3d(0, 0, 0.2)), Vec3d(0.5, -0.1, 0)));
    require(close(keyframe->PoseInMap(current.GetPose()), correction * current.GetPose()), "Wrong SE3 correction order");
    auto message = MakeTrackedPose(current, keyframe);
    require(message.has_value(), "Valid pose missing");
    require(message->header.frame_id == "map", "Wrong frame");
    require(message->header.stamp.sec == 43 && message->header.stamp.nanosec == 123456789, "Processing clock leaked into measurement stamp");
    require((Vec3d(message->pose.position.x, message->pose.position.y, message->pose.position.z) -
             (correction * current.GetPose()).translation()).norm() < 1e-9, "Published raw LIO instead of map pose");
    const auto& q = message->pose.orientation;
    require(close(SE3(Quatd(q.w, q.x, q.y, q.z),
                      Vec3d(message->pose.position.x, message->pose.position.y, message->pose.position.z)),
                  correction * current.GetPose()), "Quaternion correction mismatch");
    auto raw = MakeTrackedPose(current, nullptr);
    require(raw.has_value() && std::abs(raw->pose.position.x - current.pos_.x()) < 1e-12, "No-loop fallback broken");
    auto snapshot = keyframe->RenderSnapshot();
    keyframe->SetOptPose(SE3());
    require(close(snapshot->GetOptPose(), correction * initial.GetPose()), "Rendering pose changed during optimization");
    require(snapshot->GetCloud() == cloud && snapshot->GetState().timestamp_ == initial.timestamp_, "Snapshot changed body cloud/state");

    std::thread optimizer([&]() {
        for (int i = 0; i < 20000; ++i) keyframe->SetOptPose(i % 2 ? SE3() : correction);
    });
    for (int i = 0; i < 20000; ++i) {
        auto frame = keyframe->RenderSnapshot();
        const auto pose = frame->GetOptPose();
        require(close(pose, SE3()) || close(pose, correction), "Torn optimization pose");
        require(close(frame->GetOptPose(), pose), "Snapshot not stable");
    }
    optimizer.join();
    current.timestamp_ = 0;
    require(!MakeTrackedPose(current, keyframe), "Uninitialized state published");
    current.timestamp_ = std::numeric_limits<double>::quiet_NaN();
    require(!MakeTrackedPose(current, keyframe), "NaN timestamp accepted");
    std::cout << "PASS: loop-corrected map pose, scan timestamp, no-loop fallback, immutable render snapshots, concurrent optimization, invalid stamps\n";
}
