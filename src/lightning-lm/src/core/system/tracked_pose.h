#pragma once

#include <cmath>
#include <limits>
#include <optional>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "common/keyframe.h"

namespace lightning {

// Build the ROS/map contract independently of the ROS processing clock.
inline std::optional<geometry_msgs::msg::PoseStamped> MakeTrackedPose(
    const NavState& state, const Keyframe::Ptr& keyframe) {
    if (!std::isfinite(state.timestamp_) || state.timestamp_ <= 0 ||
        state.timestamp_ > std::numeric_limits<int32_t>::max() ||
        !state.pos_.allFinite() || !state.rot_.matrix().allFinite()) return std::nullopt;
    auto map_pose = state.GetPose();
    if (keyframe) map_pose = keyframe->PoseInMap(map_pose);

    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    const auto ns = static_cast<int64_t>(std::llround(state.timestamp_ * 1e9));
    pose.header.stamp.sec = static_cast<int32_t>(ns / 1000000000LL);
    pose.header.stamp.nanosec = static_cast<uint32_t>(ns % 1000000000LL);
    pose.pose.position.x = map_pose.translation().x();
    pose.pose.position.y = map_pose.translation().y();
    pose.pose.position.z = map_pose.translation().z();
    const auto q = map_pose.unit_quaternion();
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();
    return pose;
}

}  // namespace lightning
