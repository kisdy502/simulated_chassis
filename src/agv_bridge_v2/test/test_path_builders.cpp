#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include "agv_bridge_v2/navigation/StraightPathBuilder.hpp"
#include "agv_bridge_v2/navigation/BezierPathBuilder.hpp"
#include "agv_bridge_v2/navigation/BackUpPathBuilder.hpp"
#include "agv_bridge_v2/utils/TransformUtils.hpp"
#include <cmath>

namespace agv_bridge {
namespace {
const auto logger = rclcpp::get_logger("path_builder_test");
geometry_msgs::msg::Pose origin(double yaw = 0) {
    geometry_msgs::msg::Pose p;
    p.orientation = TransformUtils::yaw_to_quaternion(yaw);
    return p;
}
MoveToMessage task(double x, double y, double step = 0.1) {
    MoveToMessage m;
    m.x = x; m.y = y; m.edgeInfo.step = step;
    m.edgeInfo.id = "edge"; m.nodeId = "target"; m.edgeInfo.sourceId = "source";
    return m;
}
void headers(const std::vector<geometry_msgs::msg::PoseStamped> &path) {
    for (const auto &p : path) {
        EXPECT_EQ(p.header.frame_id, "map");
        EXPECT_EQ(p.header.stamp.sec, 0); EXPECT_EQ(p.header.stamp.nanosec, 0u);
    }
}
TEST(PathBuilders, StraightRetainsSamplingHeadingAndExactEndpoint) {
    const auto path = StraightPathBuilder::build(origin(), task(3, 4), logger);
    ASSERT_EQ(path.size(), 51u); headers(path);
    EXPECT_DOUBLE_EQ(path.front().pose.position.x, 0);
    EXPECT_DOUBLE_EQ(path.back().pose.position.x, 3);
    EXPECT_DOUBLE_EQ(path.back().pose.position.y, 4);
    EXPECT_NEAR(path[25].pose.position.x, 1.5, 1e-12);
    EXPECT_NEAR(path[25].pose.position.y, 2, 1e-12);
    for (const auto &p : path)
        EXPECT_NEAR(TransformUtils::quaternion_to_yaw(p.pose.orientation), std::atan2(4., 3.), 1e-12);
    EXPECT_EQ(StraightPathBuilder::build(origin(), task(0, 0), logger).size(), 10u);
}
TEST(PathBuilders, BackUpKeepsRobotHeadingInsteadOfFacingTarget) {
    auto start = origin(0.6);
    const auto path = BackUpPathBuilder::build(start, task(-1, 0), logger);
    ASSERT_EQ(path.size(), 11u); headers(path);
    EXPECT_DOUBLE_EQ(path.back().pose.position.x, -1);
    for (const auto &p : path)
        EXPECT_NEAR(TransformUtils::quaternion_to_yaw(p.pose.orientation), 0.6, 1e-12);
    EXPECT_EQ(BackUpPathBuilder::build(start, task(0, 0), logger).size(), 10u);
}
TEST(PathBuilders, QuadraticRetainsCurveAndTangentSamples) {
    auto m = task(2, 0, 0.2); m.edgeInfo.controlPoints = {{1, 1}};
    const auto path = BezierPathBuilder::build(origin(), m, logger);
    ASSERT_EQ(path.size(), 12u); headers(path);
    for (std::size_t i = 0; i < path.size(); ++i) {
        const double t = static_cast<double>(i) / (path.size() - 1);
        EXPECT_NEAR(path[i].pose.position.x, 2*t, 1e-12);
        EXPECT_NEAR(path[i].pose.position.y, 2*t*(1-t), 1e-12);
        if (i == 1 || i == path.size()-1 || i % 2 == 0)
            EXPECT_NEAR(TransformUtils::quaternion_to_yaw(path[i].pose.orientation), std::atan2(2-4*t, 2.), 1e-12);
        else {
            // Preserve the legacy unset-orientation policy in this structural refactor.
            const geometry_msgs::msg::Quaternion defaults;
            EXPECT_DOUBLE_EQ(path[i].pose.orientation.x, defaults.x);
            EXPECT_DOUBLE_EQ(path[i].pose.orientation.y, defaults.y);
            EXPECT_DOUBLE_EQ(path[i].pose.orientation.z, defaults.z);
            EXPECT_DOUBLE_EQ(path[i].pose.orientation.w, defaults.w);
        }
    }
}
TEST(PathBuilders, CubicAndSourceTargetControlPointOrderArePreserved) {
    auto m = task(2, 0, 0.2); m.edgeInfo.controlPoints = {{0, 1}, {2, 1}};
    auto path = BezierPathBuilder::build(origin(), m, logger);
    ASSERT_FALSE(path.empty()); headers(path);
    for (std::size_t i = 0; i < path.size(); ++i) {
        const double t = static_cast<double>(i) / (path.size()-1);
        EXPECT_NEAR(path[i].pose.position.x, 6*t*t-4*t*t*t, 1e-12);
        EXPECT_NEAR(path[i].pose.position.y, 3*t*(1-t), 1e-12);
    }
    m.nodeId = m.edgeInfo.sourceId;
    path = BezierPathBuilder::build(origin(), m, logger);
    ASSERT_GT(path.size(), 2u);
    const double t = 1.0 / (path.size()-1);
    EXPECT_NEAR(path[1].pose.position.x, 6*(1-t)*(1-t)*t + 2*t*t*t, 1e-12);
    EXPECT_DOUBLE_EQ(path.back().pose.position.x, m.x);
    EXPECT_DOUBLE_EQ(path.back().pose.position.y, m.y);
    EXPECT_NEAR(BezierPathBuilder::calculateBezierTangent(0, 0, 0, {0,2}, {1,1}, 2, 0), M_PI/2, 1e-12);
}
TEST(PathBuilders, InvalidCurvesStillFailInsteadOfFallingBackToStraight) {
    auto m = task(2, 0);
    EXPECT_TRUE(BezierPathBuilder::build(origin(), m, logger).empty());
    m.edgeInfo.controlPoints = {{0,1},{1,1},{2,1}};
    EXPECT_TRUE(BezierPathBuilder::build(origin(), m, logger).empty());
}
} // namespace
} // namespace agv_bridge
