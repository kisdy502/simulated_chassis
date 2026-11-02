#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include "agv_bridge_v2/navigation/FreeNavigationExecutor.hpp"
#include "agv_bridge_v2/navigation/PathNavigationExecutor.hpp"
#include "agv_bridge_v2/utils/TransformUtils.hpp"

namespace agv_bridge {
namespace {
struct Context : NavigationExecutionContext {
    rclcpp::Logger logger = rclcpp::get_logger("executor_test");
    std::optional<geometry_msgs::msg::PoseStamped> pose = geometry_msgs::msg::PoseStamped{};
    bool send_ok = true, server_ready = true;
    int navigate_count = 0, path_count = 0, finished = 0, failed = 0;
    bool success = false;
    nav2_msgs::action::NavigateToPose::Goal navigate;
    nav2_msgs::action::FollowPath::Goal path;
    std::vector<double> spins, speeds;
    std::vector<NavigationState> stages;
    std::string message, ack;
    double x = 0, y = 0, yaw = 0;
    const rclcpp::Logger &executionLogger() const override { return logger; }
    rclcpp::Time executionNow() const override { return rclcpp::Time(123, 0); }
    std::optional<geometry_msgs::msg::PoseStamped> executionPose() const override { return pose; }
    double backUpHeadingLimit() const override { return 20; }
    bool navigateServerReady() const override { return server_ready; }
    bool sendNavigateGoal(const nav2_msgs::action::NavigateToPose::Goal &goal) override {
        ++navigate_count; navigate = goal; return send_ok;
    }
    bool sendPathGoal(const nav2_msgs::action::FollowPath::Goal &goal) override {
        ++path_count; path = goal; return send_ok;
    }
    bool sendSpinCommand(double angle) override { spins.push_back(angle); return send_ok; }
    void setSpeedLimit(double speed) override { speeds.push_back(speed); }
    void setState(NavigationState state) override { stages.push_back(state); }
    void finishTask(bool ok, const std::string, const std::string text) override {
        ++finished; success = ok; message = text;
    }
    void failTask(const std::string, const std::string text) override { ++failed; message = text; }
    void reportAccepted(const std::string &text) override { ack = text; }
    void reportFeedback(double px, double py, double angle) override { x=px; y=py; yaw=angle; }
};
MoveToMessage task() {
    MoveToMessage t; t.commandId="test"; t.nodeId="target";
    t.edgeInfo.id="edge"; t.edgeInfo.sourceId="source";
    t.edgeInfo.step=0.2; t.edgeInfo.maxSpeed=0.4; t.x=2;
    return t;
}

TEST(NavigationExecutors, FreeUsesNavigateToPoseAndReportsThroughContext) {
    Context c; FreeNavigationExecutor executor(c);
    auto t=task(); t.y=1; t.theta=0.5;
    ASSERT_TRUE(executor.start(t));
    EXPECT_EQ(c.navigate_count,1); EXPECT_EQ(c.path_count,0); EXPECT_TRUE(c.spins.empty());
    EXPECT_EQ(c.navigate.pose.header.frame_id,"map"); EXPECT_EQ(c.navigate.pose.header.stamp.sec,123);
    EXPECT_DOUBLE_EQ(c.navigate.pose.pose.position.x,t.x);
    EXPECT_NEAR(TransformUtils::quaternion_to_yaw(c.navigate.pose.pose.orientation),t.theta,1e-12);
    ASSERT_EQ(c.speeds.size(),1u); EXPECT_DOUBLE_EQ(c.speeds[0],0.4);
    EXPECT_EQ(c.stages.back(),NavigationState::EXECUTING);
    executor.onAccepted(); EXPECT_FALSE(c.ack.empty());
    nav2_msgs::action::NavigateToPose::Feedback feedback;
    feedback.current_pose.pose.position.x=1.2; feedback.current_pose.pose.position.y=0.3;
    feedback.current_pose.pose.orientation=TransformUtils::yaw_to_quaternion(0.2);
    executor.onFeedback(feedback); EXPECT_DOUBLE_EQ(c.x,1.2); EXPECT_NEAR(c.yaw,0.2,1e-12);
    executor.onResult(rclcpp_action::ResultCode::SUCCEEDED);
    EXPECT_EQ(c.finished,1); EXPECT_TRUE(c.success);
}

TEST(NavigationExecutors, FreeRejectsInvalidOrUnavailableGoalsBeforeDispatch) {
    Context c; FreeNavigationExecutor executor(c); auto t=task(); t.edgeInfo.backUp=true;
    EXPECT_FALSE(executor.start(t)); EXPECT_EQ(c.failed,1); EXPECT_EQ(c.navigate_count,0);
    t.edgeInfo.backUp=false; c.server_ready=false;
    EXPECT_FALSE(executor.start(t)); EXPECT_EQ(c.failed,2); EXPECT_EQ(c.navigate_count,0);
}

TEST(NavigationExecutors, StraightWaitsForPreRotationAndFinalRotationBeforeCompletion) {
    Context c; PathNavigationExecutor executor(c); auto t=task(); t.x=0; t.y=2;
    t.theta=1.0; t.endPoint=true;
    ASSERT_TRUE(executor.startStraight(t));
    ASSERT_EQ(c.spins.size(),1u); EXPECT_NEAR(c.spins[0],M_PI/2,1e-12);
    EXPECT_EQ(c.path_count,0); EXPECT_EQ(c.stages.back(),NavigationState::PRE_ROTATING);
    executor.onSpinSucceeded(NavigationState::PRE_ROTATING,c.pose->pose,t);
    EXPECT_EQ(c.path_count,1); EXPECT_EQ(c.stages.back(),NavigationState::EXECUTING);
    EXPECT_EQ(c.path.controller_id,"FollowPath"); EXPECT_EQ(c.path.path.header.frame_id,"map");
    // Preserve the original straight Path header stamp (zero).
    EXPECT_EQ(c.path.path.header.stamp.sec,0);
    executor.onResult(rclcpp_action::ResultCode::SUCCEEDED,t);
    ASSERT_EQ(c.spins.size(),2u); EXPECT_NEAR(c.spins[1],1.0,1e-12);
    EXPECT_EQ(c.stages.back(),NavigationState::END_ROTATING); EXPECT_EQ(c.finished,0);
    executor.onSpinSucceeded(NavigationState::END_ROTATING,c.pose->pose,t);
    EXPECT_EQ(c.finished,1); EXPECT_TRUE(c.success);
}

TEST(NavigationExecutors, CurveResumesBezierFollowingAfterTangentRotation) {
    Context c; PathNavigationExecutor executor(c); auto t=task();
    t.edgeInfo.type=EdgeType::CURVE; t.edgeInfo.controlPoints={{1,1}};
    ASSERT_TRUE(executor.startCurve(t)); EXPECT_EQ(c.path_count,0);
    ASSERT_EQ(c.spins.size(),1u); EXPECT_NEAR(c.spins[0],M_PI/4,1e-12);
    executor.onSpinSucceeded(NavigationState::PRE_ROTATING,c.pose->pose,t);
    ASSERT_EQ(c.path_count,1); ASSERT_EQ(c.path.path.poses.size(),12u);
    EXPECT_EQ(c.path.path.header.stamp.sec,123);
    EXPECT_GT(c.path.path.poses[5].pose.position.y,0);
}

TEST(NavigationExecutors, BackUpNeverPreRotatesAndUsesNegativeSpeed) {
    Context c; PathNavigationExecutor executor(c); auto t=task(); t.x=-2; t.edgeInfo.backUp=true;
    ASSERT_TRUE(executor.startStraight(t)); EXPECT_TRUE(c.spins.empty());
    EXPECT_EQ(c.path_count,1); EXPECT_EQ(c.stages.back(),NavigationState::BACKING_UP);
    ASSERT_EQ(c.speeds.size(),1u); EXPECT_DOUBLE_EQ(c.speeds[0],-0.4);
    for (const auto &p:c.path.path.poses)
        EXPECT_NEAR(TransformUtils::quaternion_to_yaw(p.pose.orientation),0,1e-12);
    executor.onResult(rclcpp_action::ResultCode::SUCCEEDED,t);
    EXPECT_EQ(c.finished,1); EXPECT_TRUE(c.spins.empty());
}

TEST(NavigationExecutors, BadBackUpHeadingFailsWithoutSendingMotion) {
    Context c; PathNavigationExecutor executor(c); auto t=task(); t.x=-2; t.edgeInfo.backUp=true;
    c.pose->pose.orientation=TransformUtils::yaw_to_quaternion(M_PI/2);
    EXPECT_TRUE(executor.startStraight(t)); // Existing convention: result already delivered.
    EXPECT_EQ(c.failed,1); EXPECT_EQ(c.path_count,0); EXPECT_TRUE(c.spins.empty());
}

TEST(NavigationExecutors, LostPoseAtPathCompletionReportsFailureInsteadOfThrowing) {
    Context c; PathNavigationExecutor executor(c); c.pose.reset();
    EXPECT_NO_THROW(executor.onResult(rclcpp_action::ResultCode::SUCCEEDED,task()));
    EXPECT_EQ(c.failed,1); EXPECT_EQ(c.finished,0); EXPECT_TRUE(c.spins.empty());
}

TEST(NavigationExecutors, MissingPoseAtEntryReturnsFailureBeforeAnyMotion) {
    Context c; PathNavigationExecutor executor(c); c.pose.reset();
    EXPECT_FALSE(executor.startStraight(task()));
    EXPECT_EQ(c.failed,1); EXPECT_EQ(c.path_count,0); EXPECT_TRUE(c.spins.empty());
    auto curve=task(); curve.edgeInfo.type=EdgeType::CURVE; curve.edgeInfo.controlPoints={{1,1}};
    EXPECT_FALSE(executor.startCurve(curve)); EXPECT_EQ(c.failed,2);
}

TEST(NavigationExecutors, FailedDispatchDoesNotEnterExecuting) {
    Context c; c.send_ok=false; FreeNavigationExecutor free(c); PathNavigationExecutor path(c);
    EXPECT_FALSE(free.start(task())); EXPECT_TRUE(c.stages.empty());
    EXPECT_FALSE(path.startStraight(task())); EXPECT_TRUE(c.stages.empty());
}

TEST(NavigationExecutors, AbortAndCancelAreReportedToSingleOwner) {
    Context c; PathNavigationExecutor path(c); FreeNavigationExecutor free(c);
    path.onResult(rclcpp_action::ResultCode::ABORTED,task()); EXPECT_EQ(c.failed,1);
    path.onResult(rclcpp_action::ResultCode::CANCELED,task()); EXPECT_EQ(c.finished,1); EXPECT_FALSE(c.success);
    free.onResult(rclcpp_action::ResultCode::ABORTED); EXPECT_EQ(c.failed,2);
    free.onResult(rclcpp_action::ResultCode::CANCELED); EXPECT_EQ(c.finished,2);
}
} // namespace
} // namespace agv_bridge
