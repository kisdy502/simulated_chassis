#include "agv_bridge_v2/NavigationManager.hpp"
#include <gtest/gtest.h>
#include <atomic>
#include <thread>
#include <cmath>
#include <limits>
#include "agv_bridge_v2/NavigationValidation.hpp"

namespace agv_bridge {
struct NavigationManagerTestPeer {
  static void timeoutAction(NavigationManager &m, bool cancel) {
    std::lock_guard<std::recursive_mutex> lock(m.mutex_);
    MoveToMessage task; task.commandId = "timed-out-goal"; m.current_task_ = task;
    ++m.action_generation_;
    m.awaiting_response_ = !cancel;
    m.cancel_pending_ = cancel;
    m.action_deadline_ = std::chrono::steady_clock::now() - std::chrono::seconds(1);
    m.checkActionProgress();
  }
  static bool navAccepted(NavigationManager &m) {
    std::lock_guard<std::recursive_mutex> lock(m.mutex_);
    return bool(m.nav_goal_handle_);
  }
  static void expireSmoother(NavigationManager &m) {
    std::lock_guard<std::recursive_mutex> lock(m.mutex_);
    for (auto &c : m.lifecycle_checks_) if (c.name == "velocity_smoother") {
      c.responded = std::chrono::steady_clock::now() - std::chrono::seconds(5);
      c.checked = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    }
  }
  static bool submitSpin(NavigationManager &m) {
    std::lock_guard<std::recursive_mutex> lock(m.mutex_);
    MoveToMessage task; task.commandId = "test-goal"; m.current_task_ = task;
    m.setState(NavigationState::PRE_ROTATING);
    return m.dispatchGoal<Spin>(m.spin_client_, Spin::Goal(), {});
  }
  static void resetPendingTask(NavigationManager &m) {
    std::lock_guard<std::recursive_mutex> lock(m.mutex_);
    MoveToMessage task; task.commandId = "old-goal"; m.current_task_ = task;
    m.unsafe_generation_ = ++m.action_generation_;
    m.awaiting_response_ = true;
    m.resetAfterServerShutdown();
    EXPECT_FALSE(m.current_task_);
    EXPECT_FALSE(m.awaiting_response_);
    EXPECT_EQ(m.unsafe_generation_, 0u);
    for (const auto &c : m.lifecycle_checks_) EXPECT_FALSE(c.active);
  }
};

struct TestCallbacks : NavigationCallbacks {
  std::atomic<int> results{0}; std::string message;
  std::string command_id, node_id; bool success = false;
  void onNavigationStateChanged(NavigationState, const std::string &) override {}
  void onNavigationFeedback(double, double, double) override {}
  void onNavigationResult(bool ok, const std::string &text, const std::string &command, const std::string &node) override {
    message = text; command_id = command; node_id = node; success = ok; ++results;
  }
  void onCommandAck(const std::string &, const std::string &, const std::string &, const std::string &) override {}
};

class NavigationReadinessTest : public ::testing::Test {
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
  void SetUp() override {
    node = std::make_shared<rclcpp::Node>("navigation_readiness_test");
    for (auto name : {"controller_server", "smoother_server", "planner_server", "behavior_server",
                      "bt_navigator", "waypoint_follower", "velocity_smoother"}) {
      const std::string component(name);
      services.push_back(node->create_service<lifecycle_msgs::srv::GetState>(component + "/get_state",
        [this, component](std::shared_ptr<lifecycle_msgs::srv::GetState::Request>,
                          std::shared_ptr<lifecycle_msgs::srv::GetState::Response> reply) {
          reply->current_state.id = component == "velocity_smoother" && !smoother_active ? 2 : 3;
        }));
    }
    spin_server = rclcpp_action::create_server<Spin>(node, "spin",
      [this](auto, auto) { ++spin_goals; return rclcpp_action::GoalResponse::REJECT; },
      [](auto) { return rclcpp_action::CancelResponse::ACCEPT; }, [](auto) {});
    path_server = rclcpp_action::create_server<FollowPath>(node, "follow_path",
      [](auto, auto) { return rclcpp_action::GoalResponse::REJECT; },
      [](auto) { return rclcpp_action::CancelResponse::ACCEPT; }, [](auto) {});
    nav_server = rclcpp_action::create_server<NavigationManager::NavigateToPose>(node, "navigate_to_pose",
      [this](auto, auto) { ++nav_goals; return accept_nav ?
        rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE : rclcpp_action::GoalResponse::REJECT; },
      [](auto) { return rclcpp_action::CancelResponse::ACCEPT; },
      [this](auto handle) {
        std::lock_guard<std::mutex> lock(nav_mutex);
        nav_handle = handle;
      });
    executor.add_node(node); worker = std::thread([this] { executor.spin(); });
    manager = std::make_shared<NavigationManager>(node.get(), &callbacks, nullptr);
    ASSERT_TRUE(manager->initialize());
  }
  void TearDown() override {
    executor.cancel(); if (worker.joinable()) worker.join(); manager.reset(); executor.remove_node(node);
  }
  bool waitFor(const std::function<bool()> &predicate) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(8);
    while (std::chrono::steady_clock::now() < deadline) {
      if (predicate()) return true;
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return false;
  }
  using NavHandle = rclcpp_action::ServerGoalHandle<NavigationManager::NavigateToPose>;
  std::shared_ptr<NavHandle> navigationHandle() {
    std::lock_guard<std::mutex> lock(nav_mutex); return nav_handle;
  }
  std::mutex nav_mutex;
  std::shared_ptr<NavHandle> nav_handle;
  std::atomic<bool> accept_nav{false};
  std::atomic<int> nav_goals{0};
  std::atomic<bool> smoother_active{false}; std::atomic<int> spin_goals{0};
  rclcpp::Node::SharedPtr node;
  std::vector<rclcpp::Service<lifecycle_msgs::srv::GetState>::SharedPtr> services;
  rclcpp_action::Server<Spin>::SharedPtr spin_server;
  rclcpp_action::Server<FollowPath>::SharedPtr path_server;
  rclcpp_action::Server<NavigationManager::NavigateToPose>::SharedPtr nav_server;
  rclcpp::executors::MultiThreadedExecutor executor{rclcpp::ExecutorOptions(), 2};
  std::thread worker;
  TestCallbacks callbacks;
  std::shared_ptr<NavigationManager> manager;
};

TEST_F(NavigationReadinessTest, InactiveVelocitySmootherBlocksTaskAndRecoversWhenActive) {
  ASSERT_TRUE(waitFor([this] { return manager->readinessError().find("velocity_smoother") != std::string::npos; }));
  EXPECT_FALSE(manager->isReady());
  EXPECT_FALSE(manager->followPathNavigation(MoveToMessage{}));
  smoother_active = true;
  ASSERT_TRUE(waitFor([this] { return manager->isReady(); }));
  smoother_active = false;
  ASSERT_TRUE(waitFor([this] { return !manager->isStackActive(); }));
}

TEST_F(NavigationReadinessTest, StaleLifecycleResponseIsNotReady) {
  smoother_active = true; ASSERT_TRUE(waitFor([this] { return manager->isReady(); }));
  NavigationManagerTestPeer::expireSmoother(*manager);
  EXPECT_FALSE(manager->isReady());
  EXPECT_NE(manager->readinessError().find("velocity_smoother"), std::string::npos);
}

TEST_F(NavigationReadinessTest, DoesNotSendRotationWhenTranslationChainIsInactive) {
  ASSERT_TRUE(waitFor([this] { return manager->readinessError().find("velocity_smoother") != std::string::npos; }));
  EXPECT_FALSE(NavigationManagerTestPeer::submitSpin(*manager));
  EXPECT_EQ(spin_goals.load(), 0);
  EXPECT_EQ(callbacks.results.load(), 1);
  EXPECT_NE(callbacks.message.find("velocity_smoother"), std::string::npos);
  EXPECT_EQ(manager->getCurrentState(), NavigationState::IDLE);
}

TEST_F(NavigationReadinessTest, ServerShutdownClearsPendingTaskWithoutReplayingIt) {
  smoother_active = true; ASSERT_TRUE(waitFor([this] { return manager->isReady(); }));
  NavigationManagerTestPeer::resetPendingTask(*manager);
  EXPECT_EQ(callbacks.results.load(), 1);
  EXPECT_NE(callbacks.message.find("重新下发任务"), std::string::npos);
  EXPECT_EQ(spin_goals.load(), 0);
}

TEST_F(NavigationReadinessTest, CancelTimeoutRequiresRestartEvenWithActiveServers) {
  smoother_active = true;
  ASSERT_TRUE(waitFor([this] { return manager->isReady(); }));
  NavigationManagerTestPeer::timeoutAction(*manager, true);
  EXPECT_TRUE(manager->isStackActive());
  EXPECT_TRUE(manager->requiresServerRestart());
  EXPECT_FALSE(manager->isReady());
  EXPECT_EQ(spin_goals.load(), 0);
  EXPECT_NE(callbacks.message.find("取消结果等待超时"), std::string::npos);
  // Lifecycle polling alone cannot prove the old action stopped.
  ASSERT_TRUE(waitFor([this] { return manager->isStackActive(); }));
  EXPECT_TRUE(manager->requiresServerRestart());
  manager->resetAfterServerShutdown();
  EXPECT_FALSE(manager->requiresServerRestart());
  EXPECT_FALSE(manager->isReady());
  ASSERT_TRUE(waitFor([this] { return manager->isReady(); }));
  EXPECT_EQ(spin_goals.load(), 0);  // no automatic replay
  ASSERT_TRUE(NavigationManagerTestPeer::submitSpin(*manager));
  ASSERT_TRUE(waitFor([this] { return spin_goals.load() == 1; }));
}

TEST_F(NavigationReadinessTest, GoalResponseTimeoutAlsoRequiresRestart) {
  smoother_active = true;
  ASSERT_TRUE(waitFor([this] { return manager->isReady(); }));
  NavigationManagerTestPeer::timeoutAction(*manager, false);
  EXPECT_TRUE(manager->requiresServerRestart());
  EXPECT_TRUE(manager->isStackActive());
  EXPECT_FALSE(manager->isReady());
  EXPECT_NE(callbacks.message.find("目标响应超时"), std::string::npos);
  manager->resetAfterServerShutdown();
  EXPECT_FALSE(manager->requiresServerRestart());
  ASSERT_TRUE(waitFor([this] { return manager->isReady(); }));
}

TEST_F(NavigationReadinessTest, FreeNavigationRequiresReadyStack) {
  EXPECT_FALSE(manager->startNavigation("free", "target", 2.0, 3.0, 0.4));
  EXPECT_EQ(nav_goals.load(), 0);
}

TEST_F(NavigationReadinessTest, FreeNavigationPreservesGoalAndCompletesTask) {
  smoother_active = true; accept_nav = true;
  ASSERT_TRUE(waitFor([this] { return manager->isReady(); }));
  MoveToMessage task; task.commandId = "free-success"; task.nodeId = "destination";
  task.x = 2.0; task.y = -3.0; task.theta = 0.4;
  task.edgeInfo.type = EdgeType::FREE; task.edgeInfo.maxSpeed = 0.6;
  ASSERT_TRUE(manager->startNavigation(task));
  ASSERT_TRUE(waitFor([this] { return bool(navigationHandle()); }));
  auto handle = navigationHandle();
  const auto goal = handle->get_goal();
  EXPECT_EQ(goal->pose.header.frame_id, "map");
  EXPECT_DOUBLE_EQ(goal->pose.pose.position.x, 2.0);
  EXPECT_DOUBLE_EQ(goal->pose.pose.position.y, -3.0);
  EXPECT_NEAR(goal->pose.pose.orientation.z, std::sin(0.2), 1e-9);
  EXPECT_TRUE(goal->behavior_tree.empty());
  EXPECT_EQ(spin_goals.load(), 0);
  handle->succeed(std::make_shared<NavigationManager::NavigateToPose::Result>());
  ASSERT_TRUE(waitFor([this] { return callbacks.results.load() == 1; }));
  EXPECT_TRUE(callbacks.success);
  EXPECT_EQ(callbacks.command_id, "free-success");
  EXPECT_EQ(callbacks.node_id, "destination");
  ASSERT_TRUE(waitFor([this] { return manager->getCurrentState() == NavigationState::IDLE; }));
}

TEST_F(NavigationReadinessTest, FreeNavigationRejectionReleasesTask) {
  smoother_active = true;
  ASSERT_TRUE(waitFor([this] { return manager->isReady(); }));
  ASSERT_TRUE(manager->startNavigation("free-rejected", "target", 1.0, 0.0, 0.0));
  ASSERT_TRUE(waitFor([this] { return callbacks.results.load() == 1; }));
  EXPECT_FALSE(callbacks.success);
  EXPECT_EQ(callbacks.command_id, "free-rejected");
  EXPECT_FALSE(manager->requiresServerRestart());
}

TEST_F(NavigationReadinessTest, FreeNavigationCancelConfirmsStopAndAllowsNewTask) {
  smoother_active = true; accept_nav = true;
  ASSERT_TRUE(waitFor([this] { return manager->isReady(); }));
  ASSERT_TRUE(manager->startNavigation("free-cancel", "target", 1.0, 0.0, 0.0));
  ASSERT_TRUE(waitFor([this] { return bool(navigationHandle()); }));
  // Wait for the client's accepted callback before requesting cancellation.
  ASSERT_TRUE(waitFor([this] { return manager->getCurrentState() == NavigationState::EXECUTING; }));
  ASSERT_TRUE(waitFor([this] { return NavigationManagerTestPeer::navAccepted(*manager); }));
  auto handle = navigationHandle();
  ASSERT_TRUE(manager->cancelNavigation());
  ASSERT_TRUE(waitFor([handle] { return handle->is_canceling(); }));
  handle->canceled(std::make_shared<NavigationManager::NavigateToPose::Result>());
  ASSERT_TRUE(waitFor([this] { return manager->getCurrentState() == NavigationState::IDLE; }));
  EXPECT_EQ(callbacks.results.load(), 1);
  EXPECT_FALSE(manager->requiresServerRestart());
  accept_nav = false;
  ASSERT_TRUE(manager->startNavigation("free-next", "target", 2.0, 0.0, 0.0));
  ASSERT_TRUE(waitFor([this] { return callbacks.results.load() == 2; }));
}

TEST_F(NavigationReadinessTest, FreeNavigationRejectsConflictingPathOptions) {
  smoother_active = true;
  ASSERT_TRUE(waitFor([this] { return manager->isReady(); }));
  MoveToMessage task; task.commandId = "invalid-free"; task.edgeInfo.backUp = true;
  EXPECT_FALSE(manager->startNavigation(task));
  EXPECT_EQ(callbacks.results.load(), 1);
  EXPECT_EQ(nav_goals.load(), 0);
}

TEST(NavigationValidationTest, TypeMustBeExplicitAndKnown) {
  std::vector<ControlPoint> points;
  for (const auto &type : {"", "  ", "free", "UNKNOWN"}) {
    const auto error = validateNavigationMode(type, "E1", points, false, false);
    EXPECT_NE(error.find("edge_type"), std::string::npos);
  }
}

TEST(NavigationValidationTest, EdgeIdRequiredExceptFree) {
  std::vector<ControlPoint> points;
  for (const auto &type : {"STRAIGHT", "CURVE", "ELEVATION"}) {
    for (const auto &id : {"", " ", "\t\r\n"}) {
      const auto error = validateNavigationMode(type, id, points, false, false);
      EXPECT_NE(error.find("edge_id"), std::string::npos);
    }
  }
  for (const auto &id : {"", " ", "unused"})
    EXPECT_TRUE(validateNavigationMode("FREE", id, points, false, false).empty());
}

TEST(NavigationValidationTest, CurveNeedsExactlyOneOrTwoFiniteControlPoints) {
  for (std::size_t count : {0u, 1u, 2u, 3u, 4u}) {
    std::vector<ControlPoint> points(count, ControlPoint(1.0, 2.0));
    const auto error = validateNavigationMode("CURVE", "E1", points, false, false);
    if (count == 1 || count == 2) EXPECT_TRUE(error.empty());
    else {
      EXPECT_NE(error.find("control_points"), std::string::npos);
      EXPECT_NE(error.find(std::to_string(count)), std::string::npos);
    }
  }
  std::vector<ControlPoint> points{{1.0, 2.0}, {3.0, 4.0}};
  points[1].x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_NE(validateNavigationMode("CURVE", "E1", points, false, false).find("control_points[1].x"), std::string::npos);
  points[1].x = 3.0;
  points[0].y = std::numeric_limits<double>::infinity();
  EXPECT_NE(validateNavigationMode("CURVE", "E1", points, false, false).find("control_points[0].y"), std::string::npos);
}

TEST(NavigationValidationTest, NonCurveModesForbidControlPoints) {
  std::vector<ControlPoint> points{{1.0, 2.0}};
  for (const auto &type : {"STRAIGHT", "ELEVATION", "FREE"})
    EXPECT_NE(validateNavigationMode(type, "E1", points, false, false).find("control_points"), std::string::npos);
}

TEST(NavigationValidationTest, FreeForbidsReverseAndBackUp) {
  std::vector<ControlPoint> points;
  EXPECT_NE(validateNavigationMode("FREE", "", points, true, false).find("back_up=true"), std::string::npos);
  EXPECT_NE(validateNavigationMode("FREE", "", points, false, true).find("reverse=true"), std::string::npos);
}

TEST_F(NavigationReadinessTest, InvalidCurveFailsBeforeAnyPathOrRotation) {
  smoother_active = true;
  ASSERT_TRUE(waitFor([this] { return manager->isReady(); }));
  MoveToMessage task; task.commandId = "invalid-curve"; task.edgeInfo.id = "E1";
  task.edgeInfo.type = EdgeType::CURVE;
  EXPECT_FALSE(manager->followBezierPathNavigation(task));
  EXPECT_EQ(callbacks.results.load(), 1);
  EXPECT_NE(callbacks.message.find("数量为 0"), std::string::npos);
  EXPECT_EQ(spin_goals.load(), 0);
  EXPECT_EQ(manager->getCurrentState(), NavigationState::IDLE);
}

TEST_F(NavigationReadinessTest, StraightWithoutEdgeIdFailsBeforeAnyMovement) {
  smoother_active = true;
  ASSERT_TRUE(waitFor([this] { return manager->isReady(); }));
  MoveToMessage task; task.commandId = "invalid-straight";
  EXPECT_FALSE(manager->followPathNavigation(task));
  EXPECT_EQ(callbacks.results.load(), 1);
  EXPECT_NE(callbacks.message.find("edge_id"), std::string::npos);
  EXPECT_EQ(spin_goals.load(), 0);
}
} // namespace agv_bridge
