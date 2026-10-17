#include "agv_bridge_v2/NavigationManager.hpp"
#include <gtest/gtest.h>
#include <atomic>
#include <thread>

namespace agv_bridge {
struct NavigationManagerTestPeer {
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
  int results = 0; std::string message;
  void onNavigationStateChanged(NavigationState, const std::string &) override {}
  void onNavigationFeedback(double, double, double) override {}
  void onNavigationResult(bool, const std::string &text, const std::string &, const std::string &) override {
    ++results; message = text;
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
      [](auto, auto) { return rclcpp_action::GoalResponse::REJECT; },
      [](auto) { return rclcpp_action::CancelResponse::ACCEPT; }, [](auto) {});
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
  EXPECT_EQ(callbacks.results, 1);
  EXPECT_NE(callbacks.message.find("velocity_smoother"), std::string::npos);
  EXPECT_EQ(manager->getCurrentState(), NavigationState::IDLE);
}

TEST_F(NavigationReadinessTest, ServerShutdownClearsPendingTaskWithoutReplayingIt) {
  smoother_active = true; ASSERT_TRUE(waitFor([this] { return manager->isReady(); }));
  NavigationManagerTestPeer::resetPendingTask(*manager);
  EXPECT_EQ(callbacks.results, 1);
  EXPECT_NE(callbacks.message.find("重新下发任务"), std::string::npos);
  EXPECT_EQ(spin_goals.load(), 0);
}
} // namespace agv_bridge
