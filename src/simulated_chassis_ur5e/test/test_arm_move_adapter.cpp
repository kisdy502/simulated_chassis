#include "simulated_chassis_ur5e/arm_move_adapter.hpp"
#include <gtest/gtest.h>

#include <chrono>
#include <functional>
#include <limits>
#include <thread>

using namespace std::chrono_literals;
using simulated_chassis_ur5e::ArmMoveAdapter;
using ArmMove = agv_bridge_v2_interfaces::srv::ArmMove;
using ListControllers = controller_manager_msgs::srv::ListControllers;
using JointState = sensor_msgs::msg::JointState;
using Trajectory = trajectory_msgs::msg::JointTrajectory;

namespace
{
const std::vector<std::string> names{
  "ur5e_elbow_joint", "ur5e_shoulder_lift_joint", "ur5e_shoulder_pan_joint",
  "ur5e_wrist_1_joint", "ur5e_wrist_2_joint", "ur5e_wrist_3_joint"};

std::string description()
{
  std::string xml = "<robot name='test_arm'><link name='root'/>";
  std::string parent = "root";
  for (std::size_t i = 0; i < names.size(); ++i) {
    const auto child = "link_" + std::to_string(i);
    xml += "<link name='" + child + "'/><joint name='" + names[i] + "' type='revolute'>" +
      "<parent link='" + parent + "'/><child link='" + child + "'/><axis xyz='0 0 1'/>" +
      "<limit lower='-2' upper='2' velocity='1' effort='10'/></joint>";
    parent = child;
  }
  return xml + "</robot>";
}

class AdapterTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void SetUp() override
  {
    fake_ = std::make_shared<rclcpp::Node>("arm_contract_test");
    state_.name = names;
    state_.position = {0, 0, 0, 0, 0, 0};
    states_ = fake_->create_publisher<JointState>("/joint_states", rclcpp::SensorDataQoS());
    timer_ = fake_->create_wall_timer(20ms, [this]() {
      if (publish_states_) states_->publish(state_);
    });
    controller_ = fake_->create_service<ListControllers>("/controller_manager/list_controllers",
      [this](const ListControllers::Request::SharedPtr, ListControllers::Response::SharedPtr response) {
        ++controller_replies_;
        controller_manager_msgs::msg::ControllerState item;
        item.name = "joint_trajectory_controller";
        item.state = active_ ? "active" : "inactive";
        response->controller.push_back(item);
      });
    trajectories_ = fake_->create_subscription<Trajectory>(
      "/joint_trajectory_controller/joint_trajectory", 10,
      [this](Trajectory::ConstSharedPtr message) {received_.push_back(*message);});
    client_ = fake_->create_client<ArmMove>("/agv/arm_move");
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("robot_description", description())});
    adapter_ = std::make_shared<ArmMoveAdapter>(options);
    executor_.add_node(fake_);
    executor_.add_node(adapter_);
    ASSERT_TRUE(until([this]() {return client_->service_is_ready() && states_->get_subscription_count() > 0;}));
    ASSERT_TRUE(until([this]() {return controller_replies_ > 0;}));
    pump(50ms);  // dispatch the corresponding client response
  }

  void TearDown() override
  {
    executor_.remove_node(adapter_);
    executor_.remove_node(fake_);
    adapter_.reset();
    client_.reset();
    trajectories_.reset();
    controller_.reset();
    timer_.reset();
    states_.reset();
    fake_.reset();
  }

  bool until(const std::function<bool()> & predicate, std::chrono::milliseconds timeout = 4s)
  {
    const auto end = std::chrono::steady_clock::now() + timeout;
    do {
      executor_.spin_some();
      if (predicate()) return true;
      std::this_thread::sleep_for(5ms);
    } while (std::chrono::steady_clock::now() < end);
    return false;
  }

  void pump(std::chrono::milliseconds duration) {until([]() {return false;}, duration);}

  ArmMove::Response::SharedPtr call(std::vector<double> values, double duration = 4, const std::string & arm = "left")
  {
    auto request = std::make_shared<ArmMove::Request>();
    request->arm = arm;
    request->joints = std::move(values);
    request->duration_sec = duration;
    auto future = client_->async_send_request(request);
    if (!until([&future]() {return future.wait_for(0s) == std::future_status::ready;})) {
      ADD_FAILURE() << "arm service response timed out";
      return std::make_shared<ArmMove::Response>();
    }
    return future.get();
  }

  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Node::SharedPtr fake_;
  std::shared_ptr<ArmMoveAdapter> adapter_;
  rclcpp::Publisher<JointState>::SharedPtr states_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<ListControllers>::SharedPtr controller_;
  rclcpp::Subscription<Trajectory>::SharedPtr trajectories_;
  rclcpp::Client<ArmMove>::SharedPtr client_;
  JointState state_;
  bool active_{true};
  std::size_t controller_replies_{0};
  bool publish_states_{true};
  std::vector<Trajectory> received_;
};

TEST_F(AdapterTest, SixAxisLexicalOrderAndImmediateResponse)
{
  const auto begin = std::chrono::steady_clock::now();
  ASSERT_TRUE(call({.1, .2, .3, .4, .5, .6}, 20)->success);
  EXPECT_LT(std::chrono::steady_clock::now() - begin, 1s);
  ASSERT_TRUE(until([this]() {return !received_.empty();}));
  EXPECT_EQ(received_.back().joint_names, names);
  EXPECT_EQ(received_.back().points.back().positions, (std::vector<double>{.1, .2, .3, .4, .5, .6}));
  EXPECT_EQ(received_.back().points.back().time_from_start.sec, 20);
}

TEST_F(AdapterTest, SevenAxisAliasMapsPhysicalOrderAndDisclosesUnrepresentedAxis)
{
  const auto response = call({.1, .2, 1.2, .3, .4, .5, .6}, 4, "right");
  ASSERT_TRUE(response->success);
  EXPECT_NE(response->message.find("joint_3 not simulated"), std::string::npos);
  ASSERT_TRUE(until([this]() {return !received_.empty();}));
  EXPECT_EQ(received_.back().points.back().positions, (std::vector<double>{.3, .2, .1, .4, .5, .6}));
  EXPECT_EQ(received_.back().joint_names.size(), 6u);
}

TEST_F(AdapterTest, RejectsWaistWrongCountAndNonfiniteInputsWithoutMotion)
{
  auto response = call({0, 0}, 3, "waist");
  EXPECT_FALSE(response->success);
  EXPECT_EQ(response->message, "waist not equipped");
  EXPECT_FALSE(call({0, 0, 0, 0, 0})->success);
  EXPECT_FALSE(call({0, 0, 0, 0, 0, std::numeric_limits<double>::quiet_NaN()})->success);
  EXPECT_FALSE(call(std::vector<double>(6), std::numeric_limits<double>::infinity())->success);
  EXPECT_FALSE(call(std::vector<double>(6), 3, "unknown")->success);
  pump(50ms);
  EXPECT_TRUE(received_.empty());
}

TEST_F(AdapterTest, DefaultDurationClippingAndVelocityLimitedDuration)
{
  ASSERT_TRUE(call({.2, .2, .2, .2, .2, .2}, 0)->success);
  ASSERT_TRUE(until([this]() {return !received_.empty();}));
  EXPECT_EQ(received_.back().points.back().time_from_start.sec, 3);
  state_.position.assign(6, -2.0);
  pump(50ms);
  const auto previous = received_.size();
  ASSERT_TRUE(call({100, 100, 100, 100, 100, 100}, .5)->success);
  ASSERT_TRUE(until([this, previous]() {return received_.size() > previous;}));
  EXPECT_EQ(received_.back().points.back().positions, std::vector<double>(6, 2));
  EXPECT_EQ(received_.back().points.back().time_from_start.sec, 4);
}

TEST_F(AdapterTest, RepeatedCommandUsesLatestMeasuredState)
{
  ASSERT_TRUE(call(std::vector<double>(6, 1), 20)->success);
  ASSERT_TRUE(until([this]() {return !received_.empty();}));
  state_.name = {names[5], names[0], names[2], names[1], names[3], names[4]};
  state_.position = {.6, .1, .3, .2, .4, .5};
  pump(50ms);
  ASSERT_TRUE(call(std::vector<double>(7, -.5), 3, "right")->success);
  ASSERT_TRUE(until([this]() {return received_.size() >= 2;}));
  EXPECT_EQ(received_.back().points.front().positions, (std::vector<double>{.1, .2, .3, .4, .5, .6}));
}

TEST_F(AdapterTest, RejectsInactiveControllerAndStaleMeasuredState)
{
  active_ = false;
  pump(650ms);
  auto response = call(std::vector<double>(6));
  EXPECT_FALSE(response->success);
  EXPECT_EQ(response->message, "joint_trajectory_controller not ready");
  active_ = true;
  pump(650ms);
  publish_states_ = false;
  pump(2100ms);
  response = call(std::vector<double>(7));
  EXPECT_FALSE(response->success);
  EXPECT_EQ(response->message, "joint states unavailable or stale");
  EXPECT_TRUE(received_.empty());
}

TEST(ArmMapping, RejectsDuplicateAndOutOfRangeConfiguration)
{
  // Use a separate context so this test does not depend on fixture execution order.
  auto context = std::make_shared<rclcpp::Context>();
  context->init(0, nullptr);
  rclcpp::NodeOptions options;
  options.context(context);
  options.parameter_overrides({rclcpp::Parameter("robot_description", description()),
    rclcpp::Parameter("real_arm_joint_indices", std::vector<int64_t>{0, 0, 1, 2, 3, 4})});
  EXPECT_THROW(std::make_shared<ArmMoveAdapter>(options), std::runtime_error);
  options.parameter_overrides({rclcpp::Parameter("robot_description", description()),
    rclcpp::Parameter("real_arm_joint_indices", std::vector<int64_t>{0, 1, 2, 3, 4, 7})});
  EXPECT_THROW(std::make_shared<ArmMoveAdapter>(options), std::runtime_error);
  context->shutdown("test completed");
}
}  // namespace
