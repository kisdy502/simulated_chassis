#include "agv_bridge_v2/BatteryGateway.hpp"
#include <gtest/gtest.h>
#include <atomic>
#include <thread>

using agv_bridge::BatteryGateway;
using B = sensor_msgs::msg::BatteryState;
class BatteryGatewayTest : public testing::Test {
 protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
  void SetUp() override {
    node = std::make_shared<rclcpp::Node>("battery_gateway_test");
    gateway = std::make_shared<BatteryGateway>(node.get(), [this] { return busy.load(); }, []{});
    pub = node->create_publisher<B>("/battery_state",10);
    service = node->create_service<std_srvs::srv::SetBool>("/battery/set_charging",
      [this](std_srvs::srv::SetBool::Request::SharedPtr req,
             std_srvs::srv::SetBool::Response::SharedPtr res) {
        std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms.load()));
        ++calls; requested=req->data; res->success=accept.load(); res->message="backend reply";
      });
    executor.add_node(node); worker=std::thread([this]{ executor.spin(); });
    auto probe = node->create_client<std_srvs::srv::SetBool>("/battery/set_charging");
    ASSERT_TRUE(probe->wait_for_service(std::chrono::seconds(3)));
  }
  void TearDown() override {
    executor.cancel(); if(worker.joinable()) worker.join(); executor.remove_node(node); gateway.reset();
  }
  bool wait(const std::function<bool()> &fn) {
    auto end=std::chrono::steady_clock::now()+std::chrono::seconds(4);
    while(std::chrono::steady_clock::now()<end) {
      if(fn()) return true;
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return false;
  }
  void publish(double percentage,uint8_t state) {
    B msg; msg.percentage=percentage; msg.power_supply_status=state; msg.present=true;
    ASSERT_TRUE(wait([&]{ pub->publish(msg); agv_bridge_v2_interfaces::msg::AgvStatus s; gateway->fill(s); return s.battery_data_valid; }));
  }
  rclcpp::Node::SharedPtr node;
  std::shared_ptr<BatteryGateway> gateway;
  rclcpp::Publisher<B>::SharedPtr pub;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service;
  rclcpp::executors::MultiThreadedExecutor executor{rclcpp::ExecutorOptions(),3};
  std::thread worker;
  std::atomic<bool> busy{false}, accept{true}, requested{true};
  std::atomic<int> calls{0};
  std::atomic<int> delay_ms{0};
};
TEST_F(BatteryGatewayTest, StopsChargingBeforeAllowingMotion) {
  publish(.5,B::POWER_SUPPLY_STATUS_CHARGING);
  std::string error; ASSERT_TRUE(wait([this]{ return service->get_service_name()!=nullptr; }));
  ASSERT_TRUE(gateway->prepareMotion(error));
  EXPECT_EQ(calls.load(),1); EXPECT_FALSE(requested.load());
}
TEST_F(BatteryGatewayTest, BackendRejectionBlocksMotion) {
  publish(.5,B::POWER_SUPPLY_STATUS_CHARGING); accept=false;
  std::string error; EXPECT_FALSE(gateway->prepareMotion(error));
  EXPECT_NE(error.find("停止充电失败"),std::string::npos);
  EXPECT_NE(error.find("backend reply"),std::string::npos);
}
TEST_F(BatteryGatewayTest, FullStillDisconnectsBeforeMovement) {
  publish(1.0,B::POWER_SUPPLY_STATUS_FULL);
  agv_bridge_v2_interfaces::msg::AgvStatus msg; gateway->fill(msg);
  EXPECT_EQ(msg.battery_state,"FULL"); EXPECT_FALSE(msg.charging);
  std::string error; EXPECT_TRUE(gateway->prepareMotion(error)); EXPECT_EQ(calls.load(),1);
}
TEST_F(BatteryGatewayTest, EmptyBatteryBlocksMotion) {
  publish(0,B::POWER_SUPPLY_STATUS_DISCHARGING);
  std::string error; EXPECT_FALSE(gateway->prepareMotion(error)); EXPECT_EQ(calls.load(),0);
}
TEST_F(BatteryGatewayTest, StatusUsesStandardFractionAndSeparateState) {
  publish(.375,B::POWER_SUPPLY_STATUS_DISCHARGING);
  agv_bridge_v2_interfaces::msg::AgvStatus msg; gateway->fill(msg);
  EXPECT_NEAR(msg.battery,37.5,1e-6); EXPECT_TRUE(msg.battery_data_valid);
  EXPECT_EQ(msg.battery_state,"DISCHARGING");
}
TEST_F(BatteryGatewayTest, ChargingRequestDuringTaskIsRejected) {
  busy=true;
  auto client=node->create_client<std_srvs::srv::SetBool>("/agv/set_charging");
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(3)));
  auto req=std::make_shared<std_srvs::srv::SetBool::Request>(); req->data=true;
  auto future=client->async_send_request(req);
  ASSERT_TRUE(wait([&]{ return future.wait_for(std::chrono::seconds(0))==std::future_status::ready; }));
  EXPECT_FALSE(future.get()->success); EXPECT_EQ(calls.load(),0);
}
TEST_F(BatteryGatewayTest, StaleBatteryBlocksNewMotion) {
  publish(.5,B::POWER_SUPPLY_STATUS_DISCHARGING);
  std::this_thread::sleep_for(std::chrono::milliseconds(5100));
  std::string error; EXPECT_FALSE(gateway->prepareMotion(error));
  EXPECT_NE(error.find("有效期"),std::string::npos); EXPECT_EQ(calls.load(),0);
}
TEST_F(BatteryGatewayTest, ControlTimeoutDoesNotAllowMotion) {
  publish(.5,B::POWER_SUPPLY_STATUS_CHARGING); delay_ms=2300;
  std::string error; EXPECT_FALSE(gateway->prepareMotion(error));
  EXPECT_NE(error.find("未确认"),std::string::npos);
  publish(.5,B::POWER_SUPPLY_STATUS_CHARGING);
  EXPECT_FALSE(gateway->prepareMotion(error));
  EXPECT_NE(error.find("data=false"),std::string::npos);
}
