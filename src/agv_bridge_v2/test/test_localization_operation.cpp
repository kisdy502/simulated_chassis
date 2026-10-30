#include <gtest/gtest.h>
#include "agv_bridge_v2/LocalizationOperationTracker.hpp"
#include <thread>

using agv_bridge::LocalizationOperationTracker;
using Status = LocalizationOperationTracker::Status;

class LocalizationOperationTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() { if (!rclcpp::ok()) rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
  std::shared_ptr<rclcpp::Node> node(double timeout = 10.0) {
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("localization_operation_timeout_sec", timeout)});
    return std::make_shared<rclcpp::Node>("operation_test", options);
  }
};

TEST_F(LocalizationOperationTest, FreshOldPoseCannotCompleteAcceptedOrStartingOperation) {
  auto n = node(); LocalizationOperationTracker tracker(n.get());
  Status s;
  EXPECT_FALSE(tracker.lookup("", s));
  const auto id = tracker.begin("LOAD_MAP", "map1");
  tracker.observe(true, true, false);
  ASSERT_TRUE(tracker.lookup(id, s)); EXPECT_FALSE(s.finished);
  tracker.stage(id, "STARTING_LOCALIZATION", "starting");
  tracker.observe(true, true, false);
  ASSERT_TRUE(tracker.lookup(id, s)); EXPECT_FALSE(s.finished);
  tracker.stage(id, "WAITING_LOCALIZATION", "waiting");
  EXPECT_FALSE(tracker.allowsModeCompletion());
  tracker.observe(true, true, true);
  ASSERT_TRUE(tracker.lookup(id, s)); EXPECT_FALSE(s.finished);
  tracker.observe(true, false, false);
  ASSERT_TRUE(tracker.lookup(id, s)); EXPECT_EQ(s.state, "SUCCEEDED");
  EXPECT_TRUE(s.finished); EXPECT_TRUE(s.success); EXPECT_FALSE(s.navigation_ready);
  EXPECT_TRUE(tracker.allowsModeCompletion());
  EXPECT_EQ(s.operation_type, "LOAD_MAP"); EXPECT_EQ(s.map_name, "map1");
}

TEST_F(LocalizationOperationTest, FailureSurvivesAutomaticRecoveryAndLateWorkerUpdates) {
  auto n = node(); LocalizationOperationTracker tracker(n.get()); Status s;
  const auto old = tracker.begin("RELOCALIZE", "map1");
  tracker.fail(old, "launch exited");
  tracker.stage(old, "WAITING_LOCALIZATION", "late callback");
  tracker.observe(true, true, false);
  ASSERT_TRUE(tracker.lookup(old, s)); EXPECT_EQ(s.state, "FAILED");
  EXPECT_EQ(s.message, "launch exited"); EXPECT_FALSE(tracker.allowsModeCompletion());
  const auto next = tracker.begin("LOAD_MAP", "map2");
  tracker.fail(old, "stale failure");
  ASSERT_TRUE(tracker.lookup(next, s)); EXPECT_EQ(s.state, "RUNNING");
  EXPECT_NE(next, old); ASSERT_TRUE(tracker.lookup(old, s)); EXPECT_EQ(s.message, "launch exited");
}

TEST_F(LocalizationOperationTest, TimeoutWinsEvenIfPoseBecomesReadyAndCannotBeRewritten) {
  auto n = node(0.01); LocalizationOperationTracker tracker(n.get()); Status s;
  const auto id = tracker.begin("RELOCALIZE", "map1");
  tracker.stage(id, "WAITING_LOCALIZATION", "waiting");
  std::this_thread::sleep_for(std::chrono::milliseconds(25));
  tracker.observe(true, true, false);
  ASSERT_TRUE(tracker.lookup(id, s)); EXPECT_EQ(s.state, "TIMED_OUT");
  EXPECT_TRUE(s.finished); EXPECT_FALSE(s.success); EXPECT_FALSE(tracker.active());
  EXPECT_FALSE(tracker.allowsModeCompletion());
  const auto elapsed = s.elapsed_sec;
  tracker.fail(id, "late failure"); tracker.observe(true, true, false);
  ASSERT_TRUE(tracker.lookup(id, s)); EXPECT_EQ(s.state, "TIMED_OUT"); EXPECT_EQ(s.elapsed_sec, elapsed);
}

TEST_F(LocalizationOperationTest, DuplicateAdmissionAndUnknownIdAreExplicit) {
  auto n = node(); LocalizationOperationTracker tracker(n.get()); Status s;
  const auto id = tracker.begin("LOAD_MAP", "map1");
  EXPECT_THROW(tracker.begin("LOAD_MAP", "map2"), std::logic_error);
  EXPECT_FALSE(tracker.lookup("does-not-exist", s));
  ASSERT_TRUE(tracker.lookup("", s)); EXPECT_EQ(s.operation_id, id);
  tracker.failActive("bridge shutting down");
  ASSERT_TRUE(tracker.lookup(id, s)); EXPECT_EQ(s.state, "FAILED");
}

TEST_F(LocalizationOperationTest, HistoryIsBoundedAndLatestIsQueryable) {
  auto n = node(); LocalizationOperationTracker tracker(n.get()); Status s;
  std::string first, latest;
  for (int i = 0; i < 129; ++i) {
    latest = tracker.begin("LOAD_MAP", "map" + std::to_string(i));
    if (i == 0) first = latest;
    tracker.fail(latest, "test");
  }
  EXPECT_FALSE(tracker.lookup(first, s));
  ASSERT_TRUE(tracker.lookup("", s)); EXPECT_EQ(s.operation_id, latest);
}

TEST_F(LocalizationOperationTest, QueryAndLateSubscriberRecoverResultWithoutModeEdge) {
  auto n = node(); LocalizationOperationTracker tracker(n.get());
  const auto id = tracker.begin("LOAD_MAP", "map1");
  tracker.fail(id, "sensor missing");
  auto reader = std::make_shared<rclcpp::Node>("operation_result_reader");
  Status received; bool got = false;
  auto sub = reader->create_subscription<Status>("/agv/localization_operation",
      rclcpp::QoS(1).reliable().transient_local(),
      [&](Status::ConstSharedPtr msg) { received = *msg; got = true; });
  auto client = reader->create_client<LocalizationOperationTracker::Query>("/agv/get_localization_operation");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(n); executor.add_node(reader);
  auto end = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while ((!got || !client->service_is_ready()) && std::chrono::steady_clock::now() < end) {
    executor.spin_some(); std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_TRUE(got); EXPECT_EQ(received.operation_id, id); EXPECT_EQ(received.message, "sensor missing");
  ASSERT_TRUE(client->service_is_ready());
  auto request = std::make_shared<LocalizationOperationTracker::Query::Request>();
  request->operation_id = id;
  auto future = client->async_send_request(request);
  ASSERT_EQ(executor.spin_until_future_complete(future, std::chrono::seconds(5)), rclcpp::FutureReturnCode::SUCCESS);
  const auto result = future.get(); EXPECT_TRUE(result->found);
  EXPECT_EQ(result->operation.state, "FAILED"); EXPECT_EQ(result->operation.operation_id, id);
}
