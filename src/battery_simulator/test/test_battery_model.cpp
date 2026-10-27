#include "battery_simulator/model.hpp"
#include <gtest/gtest.h>
#include <limits>
using battery_simulator::Model;
TEST(BatteryModel, DischargeDependsOnMotionAndElapsedTime) {
  Model idle, moving;
  for (int i=0;i<3600;++i) { idle.advance(1,false); moving.advance(1,true); }
  EXPECT_NEAR(idle.soc,68,1e-8); EXPECT_NEAR(moving.soc,50,1e-8);
}
TEST(BatteryModel, ChargingFullAndDisconnect) {
  Model m; m.connected=true;
  for (int i=0;i<3600;++i) m.advance(1,false);
  EXPECT_DOUBLE_EQ(m.soc,100); EXPECT_TRUE(m.connected); EXPECT_DOUBLE_EQ(m.rate(false),0);
  m.connected=false; m.advance(10,false); EXPECT_LT(m.soc,100);
}
TEST(BatteryModel, MotionDisconnectsAndDischarges) {
  Model m; m.connected=true; m.advance(1,true);
  EXPECT_FALSE(m.connected); EXPECT_LT(m.soc,80);
}
TEST(BatteryModel, ClockPauseResetAndLargeJumpsDoNotChangeSOC) {
  Model m; m.advance(0,false); m.advance(-100,false); m.advance(100000,false);
  EXPECT_DOUBLE_EQ(m.soc,80);
}
TEST(BatteryModel, EmptyBatteryDoesNotGoNegative) {
  Model m; m.soc=.001; m.advance(30,true); EXPECT_DOUBLE_EQ(m.soc,0);
  m.connected=true; m.advance(1,false); EXPECT_GT(m.soc,0);
}
TEST(BatteryModel, RejectInvalidConfiguration) {
  Model m; m.charge_rate=0; EXPECT_THROW(m.validate(),std::invalid_argument);
  m.charge_rate=40; m.soc=std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(m.validate(),std::invalid_argument);
}
TEST(BatteryModel, RequestedMinuteRates) {
  Model charging, idle, moving;
  charging.connected=true;
  for(int i=0;i<2;++i) charging.advance(30,false);
  for(int i=0;i<10;++i) idle.advance(30,false);
  for(int i=0;i<4;++i) moving.advance(30,true);
  EXPECT_NEAR(charging.soc,81,1e-8);  // +1% per minute
  EXPECT_NEAR(idle.soc,79,1e-8);      // -1% per five minutes
  EXPECT_NEAR(moving.soc,79,1e-8);    // -1% per two minutes
}
