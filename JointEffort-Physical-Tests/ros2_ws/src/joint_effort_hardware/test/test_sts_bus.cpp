#include <gtest/gtest.h>
#include "joint_effort_hardware/sts_bus.hpp"
#include <array>
#include <cmath>
#include <cstdlib>
#include <fcntl.h>
#include <thread>
#include <unistd.h>
using namespace joint_effort_hardware;

Joint fixture() { return {"joint_1",1,777,2048,1,1800,2300,1.,-.3,.3,.02,.02}; }

TEST(Calibration, CountAngleAndExternalRatio) {
  auto j=fixture(); j.validate(); EXPECT_DOUBLE_EQ(j.radians(2048),0.);
  EXPECT_NEAR(j.radians(2112),.09817477042,1e-10);
  j.direction=-1; j.ratio=2; EXPECT_NEAR(j.radians(2112),-.04908738521,1e-10);
  EXPECT_EQ(j.counts(j.radians(2112)),2112);
}
TEST(Calibration, InvalidAndWrappedAndQuantizedLimits) {
  auto j=fixture(); EXPECT_THROW(j.counts(NAN),std::runtime_error);
  EXPECT_THROW(j.radians(4096),std::runtime_error);
  j.zero=0; EXPECT_THROW(j.validate(),std::runtime_error);
  j=fixture(); j.velocity=.0001; EXPECT_THROW(j.validate(),std::runtime_error);
  j=fixture(); j.upper=.001; EXPECT_EQ(j.counts(.001),2048);
}
TEST(Protocol, DocumentedReadFrame) {
  EXPECT_EQ(packet(1,2,{56,2}), (Bytes{255,255,1,4,2,56,2,190}));
  EXPECT_EQ(signed_word(0x8005),-5); EXPECT_EQ(signed_word(4095),4095);
  EXPECT_THROW(packet(255,2,{}),std::runtime_error);
}
TEST(Calibration, ServoCanRebaseOnRoundedMeasuredPose) {
  auto j=fixture();
  EXPECT_NO_THROW(j.check_step(j.radians(j.counts(.01)),.01,j.radians(j.counts(.01)),.02));
  EXPECT_EQ(j.speed(),13);  // Actual motor speed remains below .02 rad/s.
  EXPECT_THROW(j.check_step(.03,.01,.03,.02),std::runtime_error);
  EXPECT_THROW(j.check_step(.03,.03,0.,.02),std::runtime_error);
  EXPECT_THROW(j.check_step(.01,.01,.01,NAN),std::runtime_error);
}
class PseudoPort : public testing::Test {
 protected:
  int master{-1}; std::string path; Bus bus;
  void SetUp() override {
    master=posix_openpt(O_RDWR|O_NOCTTY|O_CLOEXEC); ASSERT_GE(master,0);
    ASSERT_EQ(grantpt(master),0); ASSERT_EQ(unlockpt(master),0);
    path=ptsname(master); bus.open(path,1000000);
  }
  void TearDown() override { bus.close(); if(master>=0) ::close(master); }
  Bytes receive(size_t count) {
    Bytes frame(count); size_t offset=0;
    while(offset<count) {
      const auto n=::read(master,frame.data()+offset,count-offset);
      if(n<=0) break;
      offset+=n;
    }
    frame.resize(offset); return frame;
  }
  void reply(Bytes frame, bool split=false) {
    receive(8);
    if(split) { ::write(master,frame.data(),3); usleep(1000); ::write(master,frame.data()+3,frame.size()-3); }
    else ::write(master,frame.data(),frame.size());
  }
};
TEST_F(PseudoPort, ReadsPartialResponse) {
  std::thread emulator([&]{reply(packet(1,0,{0,8}),true);});
  EXPECT_EQ(bus.read(1,56,2),(Bytes{0,8})); emulator.join();
}
TEST_F(PseudoPort, RejectsChecksum) {
  auto frame=packet(1,0,{0,8}); frame.back()^=1;
  std::thread emulator([&]{reply(frame);});
  EXPECT_THROW(bus.read(1,56,2),std::runtime_error); emulator.join();
}
TEST_F(PseudoPort, RejectsWrongId) {
  std::thread emulator([&]{reply(packet(2,0,{0,8}));});
  EXPECT_THROW(bus.read(1,56,2),std::runtime_error); emulator.join();
}
TEST_F(PseudoPort, RejectsServoFault) {
  std::thread emulator([&]{reply(packet(1,4,{0,8}));});
  EXPECT_THROW(bus.read(1,56,2),std::runtime_error); emulator.join();
}
TEST_F(PseudoPort, RejectsLengthAndNoise) {
  std::thread emulator([&]{reply(packet(1,0,{8}));});
  EXPECT_THROW(bus.read(1,56,2),std::runtime_error); emulator.join();
}
TEST_F(PseudoPort, BoundedTimeout) {
  const auto start=Clock::now(); EXPECT_THROW(bus.read(1,56,2),std::runtime_error);
  EXPECT_LT(std::chrono::duration<double>(Clock::now()-start).count(),.05);
}
TEST_F(PseudoPort, ExclusivePortOwnership) {
  Bus other; EXPECT_THROW(other.open(path,1000000),std::runtime_error);
}
TEST_F(PseudoPort, SyncWriteMatchesPositionBlock) {
  std::vector<Joint> joints(5,fixture());
  for(int i=0;i<5;++i)joints[i].id=i+1;
  bus.goals(joints,{2048,2048,2048,2048,2048});
  const auto frame=receive(48); ASSERT_EQ(frame.size(),48u);
  EXPECT_EQ(frame[2],254); EXPECT_EQ(frame[4],0x83); EXPECT_EQ(frame[5],41); EXPECT_EQ(frame[6],7);
  EXPECT_EQ(frame[7],1); EXPECT_EQ(frame[8],1); EXPECT_EQ(frame[9],0); EXPECT_EQ(frame[10],8);
  EXPECT_EQ(frame[13],13);  // .02 rad/s, floor to counts/s, never zero/unlimited.
  EXPECT_THROW(bus.goals(joints,{0,2048,2048,2048,2048}),std::runtime_error);
}
