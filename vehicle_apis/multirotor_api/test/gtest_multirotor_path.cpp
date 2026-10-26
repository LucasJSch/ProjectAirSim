// Copyright (C) 2026 IAMAI CONSULTING CORP
// MIT License.

#include <functional>
#include <memory>
#include <vector>

#include "gtest/gtest.h"
#include "simple_flight_api.hpp"
#include "test_robot.hpp"

namespace projectairsim = microsoft::projectairsim;

namespace {
using projectairsim::Vector3;

class PathTestClock : public projectairsim::ClockBase {
 public:
  TimeNano NowSimNanos() const override { return now; }
  TimeNano now = 0;
};

// Exercise the production path follower, replacing only the vehicle and clock.
// Advance beyond one command interval synchronously, so no sleeping or physics
// thread is needed and stalled-vehicle tests still reach their timeout.
class PathTestApi : public projectairsim::SimpleFlightApi {
 public:
  explicit PathTestApi(PathTestClock& clock) : clock_(clock) {
    sim_robot_ = projectairsim::Scene::MakeRobot("PathTest");
  }

  Vector3 position = Vector3::Zero();
  float accuracy = 0.5f;
  std::vector<Vector3> commands;
  std::function<void(PathTestApi&, const Vector3&)> respond;

  bool Follow(const std::vector<Vector3>& path, float timeout = 20,
              float lookahead = -1, float adaptive = 1) {
    return MoveOnPathInternal(path, 0.5f, timeout,
                             projectairsim::DrivetrainType::MaxDegreeOfFreedom,
                             true, 0, lookahead, adaptive, clock_.now);
  }

  void Cancel() { GetCancelToken().Cancel(); }

  projectairsim::Kinematics GetKinematicsEstimated() const override {
    projectairsim::Kinematics state{};
    state.pose.position = position;
    return state;
  }

 protected:
  Vector3 GetPosition() const override { return position; }
  float GetTakeoffZ() const override { return -2.0f; }
  float GetDistanceAccuracy() const override { return accuracy; }
  void CommandVelocity(float vx, float vy, float vz, bool, float) override {
    const Vector3 velocity(vx, vy, vz);
    commands.push_back(velocity);
    if (respond) {
      respond(*this, velocity);
    } else {
      position += velocity * GetCommandPeriod();
    }
    clock_.now += 21'000'000;
  }

 private:
  PathTestClock& clock_;
};

class MultirotorPath : public testing::Test {
 protected:
  void SetUp() override {
    // Restore the default clock after each test; no other fixture is running.
    clock = std::make_shared<PathTestClock>();
    projectairsim::SimClock::Get(clock);
    api = std::make_unique<PathTestApi>(*clock);
  }
  void TearDown() override {
    api.reset();
    projectairsim::SimClock::Get(
        std::make_shared<projectairsim::SteppableClock>());
  }
  std::shared_ptr<PathTestClock> clock;
  std::unique_ptr<PathTestApi> api;
};

TEST_F(MultirotorPath, TakeoffCompletesAtMeasuredEndpointAndStops) {
  EXPECT_TRUE(api->Takeoff(20, clock->now));
  EXPECT_LE((api->position - Vector3(0, 0, -2)).norm(), 0.5f);
  ASSERT_FALSE(api->commands.empty());
  EXPECT_TRUE(api->commands.back().isZero());
}

TEST_F(MultirotorPath, ShortPathStillMovesWhenLookaheadPassesEndpoint) {
  EXPECT_TRUE(api->Follow({Vector3(0, 0, -0.6f)}));
  EXPECT_LE((api->position - Vector3(0, 0, -0.6f)).norm(), 0.5f);
  EXPECT_LT(api->position.z(), 0);
}

TEST_F(MultirotorPath, ExistingAccuracyIncludesBoundary) {
  for (float distance : {0.0f, 0.49f, 0.5f}) {
    api->position.setZero();
    api->commands.clear();
    EXPECT_TRUE(api->Follow({Vector3(0, 0, -distance)}));
    EXPECT_TRUE(api->position.isZero());
    ASSERT_FALSE(api->commands.empty());
    EXPECT_TRUE(api->commands.back().isZero());
  }
}

TEST_F(MultirotorPath, ExactEndpointLookaheadCountsAsFinalTarget) {
  EXPECT_TRUE(api->Follow({Vector3(0, 0, -0.5f)}, 1, 0.5f));
  EXPECT_TRUE(api->position.isZero());
  ASSERT_EQ(api->commands.size(), 1u);
  EXPECT_TRUE(api->commands.back().isZero());
}

TEST_F(MultirotorPath, StalledVehicleTimesOutRatherThanCompleting) {
  api->respond = [](PathTestApi& vehicle, const Vector3&) {
    vehicle.position.z() = -1.3f;
  };
  EXPECT_FALSE(api->Follow({Vector3(0, 0, -2)}, 1));
  EXPECT_GE(clock->now, 1'000'000'000);
  ASSERT_FALSE(api->commands.empty());
  EXPECT_FALSE(api->commands.back().isZero());
}

TEST_F(MultirotorPath, NegativeProgressDoesNotEndCommandEarly) {
  api->respond = [](PathTestApi& vehicle, const Vector3&) {
    if (vehicle.commands.size() == 1) {
      vehicle.position.z() = -1.3f;
    } else {
      vehicle.position.z() += 0.01f;
    }
  };
  EXPECT_FALSE(api->Follow({Vector3(0, 0, -2)}, 1));
  EXPECT_GE(clock->now, 1'000'000'000);
}

TEST_F(MultirotorPath, CancellationDoesNotSendAStopForReplacementTask) {
  api->respond = [](PathTestApi& vehicle, const Vector3&) { vehicle.Cancel(); };
  EXPECT_FALSE(api->Follow({Vector3(0, 0, -2)}));
  ASSERT_EQ(api->commands.size(), 1u);
  EXPECT_FALSE(api->commands.back().isZero());
}

TEST_F(MultirotorPath, CoincidentPathPositionsDoNotHideLateralError) {
  api->respond = [](PathTestApi& vehicle, const Vector3& velocity) {
    if (vehicle.commands.size() == 1) {
      vehicle.position = Vector3(1, 0, -3);  // beyond endpoint, off path
    } else {
      vehicle.position += velocity * 0.02f;
    }
  };
  EXPECT_TRUE(api->Follow({Vector3(0, 0, -2)}));
  EXPECT_LE((api->position - Vector3(0, 0, -2)).norm(), 0.5f);
}

TEST_F(MultirotorPath, MixedAxisPathKeepsVerticalCorrectionOutsideTolerance) {
  EXPECT_TRUE(api->Follow({Vector3(0.4f, 0, -0.4f)}));
  ASSERT_FALSE(api->commands.empty());
  EXPECT_LT(api->commands.front().z(), 0);
  EXPECT_LE((api->position - Vector3(0.4f, 0, -0.4f)).norm(), 0.5f);
}

TEST_F(MultirotorPath, DuplicateWaypointsAndHorizontalPathAtAltitude) {
  api->position = Vector3(0, 0, -100);
  EXPECT_TRUE(api->Follow({api->position, Vector3(2, 0, -100),
                          Vector3(2, 0, -100), Vector3(2, 2, -100)}));
  EXPECT_LE((api->position - Vector3(2, 2, -100)).norm(), 0.5f);
  EXPECT_FLOAT_EQ(api->position.z(), -100);
}

TEST_F(MultirotorPath, EmptyPathRemainsSuccessfulWithoutCommands) {
  EXPECT_TRUE(api->Follow({}));
  EXPECT_TRUE(api->commands.empty());
}

TEST_F(MultirotorPath, MoveToZAndPositionUseControllerAccuracy) {
  api->accuracy = 0.1f;
  EXPECT_TRUE(api->MoveToZ(-2, 0.5f, 20, true, 0, -1, 0, clock->now));
  EXPECT_LE((api->position - Vector3(0, 0, -2)).norm(), api->accuracy);
  EXPECT_TRUE(api->MoveToPosition(
      2, 0, -2, 0.5f, 20,
      projectairsim::DrivetrainType::MaxDegreeOfFreedom, true, 0, -1, 1,
      clock->now));
  EXPECT_LE((api->position - Vector3(2, 0, -2)).norm(), api->accuracy);
}

TEST_F(MultirotorPath, ExpiredCommandDoesNotMoveOrSucceed) {
  EXPECT_FALSE(api->Follow({Vector3(0, 0, -2)}, 0));
  EXPECT_TRUE(api->commands.empty());
}
}  // namespace
