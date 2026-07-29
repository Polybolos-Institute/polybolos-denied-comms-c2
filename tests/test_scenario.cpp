#include <gtest/gtest.h>

#include "sim/scenario_generator.h"

using namespace Polybolos::Fusion::Sim;

TEST(ScenarioGenerator, ProducesMultiSensorFrames) {
    auto cfg = PredefinedScenarios::straightLineCrossing();
    cfg.end_time = 2.0;
    cfg.dt = 0.1;
    ScenarioGenerator gen(cfg);
    auto out = gen.generateScenario();
    EXPECT_FALSE(out.ground_truth.empty());
    int radar = 0, rf = 0, opt = 0, valid = 0;
    for (const auto& m : out.measurements) {
        if (!m.is_valid) {
            continue;
        }
        ++valid;
        if (m.sensor_type == GeneratedMeasurement::SensorType::RADAR) {
            ++radar;
        } else if (m.sensor_type == GeneratedMeasurement::SensorType::RF) {
            ++rf;
        } else {
            ++opt;
        }
    }
    EXPECT_GT(valid, 0);
    EXPECT_GT(radar + rf + opt, 0);
}

TEST(ScenarioGenerator, SeededReproducible) {
    auto cfg = PredefinedScenarios::loiteringTarget();
    cfg.end_time = 1.0;
    cfg.dt = 0.1;
    ScenarioGenerator a(cfg);
    ScenarioGenerator b(cfg);
    auto oa = a.generateScenario();
    auto ob = b.generateScenario();
    ASSERT_EQ(oa.measurements.size(), ob.measurements.size());
    ASSERT_EQ(oa.ground_truth.size(), ob.ground_truth.size());
    for (size_t i = 0; i < oa.ground_truth.size(); ++i) {
        EXPECT_DOUBLE_EQ(oa.ground_truth[i].latitude, ob.ground_truth[i].latitude);
    }
    for (size_t i = 0; i < oa.measurements.size(); ++i) {
        EXPECT_EQ(oa.measurements[i].is_valid, ob.measurements[i].is_valid);
        if (oa.measurements[i].sensor_type == GeneratedMeasurement::SensorType::RADAR &&
            oa.measurements[i].is_valid) {
            EXPECT_DOUBLE_EQ(oa.measurements[i].radar.range, ob.measurements[i].radar.range);
        }
    }
}

TEST(ScenarioGenerator, InterpolatesWaypoints) {
    ScenarioConfig cfg;
    cfg.target_waypoints = {
        GroundTruthTarget{0.0, 0, 0, 100, 0, 0, 0, 0, 0, 0},
        GroundTruthTarget{10.0, 100, 0, 200, 10, 0, 0, 0, 0, 0},
    };
    ScenarioGenerator gen(cfg);
    auto mid = gen.getGroundTruthAtTime(5.0);
    EXPECT_NEAR(mid.latitude, 50.0, 1e-9);
    EXPECT_NEAR(mid.altitude, 150.0, 1e-9);
}

TEST(PredefinedScenarios, AllConfigsHaveDuration) {
    EXPECT_GT(PredefinedScenarios::straightLineCrossing().end_time, 0.0);
    EXPECT_GT(PredefinedScenarios::loiteringTarget().end_time, 0.0);
    EXPECT_GT(PredefinedScenarios::evasiveManeuver().end_time, 0.0);
    EXPECT_GT(PredefinedScenarios::fastIntercept().end_time, 0.0);
    EXPECT_GT(PredefinedScenarios::lowAltitudeTarget().end_time, 0.0);
    EXPECT_EQ(PredefinedScenarios::multipleTargets(3).num_targets, 3);
}
