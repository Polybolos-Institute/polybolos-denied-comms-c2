#include <gtest/gtest.h>
#include "sim/scenario_generator.h"
#include <cmath>
#include <iostream>
using namespace Polybolos::Fusion::Sim;
class ScenarioGeneratorTest : public ::testing::Test {
protected:
    ScenarioConfig config_ = PredefinedScenarios::straightLineCrossing();
};
// Test 1: Scenario generation
TEST_F(ScenarioGeneratorTest, ScenarioGeneration) {
    ScenarioGenerator gen(config_);
    auto output = gen.generateScenario();
    
    EXPECT_GT(output.ground_truth.size(), 0);
    EXPECT_GT(output.measurements.size(), 0);
}
// Test 2: Ground truth interpolation
TEST_F(ScenarioGeneratorTest, GroundTruthInterpolation) {
    ScenarioGenerator gen(config_);
    
    auto truth_start = gen.getGroundTruthAtTime(0.0);
    auto truth_mid = gen.getGroundTruthAtTime(15.0);
    auto truth_end = gen.getGroundTruthAtTime(30.0);
    
    // Mid should be between start and end
    EXPECT_GT(truth_mid.latitude, truth_start.latitude);
    EXPECT_LT(truth_mid.latitude, truth_end.latitude);
}
// Test 3: Sensor measurements exist
TEST_F(ScenarioGeneratorTest, SensorMeasurementsExist) {
    ScenarioGenerator gen(config_);
    auto measurements = gen.generateMeasurementsAtTime(5.0);
    
    int radar_count = 0, rf_count = 0, optical_count = 0;
    for (const auto& m : measurements) {
        if (m.sensor_type == GeneratedMeasurement::SensorType::RADAR) radar_count++;
        else if (m.sensor_type == GeneratedMeasurement::SensorType::RF) rf_count++;
        else if (m.sensor_type == GeneratedMeasurement::SensorType::OPTICAL) optical_count++;
    }
    
    // Should have at least some measurements
    EXPECT_GT(radar_count + rf_count + optical_count, 0);
}
// Test 4: Predefined scenarios
TEST_F(ScenarioGeneratorTest, PredefinedScenariosValid) {
    auto scenarios = {
        PredefinedScenarios::straightLineCrossing(),
        PredefinedScenarios::loiteringTarget(),
        PredefinedScenarios::evasiveManeuver(),
        PredefinedScenarios::fastIntercept(),
        PredefinedScenarios::lowAltitudeTarget()
    };
    
    for (const auto& scenario : scenarios) {
        EXPECT_GT(scenario.end_time, scenario.start_time);
        EXPECT_GT(scenario.dt, 0.0);
    }
}
// Test 5: Determinism (seeded RNG)
TEST_F(ScenarioGeneratorTest, GeneratorDeterminism) {
    ScenarioGenerator gen1(config_);
    ScenarioGenerator gen2(config_);
    
    auto output1 = gen1.generateScenario();
    auto output2 = gen2.generateScenario();
    
    EXPECT_EQ(output1.measurements.size(), output2.measurements.size());
    EXPECT_EQ(output1.ground_truth.size(), output2.ground_truth.size());
    
    // Compare some measurements
    for (size_t i = 0; i < std::min(output1.measurements.size(), output2.measurements.size()); i++) {
        EXPECT_DOUBLE_EQ(output1.measurements[i].timestamp, output2.measurements[i].timestamp);
        EXPECT_EQ(output1.measurements[i].sensor_type, output2.measurements[i].sensor_type);
    }
}
