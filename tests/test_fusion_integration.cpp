#include <gtest/gtest.h>
#include "filters/kalman_filter.h"
#include "fusion/data_association.h"
#include "sim/scenario_generator.h"
#include <iostream>
#include <map>
using namespace Polybolos::Fusion;
using namespace Polybolos::Fusion::Sim;
class FusionIntegrationTest : public ::testing::Test {
protected:
    TrackManager track_manager_;
};
// Test: Full fusion loop with synthetic scenario
TEST_F(FusionIntegrationTest, FullFusionLoop) {
    ScenarioConfig scenario = PredefinedScenarios::straightLineCrossing();
    ScenarioGenerator gen(scenario);
    auto output = gen.generateScenario();
    
    // Group measurements by timestamp
    std::map<double, std::vector<GeneratedMeasurement>> measurements_by_time;
    for (const auto& m : output.measurements) {
        measurements_by_time[m.timestamp].push_back(m);
    }
    
    // Process each timestamp
    int processed_frames = 0;
    for (const auto& [timestamp, measurements] : measurements_by_time) {
        std::vector<Eigen::Matrix<double, 6, 1>> z_vec;
        std::vector<std::string> sensor_types;
        std::vector<Eigen::Matrix<double, 9, 1>> filter_states;
        std::vector<Eigen::Matrix<double, 9, 9>> filter_covs;
        
        for (const auto& m : measurements) {
            if (!m.is_valid) continue;
            
            Eigen::Matrix<double, 6, 1> z;
            std::string sensor_type;
            
            if (m.sensor_type == GeneratedMeasurement::SensorType::RADAR) {
                z << m.radar.range, m.radar.bearing, m.radar.velocity,
                     0.0, 0.0, 0.0;
                sensor_type = "RADAR";
            } else if (m.sensor_type == GeneratedMeasurement::SensorType::RF) {
                z << m.rf.rssi, m.rf.direction, 0.0, 0.0, 0.0, 0.0;
                sensor_type = "RF";
            } else if (m.sensor_type == GeneratedMeasurement::SensorType::OPTICAL) {
                z << m.optical.pixel_x, m.optical.pixel_y, m.optical.confidence,
                     m.optical.thermal_sig, 0.0, 0.0;
                sensor_type = "OPTICAL";
            }
            
            z_vec.push_back(z);
            sensor_types.push_back(sensor_type);
            filter_states.push_back(Eigen::Matrix<double, 9, 1>::Zero());
            filter_covs.push_back(Eigen::Matrix<double, 9, 9>::Identity());
        }
        
        track_manager_.update(z_vec, sensor_types, filter_states, filter_covs, timestamp);
        processed_frames++;
    }
    
    EXPECT_GT(processed_frames, 0);
    EXPECT_GT(track_manager_.getTrackCount(), 0);
}
