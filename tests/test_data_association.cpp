#include <gtest/gtest.h>
#include "fusion/data_association.h"
#include <array>
#include <iostream>
using namespace Polybolos::Fusion;
class DataAssociationTest : public ::testing::Test {
protected:
    TrackManager track_manager_;
    
    std::vector<Eigen::Matrix<double, 6, 1>> createMeasurements(
        std::vector<std::array<double, 6>> meas_data) {
        
        std::vector<Eigen::Matrix<double, 6, 1>> result;
        for (const auto& data : meas_data) {
            Eigen::Matrix<double, 6, 1> z;
            z << data[0], data[1], data[2], data[3], data[4], data[5];
            result.push_back(z);
        }
        return result;
    }
};
// Test 1: Track creation from measurement
TEST_F(DataAssociationTest, TrackCreation) {
    std::vector<Eigen::Matrix<double, 6, 1>> measurements = createMeasurements({
        {1000.0, 0.0, 50.0, 500.0, 0.0, 1.0}
    });
    
    std::vector<std::string> sensor_types = {"RADAR"};
    std::vector<Eigen::Matrix<double, 9, 1>> filter_states;
    std::vector<Eigen::Matrix<double, 9, 9>> filter_covs;
    
    for (size_t i = 0; i < measurements.size(); i++) {
        filter_states.push_back(Eigen::Matrix<double, 9, 1>::Zero());
        filter_covs.push_back(Eigen::Matrix<double, 9, 9>::Identity());
    }
    
    track_manager_.update(measurements, sensor_types, filter_states, filter_covs, 0.0);
    
    EXPECT_EQ(track_manager_.getTrackCount(), 1);
}
// Test 2: Track confirmation
TEST_F(DataAssociationTest, TrackConfirmation) {
    for (int t = 0; t < 5; t++) {
        std::vector<Eigen::Matrix<double, 6, 1>> measurements = createMeasurements({
            {1000.0 + t*50, 0.0, 50.0, 500.0, 0.0, 1.0}
        });
        
        std::vector<std::string> sensor_types = {"RADAR"};
        std::vector<Eigen::Matrix<double, 9, 1>> filter_states;
        std::vector<Eigen::Matrix<double, 9, 9>> filter_covs;
        
        for (size_t i = 0; i < measurements.size(); i++) {
            filter_states.push_back(Eigen::Matrix<double, 9, 1>::Zero());
            filter_covs.push_back(Eigen::Matrix<double, 9, 9>::Identity());
        }
        
        track_manager_.update(measurements, sensor_types, filter_states, filter_covs, (double)t);
    }
    
    auto confirmed = track_manager_.getConfirmedTracks();
    EXPECT_GT(confirmed.size(), 0);
}
// Test 3: Multiple track association
TEST_F(DataAssociationTest, MultipleTrackAssociation) {
    // Two measurements at different locations
    std::vector<Eigen::Matrix<double, 6, 1>> measurements = createMeasurements({
        {1000.0, 0.0, 50.0, 500.0, 0.0, 1.0},
        {2000.0, 1.57, 60.0, 600.0, 1.57, 1.0}
    });
    
    std::vector<std::string> sensor_types = {"RADAR", "RADAR"};
    std::vector<Eigen::Matrix<double, 9, 1>> filter_states;
    std::vector<Eigen::Matrix<double, 9, 9>> filter_covs;
    
    for (size_t i = 0; i < measurements.size(); i++) {
        filter_states.push_back(Eigen::Matrix<double, 9, 1>::Zero());
        filter_covs.push_back(Eigen::Matrix<double, 9, 9>::Identity());
    }
    
    track_manager_.update(measurements, sensor_types, filter_states, filter_covs, 0.0);
    
    EXPECT_EQ(track_manager_.getTrackCount(), 2);
}
// Test 4: Track coasting
TEST_F(DataAssociationTest, TrackCoasting) {
    // Create track
    std::vector<Eigen::Matrix<double, 6, 1>> measurements = createMeasurements({
        {1000.0, 0.0, 50.0, 500.0, 0.0, 1.0}
    });
    
    std::vector<std::string> sensor_types = {"RADAR"};
    std::vector<Eigen::Matrix<double, 9, 1>> filter_states;
    std::vector<Eigen::Matrix<double, 9, 9>> filter_covs;
    
    for (size_t i = 0; i < measurements.size(); i++) {
        filter_states.push_back(Eigen::Matrix<double, 9, 1>::Zero());
        filter_covs.push_back(Eigen::Matrix<double, 9, 9>::Identity());
    }
    
    track_manager_.update(measurements, sensor_types, filter_states, filter_covs, 0.0);
    
    // No more measurements (coast)
    measurements.clear();
    sensor_types.clear();
    filter_states.clear();
    filter_covs.clear();
    
    for (int t = 1; t < 3; t++) {
        track_manager_.update(measurements, sensor_types, filter_states, filter_covs, (double)t);
    }
    
    auto stats = track_manager_.getStatistics();
    EXPECT_GT(stats.coasted_tracks, 0);
}
