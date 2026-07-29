#include <gtest/gtest.h>
#include "filters/kalman_filter.h"
#include "fusion/data_association.h"
#include <vector>
#include <cstring>
using namespace Polybolos::Fusion;
// Test: Bit-for-bit determinism across multiple runs
TEST(DeterminismTest, BitForBitReproducibility) {
    std::vector<std::vector<uint8_t>> results;
    
    for (int run = 0; run < 3; run++) {
        ExtendedKalmanFilter ekf;
        
        // Identical sequence of operations
        ExtendedKalmanFilter::MeasurementVector z;
        z << 1000.0, 0.0, 50.0, 500.0, 0.0, 1.0;
        ekf.initialize(z, 0.0);
        
        std::vector<uint8_t> state_bytes;
        const auto& state = ekf.getState();
        for (int i = 0; i < 9; i++) {
            uint8_t* bytes = (uint8_t*)&state(i);
            for (int j = 0; j < 8; j++) {
                state_bytes.push_back(bytes[j]);
            }
        }
        
        for (int i = 0; i < 5; i++) {
            ekf.predict(0.1);
            ExtendedKalmanFilter::MeasurementCovarianceMatrix R =
                ExtendedKalmanFilter::MeasurementCovarianceMatrix::Identity() * 10.0;
            ExtendedKalmanFilter::MeasurementVector z_update;
            z_update << 1000.0 + i*5, 0.0, 50.0, 500.0, 0.0, 1.0;
            ekf.update(z_update, R);
            
            const auto& state = ekf.getState();
            for (int j = 0; j < 9; j++) {
                uint8_t* bytes = (uint8_t*)&state(j);
                for (int k = 0; k < 8; k++) {
                    state_bytes.push_back(bytes[k]);
                }
            }
        }
        
        results.push_back(state_bytes);
    }
    
    // All runs should be identical
    for (size_t i = 1; i < results.size(); i++) {
        EXPECT_EQ(results[0].size(), results[i].size());
        EXPECT_EQ(results[0], results[i]);
    }
}
