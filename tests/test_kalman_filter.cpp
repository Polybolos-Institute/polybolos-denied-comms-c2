#include <gtest/gtest.h>
#include "filters/kalman_filter.h"
#include <cmath>
#include <iostream>
using namespace Polybolos::Fusion;
class KalmanFilterTest : public ::testing::Test {
protected:
    ExtendedKalmanFilter ekf_;
    UnscentedKalmanFilter ukf_;
    
    ExtendedKalmanFilter::MeasurementVector createMeasurement(
        double range, double bearing, double velocity,
        double altitude, double heading) {
        
        ExtendedKalmanFilter::MeasurementVector z;
        z(0) = range;
        z(1) = bearing;
        z(2) = velocity;
        z(3) = altitude;
        z(4) = heading;
        z(5) = 1.0;
        return z;
    }
};
// Test 1: EKF initialization
TEST_F(KalmanFilterTest, EKFInitialization) {
    auto z = createMeasurement(1000.0, 0.785, 50.0, 500.0, 0.0);
    ekf_.initialize(z, 0.0);
    
    EXPECT_NEAR(ekf_.getLatitude(), 707.1, 10.0);  // ~1000*cos(45°)
    EXPECT_NEAR(ekf_.getLongitude(), 707.1, 10.0);
    EXPECT_NEAR(ekf_.getAltitude(), 500.0, 1.0);
    EXPECT_GT(ekf_.getPositionUncertainty(), 0.0);
}
// Test 2: EKF predict step (constant velocity)
TEST_F(KalmanFilterTest, EKFPredictConstantVelocity) {
    auto z = createMeasurement(1000.0, 0.0, 50.0, 500.0, 0.0);
    ekf_.initialize(z, 0.0);
    
    ekf_.predict(1.0);  // Predict 1 second ahead
    
    // State should propagate with constant velocity
    double pos_change = ekf_.getSpeed() * 1.0;
    EXPECT_GT(pos_change, 0.0);
}
// Test 3: EKF update with measurement
TEST_F(KalmanFilterTest, EKFUpdateWithMeasurement) {
    auto z1 = createMeasurement(1000.0, 0.0, 50.0, 500.0, 0.0);
    ekf_.initialize(z1, 0.0);
    
    double cov_before = ekf_.getCovariance()(0, 0);
    
    // Update with noisy measurement
    ExtendedKalmanFilter::MeasurementCovarianceMatrix R =
        ExtendedKalmanFilter::MeasurementCovarianceMatrix::Identity() * 10.0;
    auto z2 = createMeasurement(1010.0, 0.01, 51.0, 505.0, 0.01);
    ekf_.update(z2, R);
    
    double cov_after = ekf_.getCovariance()(0, 0);
    
    // Covariance should decrease with update (more confident)
    EXPECT_LT(cov_after, cov_before);
}
// Test 4: EKF Mahalanobis distance gating
TEST_F(KalmanFilterTest, EKFMahalanobisDistance) {
    auto z = createMeasurement(1000.0, 0.0, 50.0, 500.0, 0.0);
    ekf_.initialize(z, 0.0);
    
    ExtendedKalmanFilter::MeasurementCovarianceMatrix R =
        ExtendedKalmanFilter::MeasurementCovarianceMatrix::Identity() * 10.0;
    
    // Measurement close to prediction (low distance)
    auto z_close = createMeasurement(1001.0, 0.001, 50.1, 500.5, 0.001);
    double dist_close = ekf_.mahalanobisDistance(z_close, R);
    
    // Measurement far from prediction (high distance)
    auto z_far = createMeasurement(2000.0, 1.57, 100.0, 1000.0, 1.57);
    double dist_far = ekf_.mahalanobisDistance(z_far, R);
    
    EXPECT_LT(dist_close, dist_far);
    EXPECT_GT(dist_far, 3.0);  // Should exceed 3-sigma gate
}
// Test 5: UKF initialization
TEST_F(KalmanFilterTest, UKFInitialization) {
    auto z = createMeasurement(1000.0, 0.785, 50.0, 500.0, 0.0);
    ukf_.initialize(z, 0.0);
    
    EXPECT_NEAR(ukf_.getLatitude(), 707.1, 10.0);
    EXPECT_NEAR(ukf_.getAltitude(), 500.0, 1.0);
}
// Test 6: UKF predict and update
TEST_F(KalmanFilterTest, UKFPredictUpdate) {
    auto z = createMeasurement(1000.0, 0.0, 50.0, 500.0, 0.0);
    ukf_.initialize(z, 0.0);
    
    ukf_.predict(1.0);
    
    ExtendedKalmanFilter::MeasurementCovarianceMatrix R =
        ExtendedKalmanFilter::MeasurementCovarianceMatrix::Identity() * 10.0;
    auto z2 = createMeasurement(1050.0, 0.0, 50.0, 500.0, 0.0);
    ukf_.update(z2, R);
    
    EXPECT_GT(ukf_.getSpeed(), 0.0);
}
// Test 7: EKF covariance symmetry (numerical stability)
TEST_F(KalmanFilterTest, EKFCovarianceSymmetry) {
    auto z = createMeasurement(1000.0, 0.0, 50.0, 500.0, 0.0);
    ekf_.initialize(z, 0.0);
    
    for (int i = 0; i < 10; i++) {
        ekf_.predict(0.1);
        ExtendedKalmanFilter::MeasurementCovarianceMatrix R =
            ExtendedKalmanFilter::MeasurementCovarianceMatrix::Identity() * 10.0;
        auto z_update = createMeasurement(1000.0 + i*10, 0.0, 50.0, 500.0, 0.0);
        ekf_.update(z_update, R);
    }
    
    auto P = ekf_.getCovariance();
    for (int i = 0; i < 9; i++) {
        for (int j = 0; j < 9; j++) {
            EXPECT_NEAR(P(i, j), P(j, i), 1e-10) << "Covariance not symmetric at (" << i << "," << j << ")";
        }
    }
}
// Test 8: EKF determinism (reproducibility)
TEST_F(KalmanFilterTest, EKFDeterminism) {
    std::vector<ExtendedKalmanFilter::StateVector> results1, results2;
    
    // Run 1
    {
        ExtendedKalmanFilter ekf;
        auto z = createMeasurement(1000.0, 0.0, 50.0, 500.0, 0.0);
        ekf.initialize(z, 0.0);
        results1.push_back(ekf.getState());
        
        for (int i = 0; i < 5; i++) {
            ekf.predict(0.1);
            ExtendedKalmanFilter::MeasurementCovarianceMatrix R =
                ExtendedKalmanFilter::MeasurementCovarianceMatrix::Identity() * 10.0;
            auto z_update = createMeasurement(1000.0 + i*5, 0.0, 50.0, 500.0, 0.0);
            ekf.update(z_update, R);
            results1.push_back(ekf.getState());
        }
    }
    
    // Run 2 (identical)
    {
        ExtendedKalmanFilter ekf;
        auto z = createMeasurement(1000.0, 0.0, 50.0, 500.0, 0.0);
        ekf.initialize(z, 0.0);
        results2.push_back(ekf.getState());
        
        for (int i = 0; i < 5; i++) {
            ekf.predict(0.1);
            ExtendedKalmanFilter::MeasurementCovarianceMatrix R =
                ExtendedKalmanFilter::MeasurementCovarianceMatrix::Identity() * 10.0;
            auto z_update = createMeasurement(1000.0 + i*5, 0.0, 50.0, 500.0, 0.0);
            ekf.update(z_update, R);
            results2.push_back(ekf.getState());
        }
    }
    
    // Compare bit-for-bit
    EXPECT_EQ(results1.size(), results2.size());
    for (size_t i = 0; i < results1.size(); i++) {
        for (int j = 0; j < 9; j++) {
            EXPECT_DOUBLE_EQ(results1[i](j), results2[i](j))
                << "Non-deterministic result at step " << i << " component " << j;
        }
    }
}
