#include <gtest/gtest.h>

#include <cmath>

#include "filters/kalman_filter.h"

using namespace Polybolos::Fusion;

namespace {

ExtendedKalmanFilter::MeasurementVector makeZ(double range, double bearing, double vel,
                                              double alt, double heading, double conf = 1.0) {
    ExtendedKalmanFilter::MeasurementVector z;
    z << range, bearing, vel, alt, heading, conf;
    return z;
}

ExtendedKalmanFilter::MeasurementCovarianceMatrix defaultR() {
    ExtendedKalmanFilter::MeasurementCovarianceMatrix R =
        ExtendedKalmanFilter::MeasurementCovarianceMatrix::Identity();
    R(0, 0) = 25.0;
    R(1, 1) = 0.01;
    R(2, 2) = 1.0;
    R(3, 3) = 25.0;
    R(4, 4) = 0.01;
    R(5, 5) = 0.01;
    return R;
}

}  // namespace

TEST(KalmanEkf, StateSizeIsNine) {
    EXPECT_EQ(ExtendedKalmanFilter::STATE_SIZE, 9);
    EXPECT_EQ(ExtendedKalmanFilter::MEASUREMENT_SIZE, 6);
}

TEST(KalmanEkf, PredictMovesWithVelocity) {
    ExtendedKalmanFilter kf;
    auto z = makeZ(1000.0, 0.0, 10.0, 100.0, 0.0);
    kf.initialize(z, 0.0);
    const double lat0 = kf.getLatitude();
    kf.predict(1.0);
    // After init, vx roughly from heading; position should evolve
    EXPECT_NE(kf.getLatitude(), lat0);
}

TEST(KalmanEkf, UpdateReducesInnovation) {
    ExtendedKalmanFilter kf;
    auto z = makeZ(1000.0, 0.25, 20.0, 120.0, 0.25);
    kf.initialize(z, 0.0);
    kf.predict(0.05);
    const double d0 = kf.mahalanobisDistance(z, defaultR());
    kf.update(z, defaultR());
    const double d1 = kf.mahalanobisDistance(z, defaultR());
    EXPECT_LE(d1, d0 + 1e-6);
}

TEST(KalmanUkf, PredictAndUpdate) {
    UnscentedKalmanFilter kf;
    auto z = makeZ(500.0, 0.1, 15.0, 80.0, 0.1);
    kf.initialize(z, 0.0);
    kf.predict(0.05);
    kf.update(z, defaultR());
    EXPECT_EQ(kf.getState().size(), 9);
    EXPECT_GE(kf.getPositionUncertainty(), 0.0);
}

TEST(KalmanEkf, GettersConsistent) {
    ExtendedKalmanFilter kf;
    kf.initialize(makeZ(200.0, 0.5, 5.0, 50.0, 0.5), 0.0);
    EXPECT_DOUBLE_EQ(kf.getLatitude(), kf.getState()(0));
    EXPECT_DOUBLE_EQ(kf.getAltitude(), kf.getState()(2));
    EXPECT_GE(kf.getSpeed(), 0.0);
}
