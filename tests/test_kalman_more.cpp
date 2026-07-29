#include <gtest/gtest.h>

#include <cmath>

#include "filters/kalman_filter.h"

using namespace Polybolos::Fusion;

namespace {

ExtendedKalmanFilter::MeasurementVector makeZ(double range, double bearing, double vel,
                                              double alt, double heading) {
    ExtendedKalmanFilter::MeasurementVector z;
    z << range, bearing, vel, alt, heading, 1.0;
    return z;
}

ExtendedKalmanFilter::MeasurementCovarianceMatrix defaultR() {
    return ExtendedKalmanFilter::MeasurementCovarianceMatrix::Identity();
}

}  // namespace

TEST(KalmanEkf, ZeroDtKeepsState) {
    ExtendedKalmanFilter kf;
    kf.initialize(makeZ(100.0, 0.0, 0.0, 10.0, 0.0), 0.0);
    const auto x0 = kf.getState();
    kf.predict(0.0);
    EXPECT_DOUBLE_EQ(kf.getState()(0), x0(0));
}

TEST(KalmanEkf, CovarianceGrowsOnPredict) {
    ExtendedKalmanFilter kf;
    kf.initialize(makeZ(100.0, 0.0, 0.0, 10.0, 0.0), 0.0);
    const double before = kf.getCovariance()(0, 0);
    kf.predict(1.0);
    EXPECT_GE(kf.getCovariance()(0, 0), before);
}

TEST(KalmanEkf, AdaptivePathRuns) {
    ExtendedKalmanFilter kf;
    kf.initialize(makeZ(100.0, 0.0, 0.0, 10.0, 0.0), 0.0);
    auto z = makeZ(150.0, 0.2, 5.0, 20.0, 0.2);
    for (int i = 0; i < 20; ++i) {
        kf.predict(0.05);
        kf.update(z, defaultR());
    }
    EXPECT_TRUE(std::isfinite(kf.mahalanobisDistance(z, defaultR())));
}

TEST(KalmanUkf, SpeedNonNegative) {
    UnscentedKalmanFilter kf;
    kf.initialize(makeZ(300.0, 1.0, 12.0, 40.0, 1.0), 0.0);
    EXPECT_GE(kf.getSpeed(), 0.0);
}

TEST(KalmanUkf, MultipleStepsStable) {
    UnscentedKalmanFilter kf;
    auto z = makeZ(400.0, 0.0, 10.0, 50.0, 0.0);
    kf.initialize(z, 0.0);
    for (int i = 0; i < 20; ++i) {
        kf.predict(0.05);
        kf.update(z, defaultR());
    }
    EXPECT_TRUE(std::isfinite(kf.getLatitude()));
}

TEST(KalmanEkf, PredictMeasurementFinite) {
    ExtendedKalmanFilter kf;
    kf.initialize(makeZ(100.0, 0.3, 8.0, 12.0, 0.3), 0.0);
    const auto zp = kf.predictMeasurement();
    EXPECT_EQ(zp.size(), 6);
    EXPECT_TRUE(std::isfinite(zp(0)));
}

TEST(KalmanEkf, PositionUncertaintyPositive) {
    ExtendedKalmanFilter kf;
    kf.initialize(makeZ(100.0, 0.0, 0.0, 10.0, 0.0), 0.0);
    EXPECT_GT(kf.getPositionUncertainty(), 0.0);
}

TEST(KalmanUkf, PredictMeasurement) {
    UnscentedKalmanFilter kf;
    kf.initialize(makeZ(100.0, 0.0, 0.0, 10.0, 0.0), 0.0);
    EXPECT_EQ(kf.predictMeasurement().size(), 6);
}
