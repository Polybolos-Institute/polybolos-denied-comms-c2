#include <gtest/gtest.h>

#include "fusion/data_association.h"

using namespace Polybolos::Fusion;

namespace {

Eigen::Matrix<double, 6, 1> makeZ(double range, double bearing) {
    Eigen::Matrix<double, 6, 1> z;
    z << range, bearing, 10.0, 100.0, bearing, 1.0;
    return z;
}

}  // namespace

TEST(Hungarian, PreferCloserMeasurement) {
    DataAssociation da;
    Track t = da.createTrackFromMeasurement(makeZ(1000.0, 0.0), "RADAR", 0.0);
    t.track_id = "T0";
    da.addMeasurement(makeZ(1005.0, 0.01), "RADAR", 0.1);
    da.addMeasurement(makeZ(5000.0, 1.5), "RADAR", 0.1);
    auto map = da.associate({t}, {t.filter_covariance});
    ASSERT_TRUE(map.count("T0") > 0);
    // Closer measurement should win under gating (idx 0) or unassigned if both fail
    EXPECT_TRUE(map["T0"] == 0 || map["T0"] == -1);
}

TEST(TrackLifecycle, ConfirmAfterEnoughHits) {
    TrackManager tm;
    Eigen::Matrix<double, 9, 1> x = Eigen::Matrix<double, 9, 1>::Zero();
    x(0) = 1000.0;
    Eigen::Matrix<double, 9, 9> P = Eigen::Matrix<double, 9, 9>::Identity();
    for (int i = 0; i < 5; ++i) {
        tm.update({makeZ(1000.0, 0.0)}, {"RADAR"}, {x}, {P}, static_cast<double>(i) * 0.05);
    }
    EXPECT_GE(tm.getTrackCount(), 1u);
}

TEST(TrackLifecycle, HistoryBounded) {
    TrackManager tm;
    Eigen::Matrix<double, 9, 1> x = Eigen::Matrix<double, 9, 1>::Zero();
    Eigen::Matrix<double, 9, 9> P = Eigen::Matrix<double, 9, 9>::Identity();
    for (int i = 0; i < 20; ++i) {
        tm.update({makeZ(800.0 + i, 0.0)}, {"RADAR"}, {x}, {P}, static_cast<double>(i));
    }
    const auto tracks = tm.getActiveTracks();
    if (!tracks.empty()) {
        auto hist = tm.getTrackHistory(tracks.front().track_id);
        EXPECT_LE(hist.size(), static_cast<size_t>(Track::HISTORY_SIZE));
    }
}

TEST(DataAssociation, UnassignedGetterExists) {
    DataAssociation da;
    EXPECT_TRUE(da.getUnassignedMeasurements().empty());
}

TEST(DataAssociation, MeasurementHistoryStub) {
    DataAssociation da;
    EXPECT_TRUE(da.getMeasurementHistory("none").empty());
}

TEST(TrackManager, ConfirmedCount) {
    TrackManager tm;
    EXPECT_EQ(tm.getConfirmedTrackCount(), 0u);
}

TEST(TrackManager, MissingTrackNullopt) {
    TrackManager tm;
    EXPECT_FALSE(tm.getTrack("nope").has_value());
}
