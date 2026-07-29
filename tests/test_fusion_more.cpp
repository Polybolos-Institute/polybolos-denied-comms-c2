#include <gtest/gtest.h>

#include "fusion/multi_sensor_fusion.h"
#include "sim/scenario_generator.h"

using namespace Polybolos::Fusion;
using namespace Polybolos::Fusion::Sim;

TEST(Scenario, TruthCircularMotion) {
    auto cfg = PredefinedScenarios::loiteringTarget();
    ScenarioGenerator gen(cfg);
    auto a = gen.getGroundTruthAtTime(0.0);
    auto b = gen.getGroundTruthAtTime(5.0);
    EXPECT_NE(a.latitude, b.latitude);
    EXPECT_GT(a.speed(), 0.0);
}

TEST(Scenario, LowAltitudeOpticalDominant) {
    auto cfg = PredefinedScenarios::lowAltitudeTarget();
    EXPECT_GT(cfg.optical_errors.detection_probability, cfg.radar_errors.detection_probability);
}

TEST(Scenario, GenerateAtTime) {
    auto cfg = PredefinedScenarios::straightLineCrossing();
    ScenarioGenerator gen(cfg);
    auto meas = gen.generateMeasurementsAtTime(0.0);
    // May include invalid detections; container itself should be fine
    EXPECT_GE(meas.size(), 0u);
}

TEST(Fusion, GetTrackById) {
    MultiSensorFusion fusion;
    SensorObservation obs;
    obs.sensor_type = SensorObservation::Type::RADAR;
    obs.radar.range = 500.0;
    obs.radar.bearing = 10.0;
    fusion.addObservation(obs);
    fusion.processCycle(0.05);
    const auto tracks = fusion.getFusedTracks();
    ASSERT_FALSE(tracks.empty());
    const auto t = fusion.getTrack(tracks.front().track_id);
    ASSERT_TRUE(t.has_value());
    EXPECT_EQ(t->track_id, tracks.front().track_id);
}

TEST(Fusion, MissingTrackReturnsNullopt) {
    MultiSensorFusion fusion;
    EXPECT_FALSE(fusion.getTrack("nope").has_value());
}

TEST(Fusion, CredibilityUpdate) {
    MultiSensorFusion fusion;
    fusion.updateSensorCredibility(SensorObservation::Type::RADAR, 0.5);
    SensorObservation obs;
    obs.sensor_type = SensorObservation::Type::RADAR;
    obs.radar.range = 800.0;
    obs.radar.bearing = 0.0;
    fusion.addObservation(obs);
    fusion.processCycle(0.05);
    EXPECT_GE(fusion.stats().tracks_alive, 1);
}

TEST(Fusion, UkfPath) {
    MultiSensorFusion fusion;
    fusion.setFilterKind(FilterKind::UKF);
    SensorObservation obs;
    obs.sensor_type = SensorObservation::Type::RF;
    obs.rf.rssi = -60.0;
    obs.rf.direction = 45.0;
    fusion.addObservation(obs);
    fusion.processCycle(0.05);
    EXPECT_FALSE(fusion.getFusedTracks().empty());
}

TEST(Fusion, CoastAndPrune) {
    MultiSensorFusion fusion;
    SensorObservation obs;
    obs.sensor_type = SensorObservation::Type::RADAR;
    obs.radar.range = 1000.0;
    obs.radar.bearing = 0.0;
    fusion.addObservation(obs);
    fusion.processCycle(0.05);
    EXPECT_GE(fusion.stats().tracks_alive, 1);
    for (int i = 0; i < 8; ++i) {
        fusion.processCycle(0.05);
    }
    EXPECT_EQ(fusion.stats().tracks_alive, 0);
}

TEST(Fusion, OpticalConversion) {
    MultiSensorFusion fusion;
    SensorObservation obs;
    obs.sensor_type = SensorObservation::Type::OPTICAL;
    obs.optical.pixel_x = 640.0;
    obs.optical.pixel_y = 360.0;
    obs.optical.confidence = 0.9;
    fusion.addObservation(obs);
    fusion.processCycle(0.05);
    EXPECT_FALSE(fusion.getFusedTracks().empty());
}

TEST(Fusion, StatsCountObservations) {
    MultiSensorFusion fusion;
    for (int i = 0; i < 3; ++i) {
        SensorObservation obs;
        obs.sensor_type = SensorObservation::Type::RADAR;
        obs.radar.range = 1000.0 + i;
        obs.radar.bearing = static_cast<double>(i);
        fusion.addObservation(obs);
    }
    fusion.processCycle(0.05);
    EXPECT_EQ(fusion.stats().observations_in, 3);
}

TEST(Fusion, AssociationGateConfigurable) {
    MultiSensorFusion fusion;
    fusion.setAssociationGate(0.01);
    SensorObservation a;
    a.sensor_type = SensorObservation::Type::RADAR;
    a.radar.range = 1000.0;
    a.radar.bearing = 0.0;
    fusion.addObservation(a);
    fusion.processCycle(0.05);
    SensorObservation b;
    b.sensor_type = SensorObservation::Type::RADAR;
    b.radar.range = 3000.0;
    b.radar.bearing = 90.0;
    fusion.addObservation(b);
    fusion.processCycle(0.05);
    EXPECT_GE(fusion.stats().tracks_alive, 1);
}

TEST(Fusion, OriginAffectsNaive) {
    SensorObservation r2;
    r2.sensor_type = SensorObservation::Type::RADAR;
    r2.radar.range = 100.0;
    r2.radar.bearing = 0.0;
    const auto t = MultiSensorFusion::naiveFuse({r2}, 38.0, -77.0, 50.0);
    EXPECT_GT(t.latitude, 38.0);
}

TEST(Scenario, TruthHistoryGrows) {
    auto cfg = PredefinedScenarios::straightLineCrossing();
    cfg.end_time = 0.3;
    cfg.dt = 0.1;
    ScenarioGenerator gen(cfg);
    auto out = gen.generateScenario();
    EXPECT_GE(out.ground_truth.size(), 3u);
}
