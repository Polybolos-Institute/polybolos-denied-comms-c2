#include <gtest/gtest.h>

#include "fusion/multi_sensor_fusion.h"
#include "sim/scenario_generator.h"

using namespace Polybolos::Fusion;
using namespace Polybolos::Fusion::Sim;

namespace {

SensorObservation toObs(const GeneratedMeasurement& m) {
    SensorObservation o;
    o.timestamp = m.timestamp;
    if (m.sensor_type == GeneratedMeasurement::SensorType::RADAR) {
        o.sensor_type = SensorObservation::Type::RADAR;
        o.radar.range = m.radar.range;
        o.radar.bearing = m.radar.bearing * 180.0 / 3.14159265358979323846;
        o.radar.velocity = m.radar.velocity;
        o.radar.rcs = m.radar.rcs;
    } else if (m.sensor_type == GeneratedMeasurement::SensorType::RF) {
        o.sensor_type = SensorObservation::Type::RF;
        o.rf.rssi = m.rf.rssi;
        o.rf.direction = m.rf.direction * 180.0 / 3.14159265358979323846;
        o.rf.frequency = m.rf.frequency;
        o.rf.emitter_id = m.rf.emitter_id;
    } else {
        o.sensor_type = SensorObservation::Type::OPTICAL;
        o.optical.pixel_x = m.optical.pixel_x;
        o.optical.pixel_y = m.optical.pixel_y;
        o.optical.bbox_w = m.optical.bbox_w;
        o.optical.bbox_h = m.optical.bbox_h;
        o.optical.confidence = m.optical.confidence;
        o.optical.thermal_sig = m.optical.thermal_sig;
    }
    return o;
}

}  // namespace

TEST(FusionLoop, TracksFromScenario) {
    auto cfg = PredefinedScenarios::straightLineCrossing();
    cfg.end_time = 2.0;
    cfg.dt = 0.05;
    ScenarioGenerator gen(cfg);

    MultiSensorFusion fusion;
    fusion.setFilterKind(FilterKind::EKF);

    for (double t = cfg.start_time; t <= cfg.end_time; t += cfg.dt) {
        auto meas = gen.generateMeasurementsAtTime(t);
        for (const auto& m : meas) {
            if (m.is_valid) {
                fusion.addObservation(toObs(m));
            }
        }
        fusion.processCycle(cfg.dt);
        EXPECT_LT(fusion.stats().last_cycle_ms, 50.0);
    }
    EXPECT_FALSE(fusion.getFusedTracks().empty());
}

TEST(FusionLoop, NaiveBaselineExists) {
    SensorObservation r;
    r.sensor_type = SensorObservation::Type::RADAR;
    r.radar.range = 1000.0;
    r.radar.bearing = 0.0;
    const auto t = MultiSensorFusion::naiveFuse({r}, 38.0, -77.0, 100.0);
    EXPECT_EQ(t.track_id, "naive");
    EXPECT_NE(t.latitude, 0.0);
}
