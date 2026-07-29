#include "fusion/multi_sensor_fusion.h"
#include "sim/scenario_generator.h"

#include <cmath>
#include <iostream>

using namespace Polybolos::Fusion;
using namespace Polybolos::Fusion::Sim;

namespace {

constexpr double kPi = 3.14159265358979323846;

SensorObservation toObs(const GeneratedMeasurement& m) {
 SensorObservation o;
 o.timestamp = m.timestamp;
 if (m.sensor_type == GeneratedMeasurement::SensorType::RADAR) {
 o.sensor_type = SensorObservation::Type::RADAR;
 o.radar.range = m.radar.range;
 o.radar.bearing = m.radar.bearing * 180.0 / kPi;
 o.radar.velocity = m.radar.velocity;
 o.radar.rcs = m.radar.rcs;
 } else if (m.sensor_type == GeneratedMeasurement::SensorType::RF) {
 o.sensor_type = SensorObservation::Type::RF;
 o.rf.rssi = m.rf.rssi;
 o.rf.direction = m.rf.direction * 180.0 / kPi;
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

} // namespace

int main() {
 auto cfg = PredefinedScenarios::straightLineCrossing();
 cfg.end_time = 10.0;
 cfg.dt = 0.05;
 ScenarioGenerator gen(cfg);

 MultiSensorFusion fusion;
 fusion.setFilterKind(FilterKind::EKF);

 double err_fusion = 0.0;
 double err_naive = 0.0;
 int n = 0;
 double max_cycle_ms = 0.0;
 double sum_cycle_ms = 0.0;

 for (double t = cfg.start_time; t <= cfg.end_time; t += cfg.dt) {
 auto truth = gen.getGroundTruthAtTime(t);
 auto meas = gen.generateMeasurementsAtTime(t);
 std::vector<SensorObservation> frame_obs;
 for (const auto& m : meas) {
 if (!m.is_valid) {
 continue;
 }
 auto o = toObs(m);
 frame_obs.push_back(o);
 fusion.addObservation(o);
 }
 fusion.processCycle(cfg.dt);
 max_cycle_ms = std::max(max_cycle_ms, fusion.stats().last_cycle_ms);
 sum_cycle_ms += fusion.stats().last_cycle_ms;

 // Truth lat/lon in this scenario are local meters - treat as ENU offsets for error metric
 const auto tracks = fusion.getFusedTracks();
 if (!tracks.empty()) {
 const auto& tr = tracks.front();
 const double dx = (tr.latitude - 38.8977) * 111320.0 - truth.latitude;
 const double dy = (tr.longitude + 77.0365) * 111320.0 - truth.longitude;
 err_fusion += std::sqrt(dx * dx + dy * dy);
 }
 const auto naive = MultiSensorFusion::naiveFuse(frame_obs, 0.0, 0.0, 100.0);
 const double ndx = naive.latitude * 111320.0 - truth.latitude;
 const double ndy = naive.longitude * 111320.0 - truth.longitude;
 err_naive += std::sqrt(ndx * ndx + ndy * ndy);
 ++n;
 }

 const double mean_f = (n > 0) ? err_fusion / n : 0.0;
 const double mean_n = (n > 0) ? err_naive / n : 0.0;
 const double mean_cycle = (n > 0) ? sum_cycle_ms / n : 0.0;

 std::cout << "frames=" << n << "\n";
 std::cout << "mean_pos_error_fusion_m=" << mean_f << "\n";
 std::cout << "mean_pos_error_naive_m=" << mean_n << "\n";
 std::cout << "improvement_ratio=" << (mean_n > 1e-9 ? mean_n / std::max(1e-9, mean_f) : 0.0)
 << "\n";
 std::cout << "mean_cycle_ms=" << mean_cycle << "\n";
 std::cout << "max_cycle_ms=" << max_cycle_ms << "\n";
 std::cout << ((max_cycle_ms < 50.0) ? "latency_budget=PASS\n" : "latency_budget=FAIL\n");
 return 0;
}
