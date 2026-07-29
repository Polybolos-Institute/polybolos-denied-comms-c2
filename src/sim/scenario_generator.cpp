#include "sim/scenario_generator.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
namespace Polybolos::Fusion::Sim {
ScenarioGenerator::ScenarioGenerator(const ScenarioConfig& config)
    : config_(config), rng_(42) {}  // Seeded for reproducibility
GroundTruthTarget ScenarioGenerator::getGroundTruthAtTime(double timestamp) const {
    // Linear interpolation between waypoints
    if (config_.target_waypoints.empty()) {
        return GroundTruthTarget{timestamp, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    }
    
    if (config_.target_waypoints.size() == 1) {
        return config_.target_waypoints[0];
    }
    
    // Find bracketing waypoints
    int idx = 0;
    for (size_t i = 0; i < config_.target_waypoints.size() - 1; i++) {
        if (timestamp >= config_.target_waypoints[i].timestamp &&
            timestamp <= config_.target_waypoints[i+1].timestamp) {
            idx = static_cast<int>(i);
            break;
        }
        if (timestamp > config_.target_waypoints[i].timestamp) {
            idx = static_cast<int>(i);
        }
    }
    
    const auto& wp0 = config_.target_waypoints[idx];
    const auto& wp1 = config_.target_waypoints[std::min(idx+1, (int)config_.target_waypoints.size()-1)];
    
    double t0 = wp0.timestamp;
    double t1 = wp1.timestamp;
    double alpha = (t1 > t0) ? (timestamp - t0) / (t1 - t0) : 0.0;
    alpha = std::clamp(alpha, 0.0, 1.0);
    
    GroundTruthTarget result;
    result.timestamp = timestamp;
    result.latitude = wp0.latitude + alpha * (wp1.latitude - wp0.latitude);
    result.longitude = wp0.longitude + alpha * (wp1.longitude - wp0.longitude);
    result.altitude = wp0.altitude + alpha * (wp1.altitude - wp0.altitude);
    result.velocity_north = wp0.velocity_north + alpha * (wp1.velocity_north - wp0.velocity_north);
    result.velocity_east = wp0.velocity_east + alpha * (wp1.velocity_east - wp0.velocity_east);
    result.velocity_vertical = wp0.velocity_vertical + alpha * (wp1.velocity_vertical - wp0.velocity_vertical);
    result.heading = wp0.heading + alpha * (wp1.heading - wp0.heading);
    result.pitch = wp0.pitch + alpha * (wp1.pitch - wp0.pitch);
    result.roll = wp0.roll + alpha * (wp1.roll - wp0.roll);
    
    return result;
}
void ScenarioGenerator::computeRangeBearing(
    const Eigen::Vector3d& target_pos,
    const Eigen::Vector3d& sensor_pos,
    double& range,
    double& bearing,
    double& elevation) const {
    
    Eigen::Vector3d delta = target_pos - sensor_pos;
    
    range = delta.norm();
    bearing = std::atan2(delta(1), delta(0));
    elevation = std::asin(delta(2) / std::max(range, 1e-6));
}
double ScenarioGenerator::computePathLoss(double range, double frequency) const {
    // Friis free-space path loss (dB)
    // PL = 20*log10(range) + 20*log10(frequency) + 20*log10(4π/c) - G_tx - G_rx
    // Simplified: PL = 20*log10(range*frequency) + constant
    if (range < 1.0) range = 1.0;
    return 20.0 * std::log10(range) + 20.0 * std::log10(frequency) - 147.55;
}
bool ScenarioGenerator::isLineOfSight(const Eigen::Vector3d& target_pos,
                                      const Eigen::Vector3d& sensor_pos) const {
    (void)target_pos;
    (void)sensor_pos;
    // Simplified: always LOS in open scenario
    return true;
}
double ScenarioGenerator::generateRCSWithSwerling(
    double mean_rcs,
    RadarErrorModel::SwerlingModel model) {
    (void)model;
    
    // Simplified: add lognormal variation
    double chi_square = std::abs(normal_(rng_));  // Swerling I
    double variation = 10.0 * std::log10(chi_square + 1.0);
    return mean_rcs + variation * config_.radar_errors.rcs_std / 10.0;
}
GeneratedMeasurement ScenarioGenerator::generateRadarMeasurement(
    const GroundTruthTarget& truth, double timestamp) {
    
    GeneratedMeasurement meas;
    meas.sensor_type = GeneratedMeasurement::SensorType::RADAR;
    meas.timestamp = timestamp;
    
    // Check detection probability
    if (uniform_(rng_) > config_.radar_errors.detection_probability) {
        meas.is_valid = false;
        return meas;
    }
    
    // Waypoints are local ENU meters (see PredefinedScenarios)
    Eigen::Vector3d target_pos(truth.latitude,
                               truth.longitude,
                               truth.altitude);
    
    double range, bearing, elevation;
    computeRangeBearing(target_pos, config_.radar_position, range, bearing, elevation);
    
    // Add measurement noise
    meas.radar.range = range +
        config_.radar_errors.range_bias +
        normal_(rng_) * config_.radar_errors.range_noise_std;
    
    meas.radar.bearing = bearing +
        config_.radar_errors.bearing_bias +
        normal_(rng_) * config_.radar_errors.bearing_noise_std;
    
    // Radial velocity (component along line of sight)
    Eigen::Vector3d velocity(truth.velocity_north, truth.velocity_east, truth.velocity_vertical);
    Eigen::Vector3d los_direction = (target_pos - config_.radar_position).normalized();
    double radial_velocity = velocity.dot(los_direction);
    
    meas.radar.velocity = radial_velocity +
        normal_(rng_) * config_.radar_errors.velocity_noise_std;
    
    // RCS with Swerling model
    meas.radar.rcs = generateRCSWithSwerling(config_.radar_errors.rcs_mean,
                                            config_.radar_errors.model);
    
    meas.is_valid = true;
    return meas;
}
GeneratedMeasurement ScenarioGenerator::generateRFMeasurement(
    const GroundTruthTarget& truth, double timestamp) {
    
    GeneratedMeasurement meas;
    meas.sensor_type = GeneratedMeasurement::SensorType::RF;
    meas.timestamp = timestamp;
    
    if (uniform_(rng_) > config_.rf_errors.detection_probability) {
        meas.is_valid = false;
        return meas;
    }
    
    Eigen::Vector3d target_pos(truth.latitude,
                               truth.longitude,
                               truth.altitude);
    
    double range, bearing, elevation;
    computeRangeBearing(target_pos, config_.rf_receiver_position, range, bearing, elevation);
    
    // RSSI (Received Signal Strength Indicator)
    double frequency = 2400.0;  // MHz (assume 2.4 GHz)
    double path_loss = computePathLoss(range / 1000.0, frequency);  // range in km
    double tx_power = 30.0;  // dBm
    
    meas.rf.rssi = tx_power - path_loss +
        config_.rf_errors.rssi_bias +
        normal_(rng_) * config_.rf_errors.rssi_noise_std;
    
    // Direction finding
    meas.rf.direction = bearing +
        config_.rf_errors.df_bias +
        normal_(rng_) * config_.rf_errors.df_noise_std;
    
    meas.rf.frequency = frequency;
    
    // Emitter ID (if detected)
    if (uniform_(rng_) < config_.rf_errors.emitter_identification_prob) {
        meas.rf.emitter_id = "EMITTER_001";
    }
    
    meas.is_valid = true;
    return meas;
}
GeneratedMeasurement ScenarioGenerator::generateOpticalMeasurement(
    const GroundTruthTarget& truth, double timestamp) {
    
    GeneratedMeasurement meas;
    meas.sensor_type = GeneratedMeasurement::SensorType::OPTICAL;
    meas.timestamp = timestamp;
    
    if (uniform_(rng_) > config_.optical_errors.detection_probability) {
        meas.is_valid = false;
        return meas;
    }
    
    // Check occlusion
    if (uniform_(rng_) < config_.optical_errors.occlusion_probability) {
        meas.is_valid = false;
        return meas;
    }
    
    Eigen::Vector3d target_pos(truth.latitude,
                               truth.longitude,
                               truth.altitude);
    
    // Project onto image plane (simplified pinhole camera model)
    Eigen::Vector3d delta = target_pos - config_.optical_position;
    double focal_length = 1000.0;  // pixels
    double image_center_x = 320.0;
    double image_center_y = 240.0;
    
    double z = delta(2);
    if (z < 0) {
        meas.is_valid = false;
        return meas;
    }
    
    meas.optical.pixel_x = image_center_x + focal_length * delta(0) / z;
    meas.optical.pixel_y = image_center_y + focal_length * delta(1) / z;
    
    // Add pixel noise
    meas.optical.pixel_x += normal_(rng_) * config_.optical_errors.pixel_x_noise_std;
    meas.optical.pixel_y += normal_(rng_) * config_.optical_errors.pixel_y_noise_std;
    
    // Bounding box (target size grows with distance)
    double apparent_size = 50.0 / (z / 1000.0 + 1.0);  // pixels
    meas.optical.bbox_w = apparent_size + normal_(rng_) * config_.optical_errors.bbox_w_noise_std;
    meas.optical.bbox_h = apparent_size + normal_(rng_) * config_.optical_errors.bbox_h_noise_std;
    
    // Confidence (decreases with distance)
    meas.optical.confidence = std::max(0.5, 1.0 - z / 10000.0) +
        normal_(rng_) * config_.optical_errors.confidence_noise_std;
    meas.optical.confidence = std::clamp(meas.optical.confidence, 0.0, 1.0);
    
    // Thermal signature
    meas.optical.thermal_sig = 310.0 +  // ~37°C
        normal_(rng_) * config_.optical_errors.thermal_noise_std;
    
    meas.is_valid = true;
    return meas;
}
std::vector<GeneratedMeasurement> ScenarioGenerator::generateMeasurementsAtTime(double timestamp) {
    std::vector<GeneratedMeasurement> measurements;
    
    GroundTruthTarget truth = getGroundTruthAtTime(timestamp);
    
    // Radar
    if (std::fmod(timestamp, 1.0 / config_.radar_update_rate) < config_.dt) {
        measurements.push_back(generateRadarMeasurement(truth, timestamp));
    }
    
    // RF
    if (std::fmod(timestamp, 1.0 / config_.rf_update_rate) < config_.dt) {
        measurements.push_back(generateRFMeasurement(truth, timestamp));
    }
    
    // Optical
    if (std::fmod(timestamp, 1.0 / config_.optical_update_rate) < config_.dt) {
        measurements.push_back(generateOpticalMeasurement(truth, timestamp));
    }
    
    // Add clutter and RFI
    addClutter(measurements, timestamp);
    addRFI(measurements, timestamp);
    
    return measurements;
}
void ScenarioGenerator::addClutter(std::vector<GeneratedMeasurement>& measurements, double timestamp) {
    // Random false radar detections
    int num_clutter = (int)(config_.clutter_density * 10.0);  // ~1 per scenario
    for (int i = 0; i < num_clutter; i++) {
        if (uniform_(rng_) < 0.05) {  // 5% chance per update
            GeneratedMeasurement clutter;
            clutter.sensor_type = GeneratedMeasurement::SensorType::RADAR;
            clutter.timestamp = timestamp;
            clutter.is_valid = true;
            
            clutter.radar.range = uniform_(rng_) * 5000.0;
            clutter.radar.bearing = uniform_(rng_) * 2 * M_PI;
            clutter.radar.velocity = normal_(rng_) * 10.0;
            clutter.radar.rcs = normal_(rng_) * 5.0 - 20.0;
            
            measurements.push_back(clutter);
        }
    }
}
void ScenarioGenerator::addRFI(std::vector<GeneratedMeasurement>& measurements, double timestamp) {
    // RF interference
    if (uniform_(rng_) < 0.02) {  // 2% chance
        GeneratedMeasurement rfi;
        rfi.sensor_type = GeneratedMeasurement::SensorType::RF;
        rfi.timestamp = timestamp;
        rfi.is_valid = true;
        rfi.rf.rssi = normal_(rng_) * 10.0 - 60.0;
        rfi.rf.direction = uniform_(rng_) * 2 * M_PI;
        rfi.rf.frequency = 2400.0 + normal_(rng_) * 100.0;
        rfi.rf.emitter_id = "JAMMER";
        measurements.push_back(rfi);
    }
}
ScenarioGenerator::ScenarioOutput ScenarioGenerator::generateScenario() {
    ScenarioOutput output;
    
    // Generate measurements for each timestamp
    for (double t = config_.start_time; t <= config_.end_time; t += config_.dt) {
        auto meas = generateMeasurementsAtTime(t);
        output.measurements.insert(output.measurements.end(), meas.begin(), meas.end());
        
        auto truth = getGroundTruthAtTime(t);
        output.ground_truth.push_back(truth);
    }
    
    return output;
}
// ============================================================================
// Predefined Scenarios
// ============================================================================
ScenarioConfig PredefinedScenarios::straightLineCrossing() {
    ScenarioConfig config;
    config.start_time = 0.0;
    config.end_time = 30.0;
    
    // Target crosses from southwest to northeast at constant altitude
    config.target_waypoints = {
        GroundTruthTarget{0.0, -1000, -1000, 500, 100, 100, 0, 0.785, 0, 0},
        GroundTruthTarget{30.0, 2000, 2000, 500, 100, 100, 0, 0.785, 0, 0}
    };
    
    return config;
}
ScenarioConfig PredefinedScenarios::loiteringTarget() {
    ScenarioConfig config;
    config.start_time = 0.0;
    config.end_time = 60.0;
    
    // Target circles at constant altitude
    std::vector<GroundTruthTarget> waypoints;
    for (double t = 0; t <= 60.0; t += 5.0) {
        double angle = (t / 60.0) * 2 * M_PI;
        double radius = 2000.0;
        waypoints.push_back(GroundTruthTarget{
            t,
            radius * std::cos(angle),
            radius * std::sin(angle),
            500.0,
            -radius/60.0 * std::sin(angle),
            radius/60.0 * std::cos(angle),
            0.0,
            angle,
            0.0,
            0.0
        });
    }
    config.target_waypoints = waypoints;
    
    return config;
}
ScenarioConfig PredefinedScenarios::evasiveManeuver() {
    ScenarioConfig config;
    config.start_time = 0.0;
    config.end_time = 45.0;
    
    config.target_waypoints = {
        GroundTruthTarget{0.0, 0, 0, 1000, 150, 0, 0, 0, 0, 0},
        GroundTruthTarget{15.0, 2250, 0, 800, 150, 0, -20, 0, -0.2, 0},
        GroundTruthTarget{30.0, 4500, 0, 600, 150, 0, 0, 0, 0, 0},
        GroundTruthTarget{45.0, 6750, 0, 600, 150, 0, 0, 0, 0, 0}
    };
    
    return config;
}
ScenarioConfig PredefinedScenarios::multipleTargets(int num_targets) {
    ScenarioConfig config;
    config.num_targets = num_targets;
    config.start_time = 0.0;
    config.end_time = 60.0;
    
    // Just return base config; in practice would add waypoints for each target
    return config;
}
ScenarioConfig PredefinedScenarios::fastIntercept() {
    ScenarioConfig config;
    config.start_time = 0.0;
    config.end_time = 20.0;
    
    config.target_waypoints = {
        GroundTruthTarget{0.0, -5000, 0, 100, 500, 0, 0, 0, 0, 0},
        GroundTruthTarget{20.0, 5000, 0, 100, 500, 0, 0, 0, 0, 0}
    };
    
    return config;
}
ScenarioConfig PredefinedScenarios::lowAltitudeTarget() {
    ScenarioConfig config;
    config.start_time = 0.0;
    config.end_time = 40.0;
    
    config.target_waypoints = {
        GroundTruthTarget{0.0, -2000, -2000, 50, 75, 75, 0, 0.785, 0, 0},
        GroundTruthTarget{40.0, 2000, 2000, 50, 75, 75, 0, 0.785, 0, 0}
    };
    
    // Low altitude → optical dominant, radar weak
    config.optical_errors.detection_probability = 0.95;
    config.radar_errors.detection_probability = 0.60;
    config.rf_errors.detection_probability = 0.75;
    
    return config;
}
}  // namespace Polybolos::Fusion::Sim
