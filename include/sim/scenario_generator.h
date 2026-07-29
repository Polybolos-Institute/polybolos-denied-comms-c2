#pragma once
#include <vector>
#include <random>
#include <cmath>
#include <string>
#include <Eigen/Dense>
namespace Polybolos::Fusion::Sim {
// Ground truth target trajectory
struct GroundTruthTarget {
    double timestamp;
    double latitude;
    double longitude;
    double altitude;
    double velocity_north;
    double velocity_east;
    double velocity_vertical;
    double heading;
    double pitch;
    double roll;
    
    double speed() const {
        return std::sqrt(velocity_north*velocity_north +
                        velocity_east*velocity_east +
                        velocity_vertical*velocity_vertical);
    }
};
// Sensor error models
struct RadarErrorModel {
    double range_bias = 0.0;        // meters
    double range_noise_std = 5.0;   // 1-sigma, meters
    double bearing_bias = 0.0;      // radians
    double bearing_noise_std = 0.01; // radians (~0.57 degrees)
    double velocity_noise_std = 0.5; // m/s
    double detection_probability = 0.95;
    
    // Swerling target models
    enum class SwerlingModel { I, II, III, IV } model = SwerlingModel::II;
    double rcs_mean = 10.0;         // dBsm
    double rcs_std = 3.0;
};
struct RFErrorModel {
    double rssi_bias = 0.0;         // dBm
    double rssi_noise_std = 2.0;    // dBm
    double df_bias = 0.0;           // radians
    double df_noise_std = 0.02;     // radians
    double frequency_accuracy = 1e6; // Hz
    double detection_probability = 0.90;
    double emitter_identification_prob = 0.85;
};
struct OpticalErrorModel {
    double pixel_x_noise_std = 2.0;  // pixels
    double pixel_y_noise_std = 2.0;
    double bbox_w_noise_std = 1.0;
    double bbox_h_noise_std = 1.0;
    double confidence_bias = 0.0;
    double confidence_noise_std = 0.05;
    double thermal_noise_std = 5.0;  // Kelvin
    double detection_probability = 0.88;
    double occlusion_probability = 0.05;
};
// Scenario definition
struct ScenarioConfig {
    // Simulation time
    double start_time = 0.0;
    double end_time = 60.0;  // 60 second scenario
    double dt = 0.1;         // 100ms samples
    
    // Target trajectory
    int num_targets = 1;
    std::vector<GroundTruthTarget> target_waypoints;  // Interpolated
    
    // Sensor configuration
    Eigen::Vector3d radar_position = {0, 0, 100};   // meters AGL
    Eigen::Vector3d rf_receiver_position = {1000, 1000, 50};
    Eigen::Vector3d optical_position = {500, -500, 150};
    
    // Sensor error models
    RadarErrorModel radar_errors;
    RFErrorModel rf_errors;
    OpticalErrorModel optical_errors;
    
    // Sensor sampling rates
    double radar_update_rate = 10.0;    // Hz
    double rf_update_rate = 2.0;        // Hz
    double optical_update_rate = 30.0;  // Hz
    
    // Environmental factors
    double wind_speed = 0.0;            // m/s (for RCS variation)
    double atmospheric_attenuation = 1.0; // factor
    double clutter_density = 0.01;      // false targets per km²
};
// Generated measurement (one sensor observation)
struct GeneratedMeasurement {
    enum class SensorType { RADAR, RF, OPTICAL };
    SensorType sensor_type;
    double timestamp;
    bool is_valid = true;  // False if not detected
    
    // Radar measurement
    struct {
        double range = 0.0;       // meters
        double bearing = 0.0;     // radians
        double velocity = 0.0;    // m/s (radial)
        double rcs = 0.0;         // dBsm
    } radar;
    
    // RF measurement
    struct {
        double rssi = 0.0;           // dBm
        double direction = 0.0;      // radians
        double frequency = 0.0;      // MHz
        std::string emitter_id = ""; // identifies emitter
    } rf;
    
    // Optical measurement
    struct {
        double pixel_x = 0.0;      // pixels
        double pixel_y = 0.0;
        double bbox_w = 0.0;       // pixels
        double bbox_h = 0.0;
        double confidence = 0.0;   // [0, 1]
        double thermal_sig = 0.0;  // Kelvin
    } optical;
};
// Scenario generator
class ScenarioGenerator {
public:
    ScenarioGenerator(const ScenarioConfig& config);
    
    // Generate full scenario
    struct ScenarioOutput {
        std::vector<GroundTruthTarget> ground_truth;
        std::vector<GeneratedMeasurement> measurements;
    };
    ScenarioOutput generateScenario();
    
    // Generate measurements for a single timestamp
    std::vector<GeneratedMeasurement> generateMeasurementsAtTime(double timestamp);
    
    // Get ground truth at timestamp (interpolated)
    GroundTruthTarget getGroundTruthAtTime(double timestamp) const;
    
    // Add clutter (false radar/RF detections)
    void addClutter(std::vector<GeneratedMeasurement>& measurements, double timestamp);
    
    // Add emitter RFI (radio frequency interference)
    void addRFI(std::vector<GeneratedMeasurement>& measurements, double timestamp);
    
private:
    ScenarioConfig config_;
    std::mt19937 rng_;  // Seeded RNG for reproducibility
    
    // RNG distributions
    std::normal_distribution<double> normal_{0.0, 1.0};
    std::uniform_real_distribution<double> uniform_{0.0, 1.0};
    
    // Radar measurement generation
    GeneratedMeasurement generateRadarMeasurement(const GroundTruthTarget& truth, double timestamp);
    
    // RF measurement generation
    GeneratedMeasurement generateRFMeasurement(const GroundTruthTarget& truth, double timestamp);
    
    // Optical measurement generation
    GeneratedMeasurement generateOpticalMeasurement(const GroundTruthTarget& truth, double timestamp);
    
    // Helper: convert ECEF to range/bearing from sensor
    void computeRangeBearing(
        const Eigen::Vector3d& target_pos,
        const Eigen::Vector3d& sensor_pos,
        double& range,
        double& bearing,
        double& elevation) const;
    
    // Helper: Swerling RCS model
    double generateRCSWithSwerling(double mean_rcs, RadarErrorModel::SwerlingModel model);
    
    // Helper: path loss model (Friis equation)
    double computePathLoss(double range, double frequency) const;
    
    // Helper: LOS check (for occlusion)
    bool isLineOfSight(const Eigen::Vector3d& target_pos,
                       const Eigen::Vector3d& sensor_pos) const;
};
// Predefined scenarios
class PredefinedScenarios {
public:
    // Straight-line target crossing sensor field
    static ScenarioConfig straightLineCrossing();
    
    // Loitering target (circles)
    static ScenarioConfig loiteringTarget();
    
    // Evasive maneuver (S-turn)
    static ScenarioConfig evasiveManeuver();
    
    // Multiple targets
    static ScenarioConfig multipleTargets(int num_targets);
    
    // High-speed intercept
    static ScenarioConfig fastIntercept();
    
    // Low-altitude target (optical dominant)
    static ScenarioConfig lowAltitudeTarget();
};
}  // namespace Polybolos::Fusion::Sim
