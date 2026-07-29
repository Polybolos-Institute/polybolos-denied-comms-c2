#pragma once
#include <vector>
#include <string>
#include <map>
#include <optional>
#include <cmath>
#include <Eigen/Dense>
namespace Polybolos::Fusion {
// Track (persistent object being tracked)
struct Track {
    std::string track_id;
    int age = 0;  // frames since creation
    int consecutive_misses = 0;
    
    enum class State {
        TENTATIVE,   // Just created, not confirmed
        CONFIRMED,   // Confirmed track
        COASTED,     // No recent measurement, coasting on prediction
        DELETED      // Marked for deletion
    };
    State state = State::TENTATIVE;
    
    // Filter state (from EKF/UKF)
    Eigen::Matrix<double, 9, 1> filter_state;  // [lat, lon, alt, vx, vy, vz, heading, pitch, roll]
    Eigen::Matrix<double, 9, 9> filter_covariance;
    
    // Measurement history (last 10 measurements)
    struct MeasurementRecord {
        double timestamp;
        Eigen::Matrix<double, 6, 1> measurement;  // [range, bearing, vel, alt, heading, conf]
        double innovation_norm;
        std::string sensor_type;
    };
    std::vector<MeasurementRecord> measurement_history;
    static constexpr int HISTORY_SIZE = 10;
    
    // Consensus status
    struct {
        bool radar_recent = false;
        bool rf_recent = false;
        bool optical_recent = false;
        int radar_count = 0;
        int rf_count = 0;
        int optical_count = 0;
    } sensor_agreement;
    
    // Last update time
    double last_update_timestamp = 0.0;
    
    // Kinematic parameters
    double speed() const {
        return std::sqrt(filter_state(3)*filter_state(3) +
                        filter_state(4)*filter_state(4) +
                        filter_state(5)*filter_state(5));
    }
    
    double latitude() const { return filter_state(0); }
    double longitude() const { return filter_state(1); }
    double altitude() const { return filter_state(2); }
    double heading() const { return filter_state(6); }
};
// Measurement-to-track assignment result
struct Assignment {
    int track_idx = -1;      // -1 means unassigned (new track)
    int measurement_idx = -1; // -1 means no assignment (track coasts)
    double cost = 1e9;       // Mahalanobis distance
    double gating_threshold = 3.0;  // 3-sigma gate
    bool is_valid() const {
        return cost < gating_threshold;
    }
};
// Data association engine
class DataAssociation {
public:
    DataAssociation();
    
    // Add measurement (from any sensor)
    void addMeasurement(const Eigen::Matrix<double, 6, 1>& z,
                       const std::string& sensor_type,
                       double timestamp);
    
    // Perform Hungarian assignment
    // Returns: map of track_id -> best measurement index (or -1 if unassigned)
    std::map<std::string, int> associate(
        const std::vector<Track>& tracks,
        const std::vector<Eigen::Matrix<double, 9, 9>>& covariances);
    
    // Create new track from unassigned measurement
    Track createTrackFromMeasurement(const Eigen::Matrix<double, 6, 1>& z,
                                    const std::string& sensor_type,
                                    double timestamp);
    
    // Update track state (age, consecutive_misses, etc.)
    void updateTrackState(Track& track, int assigned_measurement_idx, double timestamp);
    
    // Prune old/dead tracks
    void pruneTracks(std::vector<Track>& tracks, double current_timestamp);
    
    // Get unassigned measurements
    std::vector<int> getUnassignedMeasurements() const;
    
    // Mahalanobis distance computation
    static double computeMahalanobisDistance(
        const Eigen::Matrix<double, 6, 1>& z,
        const Eigen::Matrix<double, 6, 1>& z_pred,
        const Eigen::Matrix<double, 6, 6>& innovation_cov);
    
    // Get measurement history for track
    std::vector<Track::MeasurementRecord> getMeasurementHistory(const std::string& track_id) const;
    
private:
    friend class TrackManager;
    void clearPending();
    
    std::vector<Eigen::Matrix<double, 6, 1>> pending_measurements_;
    std::vector<std::string> measurement_sensor_types_;
    std::vector<double> measurement_timestamps_;
    
    // Cost matrix for Hungarian algorithm
    Eigen::MatrixXd cost_matrix_;
    
    // Hungarian algorithm implementation
    std::vector<int> hungarianAlgorithm(const Eigen::MatrixXd& cost_matrix);
    
    // Parameters
    static constexpr double GATING_THRESHOLD = 25.0;  // multi-DOF Mahalanobis gate
    static constexpr int MAX_COASTING_FRAMES = 5;
    static constexpr int CONFIRMATION_THRESHOLD = 3;  // frames to confirm
    static constexpr double MAX_TRACK_AGE = 300.0;  // seconds
};
// Track manager (maintains persistent tracks)
class TrackManager {
public:
    TrackManager();
    
    // Process new measurements and update tracks
    void update(const std::vector<Eigen::Matrix<double, 6, 1>>& measurements,
               const std::vector<std::string>& sensor_types,
               const std::vector<Eigen::Matrix<double, 9, 1>>& filter_states,
               const std::vector<Eigen::Matrix<double, 9, 9>>& filter_covariances,
               double timestamp);
    
    // Get all active tracks
    std::vector<Track> getActiveTracks() const;
    
    // Get confirmed tracks only
    std::vector<Track> getConfirmedTracks() const;
    
    // Get track by ID
    std::optional<Track> getTrack(const std::string& track_id) const;
    
    // Get track count
    size_t getTrackCount() const { return tracks_.size(); }
    size_t getConfirmedTrackCount() const;
    
    // Manual track deletion
    void deleteTrack(const std::string& track_id);
    
    // Get track history
    std::vector<Track::MeasurementRecord> getTrackHistory(const std::string& track_id) const;
    
    // Statistics
    struct Statistics {
        int total_tracks = 0;
        int confirmed_tracks = 0;
        int tentative_tracks = 0;
        int coasted_tracks = 0;
        int total_assignments = 0;
        int failed_associations = 0;
    };
    Statistics getStatistics() const;
    
private:
    std::vector<Track> tracks_;
    DataAssociation associator_;
    
    int next_track_id_ = 0;
    
    std::string generateTrackId();
};
}  // namespace Polybolos::Fusion
