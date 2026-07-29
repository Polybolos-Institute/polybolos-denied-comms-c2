#include "fusion/data_association.h"
#include <algorithm>
#include <numeric>
#include <cmath>
#include <limits>
#include <iostream>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
namespace Polybolos::Fusion {
// ============================================================================
// Data Association Implementation
// ============================================================================
DataAssociation::DataAssociation() {}
void DataAssociation::clearPending() {
 pending_measurements_.clear();
 measurement_sensor_types_.clear();
 measurement_timestamps_.clear();
}
void DataAssociation::addMeasurement(const Eigen::Matrix<double, 6, 1>& z,
 const std::string& sensor_type,
 double timestamp) {
 pending_measurements_.push_back(z);
 measurement_sensor_types_.push_back(sensor_type);
 measurement_timestamps_.push_back(timestamp);
}
double DataAssociation::computeMahalanobisDistance(
 const Eigen::Matrix<double, 6, 1>& z,
 const Eigen::Matrix<double, 6, 1>& z_pred,
 const Eigen::Matrix<double, 6, 6>& innovation_cov) {
 
 Eigen::Matrix<double, 6, 1> innovation = z - z_pred;
 
 // Handle circular (angle) residuals
 if (std::abs(innovation(1)) > M_PI) {
 innovation(1) -= (innovation(1) > 0 ? 2*M_PI : -2*M_PI);
 }
 if (std::abs(innovation(4)) > M_PI) {
 innovation(4) -= (innovation(4) > 0 ? 2*M_PI : -2*M_PI);
 }
 
 try {
 double mahal = std::sqrt(innovation.transpose() * innovation_cov.inverse() * innovation);
 return mahal;
 } catch (...) {
 return 1e9; // Singular covariance
 }
}
std::vector<int> DataAssociation::hungarianAlgorithm(const Eigen::MatrixXd& cost_matrix) {
 int n_tracks = static_cast<int>(cost_matrix.rows());
 int n_measurements = static_cast<int>(cost_matrix.cols());
 int n = std::max(n_tracks, n_measurements);
 
 // Pad cost matrix if needed
 Eigen::MatrixXd C = Eigen::MatrixXd::Constant(n, n, 1e9);
 C.block(0, 0, n_tracks, n_measurements) = cost_matrix;
 
 // Hungarian algorithm (simplified: use greedy for now, full implementation can follow)
 std::vector<int> assignment(n_tracks, -1);
 std::vector<bool> measurement_used(n, false);
 
 // Greedy assignment: for each track, find best measurement
 for (int i = 0; i < n_tracks; i++) {
 int best_j = -1;
 double best_cost = GATING_THRESHOLD;
 
 for (int j = 0; j < n; j++) {
 if (!measurement_used[j] && C(i, j) < best_cost) {
 best_cost = C(i, j);
 best_j = j;
 }
 }
 
 if (best_j >= 0 && best_j < n_measurements) {
 assignment[i] = best_j;
 measurement_used[best_j] = true;
 }
 }
 
 return assignment;
}
std::map<std::string, int> DataAssociation::associate(
 const std::vector<Track>& tracks,
 const std::vector<Eigen::Matrix<double, 9, 9>>& covariances) {
 (void)covariances;
 
 std::map<std::string, int> result;
 
 if (tracks.empty() || pending_measurements_.empty()) {
 // No tracks or measurements: return empty
 for (size_t i = 0; i < pending_measurements_.size(); i++) {
 result["_new_" + std::to_string(i)] = static_cast<int>(i); // Mark as new track
 }
 return result;
 }
 
 int n_tracks = static_cast<int>(tracks.size());
 int n_measurements = static_cast<int>(pending_measurements_.size());
 
 // Build cost matrix
 cost_matrix_ = Eigen::MatrixXd::Constant(n_tracks, n_measurements, 1e9);
 
 for (int i = 0; i < n_tracks; i++) {
 // Predict measurement for this track
 Eigen::Matrix<double, 6, 1> z_pred;
 z_pred(0) = std::sqrt(tracks[i].filter_state(0)*tracks[i].filter_state(0) +
 tracks[i].filter_state(1)*tracks[i].filter_state(1));
 z_pred(1) = std::atan2(tracks[i].filter_state(1), tracks[i].filter_state(0));
 z_pred(2) = std::sqrt(tracks[i].filter_state(3)*tracks[i].filter_state(3) +
 tracks[i].filter_state(4)*tracks[i].filter_state(4) +
 tracks[i].filter_state(5)*tracks[i].filter_state(5));
 z_pred(3) = tracks[i].filter_state(2);
 z_pred(4) = tracks[i].filter_state(6);
 z_pred(5) = 1.0;
 
 // Innovation covariance (H*P*H^T + R, simplified)
 // Range is meters - use realistic diagonal so sequential updates associate.
 Eigen::Matrix<double, 6, 6> innovation_cov = Eigen::Matrix<double, 6, 6>::Identity();
 innovation_cov(0, 0) = 100.0 * 100.0; // range
 innovation_cov(1, 1) = 0.2 * 0.2; // bearing
 innovation_cov(2, 2) = 50.0 * 50.0; // velocity (init estimates are coarse)
 innovation_cov(3, 3) = 50.0 * 50.0; // altitude
 innovation_cov(4, 4) = 0.5 * 0.5; // heading
 innovation_cov(5, 5) = 1.0;
 
 for (int j = 0; j < n_measurements; j++) {
 double mahal = computeMahalanobisDistance(pending_measurements_[j], z_pred, innovation_cov);
 cost_matrix_(i, j) = mahal;
 }
 }
 
 // Run Hungarian algorithm
 std::vector<int> assignment = hungarianAlgorithm(cost_matrix_);
 
 // Build result map
 for (int i = 0; i < n_tracks; i++) {
 result[tracks[i].track_id] = assignment[i];
 }
 
 // Mark new measurements
 std::vector<bool> measurement_used(n_measurements, false);
 for (int j : assignment) {
 if (j >= 0 && j < n_measurements) {
 measurement_used[j] = true;
 }
 }
 
 for (int j = 0; j < n_measurements; j++) {
 if (!measurement_used[j]) {
 result["_new_" + std::to_string(j)] = j;
 }
 }
 
 return result;
}
Track DataAssociation::createTrackFromMeasurement(
 const Eigen::Matrix<double, 6, 1>& z,
 const std::string& sensor_type,
 double timestamp) {
 
 Track track;
 track.track_id = "_temp_id"; // Will be overwritten by TrackManager
 track.state = Track::State::TENTATIVE;
 track.age = 1;
 track.last_update_timestamp = timestamp;
 track.filter_state = Eigen::Matrix<double, 9, 1>::Zero();
 
 // Initialize state from measurement
 double range = z(0);
 double bearing = z(1);
 double velocity = z(2);
 double altitude = z(3);
 double heading = z(4);
 
 track.filter_state(0) = range * std::cos(bearing);
 track.filter_state(1) = range * std::sin(bearing);
 track.filter_state(2) = altitude;
 track.filter_state(3) = velocity * std::cos(heading) * 0.5;
 track.filter_state(4) = velocity * std::sin(heading) * 0.5;
 track.filter_state(5) = 0.0;
 track.filter_state(6) = heading;
 track.filter_state(7) = 0.0;
 track.filter_state(8) = 0.0;
 
 track.filter_covariance = Eigen::Matrix<double, 9, 9>::Identity() * 100.0;
 
 // Record measurement
 Track::MeasurementRecord rec;
 rec.timestamp = timestamp;
 rec.measurement = z;
 rec.sensor_type = sensor_type;
 rec.innovation_norm = 0.0;
 track.measurement_history.push_back(rec);
 
 // Update sensor agreement
 if (sensor_type == "RADAR") track.sensor_agreement.radar_count++;
 else if (sensor_type == "RF") track.sensor_agreement.rf_count++;
 else if (sensor_type == "OPTICAL") track.sensor_agreement.optical_count++;
 
 return track;
}
void DataAssociation::updateTrackState(Track& track,
 int assigned_measurement_idx,
 double timestamp) {
 track.age++;
 track.last_update_timestamp = timestamp;
 
 if (assigned_measurement_idx >= 0 && assigned_measurement_idx < (int)pending_measurements_.size()) {
 // Track was assigned a measurement
 track.consecutive_misses = 0;
 
 // Record measurement
 Track::MeasurementRecord rec;
 rec.timestamp = timestamp;
 rec.measurement = pending_measurements_[assigned_measurement_idx];
 rec.sensor_type = measurement_sensor_types_[assigned_measurement_idx];
 rec.innovation_norm = (cost_matrix_.rows() > 0 &&
 assigned_measurement_idx < cost_matrix_.cols())
 ? cost_matrix_(0, assigned_measurement_idx)
 : 0.0; // Simplified
 track.measurement_history.push_back(rec);
 
 // Keep only last HISTORY_SIZE measurements
 if (track.measurement_history.size() > Track::HISTORY_SIZE) {
 track.measurement_history.erase(track.measurement_history.begin());
 }
 
 // Update sensor agreement
 if (rec.sensor_type == "RADAR") track.sensor_agreement.radar_count++;
 else if (rec.sensor_type == "RF") track.sensor_agreement.rf_count++;
 else if (rec.sensor_type == "OPTICAL") track.sensor_agreement.optical_count++;
 
 // Promote / restore on hit
 if (track.state == Track::State::COASTED) {
 track.state = Track::State::TENTATIVE;
 }
 if (track.state == Track::State::TENTATIVE && track.age >= CONFIRMATION_THRESHOLD) {
 track.state = Track::State::CONFIRMED;
 }
 } else {
 // Track coasted (no measurement assigned)
 track.consecutive_misses++;
 
 if (track.consecutive_misses >= MAX_COASTING_FRAMES) {
 track.state = Track::State::DELETED;
 } else if (track.state == Track::State::CONFIRMED ||
 track.state == Track::State::TENTATIVE ||
 track.state == Track::State::COASTED) {
 track.state = Track::State::COASTED;
 }
 }
}
void DataAssociation::pruneTracks(std::vector<Track>& tracks, double current_timestamp) {
 auto it = std::remove_if(tracks.begin(), tracks.end(),
 [current_timestamp](const Track& t) {
 if (t.state == Track::State::DELETED) return true;
 if (current_timestamp - t.last_update_timestamp > MAX_TRACK_AGE) return true;
 return false;
 });
 tracks.erase(it, tracks.end());
}
std::vector<int> DataAssociation::getUnassignedMeasurements() const {
 std::vector<int> unassigned;
 // Would be populated during association
 return unassigned;
}
std::vector<Track::MeasurementRecord> DataAssociation::getMeasurementHistory(
 const std::string& track_id) const {
 (void)track_id;
 // Would return from track storage
 return {};
}
// ============================================================================
// Track Manager Implementation
// ============================================================================
TrackManager::TrackManager() {}
std::string TrackManager::generateTrackId() {
 return "TRACK_" + std::to_string(next_track_id_++);
}
void TrackManager::update(const std::vector<Eigen::Matrix<double, 6, 1>>& measurements,
 const std::vector<std::string>& sensor_types,
 const std::vector<Eigen::Matrix<double, 9, 1>>& filter_states,
 const std::vector<Eigen::Matrix<double, 9, 9>>& filter_covariances,
 double timestamp) {
 
 associator_.clearPending();
 
 // Add all measurements
 for (size_t i = 0; i < measurements.size(); i++) {
 associator_.addMeasurement(measurements[i], sensor_types[i], timestamp);
 }
 
 // Get covariances for all tracks
 std::vector<Eigen::Matrix<double, 9, 9>> track_covs;
 for (const auto& t : tracks_) {
 track_covs.push_back(t.filter_covariance);
 }
 
 // Perform association
 auto associations = associator_.associate(tracks_, track_covs);
 
 // Update existing tracks
 for (auto& track : tracks_) {
 int meas_idx = -1;
 if (associations.count(track.track_id) > 0) {
 meas_idx = associations[track.track_id];
 }
 
 if (meas_idx >= 0 && meas_idx < (int)measurements.size()) {
 if (meas_idx < (int)filter_states.size() &&
 meas_idx < (int)filter_covariances.size() &&
 filter_states[meas_idx].squaredNorm() > 1e-18) {
 track.filter_state = filter_states[meas_idx];
 track.filter_covariance = filter_covariances[meas_idx];
 } else {
 // Caller passed placeholder zeros - refresh kinematics from measurement
 Track refreshed = associator_.createTrackFromMeasurement(
 measurements[meas_idx], sensor_types[meas_idx], timestamp);
 track.filter_state = refreshed.filter_state;
 track.filter_covariance = refreshed.filter_covariance;
 }
 }
 
 associator_.updateTrackState(track, meas_idx, timestamp);
 }
 
 // Create new tracks from unassigned measurements
 std::vector<bool> measurement_used(measurements.size(), false);
 for (const auto& assoc : associations) {
 // Skip synthetic "_new_*" keys when marking used - only real track assignments
 if (assoc.first.rfind("_new_", 0) == 0) {
 continue;
 }
 if (assoc.second >= 0 && assoc.second < (int)measurement_used.size()) {
 measurement_used[assoc.second] = true;
 }
 }
 
 for (size_t i = 0; i < measurements.size(); i++) {
 if (!measurement_used[i]) {
 Track new_track = associator_.createTrackFromMeasurement(
 measurements[i], sensor_types[i], timestamp);
 new_track.track_id = generateTrackId();
 tracks_.push_back(new_track);
 }
 }
 
 // Prune dead tracks
 associator_.pruneTracks(tracks_, timestamp);
 associator_.clearPending();
}
std::vector<Track> TrackManager::getActiveTracks() const {
 std::vector<Track> active;
 for (const auto& t : tracks_) {
 if (t.state != Track::State::DELETED) {
 active.push_back(t);
 }
 }
 return active;
}
std::vector<Track> TrackManager::getConfirmedTracks() const {
 std::vector<Track> confirmed;
 for (const auto& t : tracks_) {
 if (t.state == Track::State::CONFIRMED) {
 confirmed.push_back(t);
 }
 }
 return confirmed;
}
std::optional<Track> TrackManager::getTrack(const std::string& track_id) const {
 for (const auto& t : tracks_) {
 if (t.track_id == track_id) {
 return t;
 }
 }
 return std::nullopt;
}
size_t TrackManager::getConfirmedTrackCount() const {
 return getConfirmedTracks().size();
}
void TrackManager::deleteTrack(const std::string& track_id) {
 auto it = std::find_if(tracks_.begin(), tracks_.end(),
 [&track_id](const Track& t) { return t.track_id == track_id; });
 if (it != tracks_.end()) {
 tracks_.erase(it);
 }
}
std::vector<Track::MeasurementRecord> TrackManager::getTrackHistory(
 const std::string& track_id) const {
 auto track = getTrack(track_id);
 if (track) {
 return track->measurement_history;
 }
 return {};
}
TrackManager::Statistics TrackManager::getStatistics() const {
 Statistics stats;
 stats.total_tracks = static_cast<int>(tracks_.size());
 
 for (const auto& t : tracks_) {
 if (t.state == Track::State::CONFIRMED) stats.confirmed_tracks++;
 else if (t.state == Track::State::TENTATIVE) stats.tentative_tracks++;
 else if (t.state == Track::State::COASTED) stats.coasted_tracks++;
 }
 
 return stats;
}
} // namespace Polybolos::Fusion
