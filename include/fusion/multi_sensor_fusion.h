#pragma once

#include <map>
#include <optional>
#include <string>
#include <vector>

#include "bayesian/bayesian_network.h"
#include "filters/kalman_filter.h"
#include "fusion/data_association.h"

namespace Polybolos {
namespace Fusion {

/// Sensor observation.
struct SensorObservation {
    enum class Type { RADAR, RF, OPTICAL };

    Type sensor_type = Type::RADAR;
    double timestamp = 0.0;

    struct {
        double range = 0.0;
        double bearing = 0.0;
        double velocity = 0.0;
        double rcs = 0.0;
    } radar;

    struct {
        double rssi = 0.0;
        double direction = 0.0;
        double frequency = 0.0;
        std::string emitter_id;
    } rf;

    struct {
        double pixel_x = 0.0;
        double pixel_y = 0.0;
        double bbox_w = 0.0;
        double bbox_h = 0.0;
        double confidence = 0.0;
        double thermal_sig = 0.0;
    } optical;

    double sensor_credibility = 1.0;
};

/// Fused track (output of fusion).
struct FusedTrack {
    std::string track_id;
    double latitude = 0.0;
    double longitude = 0.0;
    double altitude = 0.0;
    double velocity = 0.0;
    double heading = 0.0;
    double pitch = 0.0;
    double roll = 0.0;

    double pos_uncertainty = 0.0;
    double vel_uncertainty = 0.0;

    struct {
        bool radar_agrees = false;
        bool rf_agrees = false;
        bool optical_agrees = false;
        double belief_mass = 0.0;
    } consensus;

    int hit_count = 0;
    int coast_count = 0;
    double timestamp = 0.0;
};

struct FusionCycleStats {
    double last_cycle_ms = 0.0;
    int observations_in = 0;
    int associations = 0;
    int tracks_alive = 0;
};

enum class FilterKind { EKF, UKF };

/// Multi-sensor fusion engine.
class MultiSensorFusion {
public:
    MultiSensorFusion();

    void setFilterKind(FilterKind kind) { filter_kind_ = kind; }
    void setSensorOrigin(double lat_deg, double lon_deg, double alt_m);
    void setAssociationGate(double /*gate*/) {}  // gating lives in DataAssociation
    void setMaxCoastFrames(int /*n*/) {}

    void addObservation(const SensorObservation& obs);
    void processCycle(double dt_sec);

    void updateBeliefs();
    void resolveContradictions();

    std::vector<FusedTrack> getFusedTracks() const;
    std::optional<FusedTrack> getTrack(const std::string& track_id) const;

    void updateSensorCredibility(SensorObservation::Type type, double credibility);
    FusionCycleStats stats() const { return stats_; }

    static FusedTrack naiveFuse(const std::vector<SensorObservation>& obs,
                                double origin_lat, double origin_lon, double origin_alt);

private:
    bool observationToZ(const SensorObservation& obs,
                        Eigen::Matrix<double, 6, 1>& z,
                        std::string& sensor_type) const;
    FusedTrack toFusedTrack(const Track& t) const;

    BayesianNetwork network_;
    TrackManager track_manager_;
    FilterKind filter_kind_ = FilterKind::EKF;
    std::vector<SensorObservation> observations_;
    std::map<SensorObservation::Type, double> sensor_credibility_;
    FusionCycleStats stats_;

    double origin_lat_ = 38.8977;
    double origin_lon_ = -77.0365;
    double origin_alt_ = 100.0;
    double time_sec_ = 0.0;
};

}  // namespace Fusion
}  // namespace Polybolos
