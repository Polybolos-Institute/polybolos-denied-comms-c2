#include "fusion/multi_sensor_fusion.h"

#include <chrono>
#include <cmath>

namespace Polybolos {
namespace Fusion {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg2Rad = kPi / 180.0;
constexpr double kEarthRadiusM = 6378137.0;

void bearingRangeToEnu(double range_m, double bearing_deg, double& east_m, double& north_m) {
    const double br = bearing_deg * kDeg2Rad;
    north_m = range_m * std::cos(br);
    east_m = range_m * std::sin(br);
}

void enuToLatLon(double origin_lat, double origin_lon, double east_m, double north_m,
                 double& lat, double& lon) {
    lat = origin_lat + (north_m / kEarthRadiusM) * (180.0 / kPi);
    const double cos_lat = std::cos(origin_lat * kDeg2Rad);
    const double safe = (std::fabs(cos_lat) < 1e-6) ? 1e-6 : cos_lat;
    lon = origin_lon + (east_m / (kEarthRadiusM * safe)) * (180.0 / kPi);
}

}  // namespace

MultiSensorFusion::MultiSensorFusion() {
    sensor_credibility_[SensorObservation::Type::RADAR] = 1.0;
    sensor_credibility_[SensorObservation::Type::RF] = 1.0;
    sensor_credibility_[SensorObservation::Type::OPTICAL] = 1.0;
}

void MultiSensorFusion::setSensorOrigin(double lat_deg, double lon_deg, double alt_m) {
    origin_lat_ = lat_deg;
    origin_lon_ = lon_deg;
    origin_alt_ = alt_m;
}

void MultiSensorFusion::addObservation(const SensorObservation& obs) {
    observations_.push_back(obs);
}

bool MultiSensorFusion::observationToZ(const SensorObservation& obs,
                                       Eigen::Matrix<double, 6, 1>& z,
                                       std::string& sensor_type) const {
    z = Eigen::Matrix<double, 6, 1>::Zero();
    double east = 0.0;
    double north = 0.0;
    double alt = origin_alt_;
    double speed = 0.0;
    double heading = 0.0;
    double confidence = obs.sensor_credibility;

    if (obs.sensor_type == SensorObservation::Type::RADAR) {
        if (obs.radar.range <= 0.0) {
            return false;
        }
        bearingRangeToEnu(obs.radar.range, obs.radar.bearing, east, north);
        speed = obs.radar.velocity;
        heading = obs.radar.bearing * kDeg2Rad;
        confidence = 0.9;
        sensor_type = "RADAR";
    } else if (obs.sensor_type == SensorObservation::Type::RF) {
        const double path = std::pow(10.0, (-40.0 - obs.rf.rssi) / 20.0);
        const double range = std::max(50.0, path);
        bearingRangeToEnu(range, obs.rf.direction, east, north);
        heading = obs.rf.direction * kDeg2Rad;
        confidence = 0.7;
        sensor_type = "RF";
    } else {
        constexpr double kFx = 800.0;
        constexpr double kFy = 800.0;
        constexpr double kCx = 640.0;
        constexpr double kCy = 360.0;
        const double zcam = std::max(50.0, origin_alt_);
        east = (obs.optical.pixel_x - kCx) * zcam / kFx;
        north = -(obs.optical.pixel_y - kCy) * zcam / kFy;
        confidence = obs.optical.confidence;
        if (confidence <= 0.05) {
            return false;
        }
        sensor_type = "OPTICAL";
    }

    const double range = std::sqrt(east * east + north * north);
    const double bearing = std::atan2(east, north);
    z(0) = range;
    z(1) = bearing;
    z(2) = speed;
    z(3) = alt;
    z(4) = heading;
    z(5) = confidence;
    return true;
}

FusedTrack MultiSensorFusion::toFusedTrack(const Track& t) const {
    FusedTrack out;
    out.track_id = t.track_id;
    enuToLatLon(origin_lat_, origin_lon_, t.longitude(), t.latitude(), out.latitude, out.longitude);
    out.altitude = t.altitude();
    out.velocity = t.speed();
    out.heading = t.heading();
    out.pitch = t.filter_state(7);
    out.roll = t.filter_state(8);
    out.pos_uncertainty = std::sqrt(std::max(0.0, t.filter_covariance(0, 0) + t.filter_covariance(1, 1)));
    out.vel_uncertainty = std::sqrt(std::max(0.0, t.filter_covariance(3, 3) + t.filter_covariance(4, 4)));
    out.hit_count = t.age;
    out.coast_count = t.consecutive_misses;
    out.timestamp = t.last_update_timestamp;
    out.consensus.radar_agrees = t.sensor_agreement.radar_count > 0;
    out.consensus.rf_agrees = t.sensor_agreement.rf_count > 0;
    out.consensus.optical_agrees = t.sensor_agreement.optical_count > 0;
    out.consensus.belief_mass =
        std::min(1.0, 0.4 * (out.consensus.radar_agrees ? 1.0 : 0.0) +
                          0.3 * (out.consensus.rf_agrees ? 1.0 : 0.0) +
                          0.3 * (out.consensus.optical_agrees ? 1.0 : 0.0));
    return out;
}

void MultiSensorFusion::processCycle(double dt_sec) {
    using clock = std::chrono::steady_clock;
    const auto t0 = clock::now();
    time_sec_ += dt_sec;

    stats_.observations_in = static_cast<int>(observations_.size());

    std::vector<Eigen::Matrix<double, 6, 1>> measurements;
    std::vector<std::string> sensor_types;
    std::vector<Eigen::Matrix<double, 9, 1>> filter_states;
    std::vector<Eigen::Matrix<double, 9, 9>> filter_covs;

    for (const auto& obs : observations_) {
        Eigen::Matrix<double, 6, 1> z;
        std::string stype;
        if (!observationToZ(obs, z, stype)) {
            continue;
        }
        measurements.push_back(z);
        sensor_types.push_back(stype);

        // Run a one-shot filter update for the measurement state estimate
        if (filter_kind_ == FilterKind::UKF) {
            UnscentedKalmanFilter ukf;
            ukf.initialize(z, time_sec_);
            ukf.predict(dt_sec);
            Eigen::Matrix<double, 6, 6> R = Eigen::Matrix<double, 6, 6>::Identity();
            ukf.update(z, R);
            filter_states.push_back(ukf.getState());
            filter_covs.push_back(ukf.getCovariance());
        } else {
            ExtendedKalmanFilter ekf;
            ekf.initialize(z, time_sec_);
            ekf.predict(dt_sec);
            Eigen::Matrix<double, 6, 6> R = Eigen::Matrix<double, 6, 6>::Identity();
            ekf.update(z, R);
            filter_states.push_back(ekf.getState());
            filter_covs.push_back(ekf.getCovariance());
        }
    }
    observations_.clear();

    const size_t before = track_manager_.getTrackCount();
    track_manager_.update(measurements, sensor_types, filter_states, filter_covs, time_sec_);
    stats_.associations = static_cast<int>(measurements.size());
    (void)before;

    updateBeliefs();
    resolveContradictions();

    const auto t1 = clock::now();
    stats_.last_cycle_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    stats_.tracks_alive = static_cast<int>(track_manager_.getActiveTracks().size());
}

void MultiSensorFusion::updateBeliefs() { (void)network_; }
void MultiSensorFusion::resolveContradictions() {}

std::vector<FusedTrack> MultiSensorFusion::getFusedTracks() const {
    std::vector<FusedTrack> out;
    for (const auto& t : track_manager_.getActiveTracks()) {
        out.push_back(toFusedTrack(t));
    }
    return out;
}

std::optional<FusedTrack> MultiSensorFusion::getTrack(const std::string& track_id) const {
    auto t = track_manager_.getTrack(track_id);
    if (!t) {
        return std::nullopt;
    }
    return toFusedTrack(*t);
}

void MultiSensorFusion::updateSensorCredibility(SensorObservation::Type type, double credibility) {
    sensor_credibility_[type] = credibility;
}

FusedTrack MultiSensorFusion::naiveFuse(const std::vector<SensorObservation>& obs,
                                        double origin_lat, double origin_lon,
                                        double origin_alt) {
    MultiSensorFusion tmp;
    tmp.setSensorOrigin(origin_lat, origin_lon, origin_alt);
    double se = 0.0, sn = 0.0, sa = 0.0;
    int n = 0;
    for (const auto& o : obs) {
        Eigen::Matrix<double, 6, 1> z;
        std::string st;
        if (!tmp.observationToZ(o, z, st)) {
            continue;
        }
        se += z(0) * std::sin(z(1));
        sn += z(0) * std::cos(z(1));
        sa += z(3);
        ++n;
    }
    FusedTrack t;
    t.track_id = "naive";
    if (n > 0) {
        enuToLatLon(origin_lat, origin_lon, se / n, sn / n, t.latitude, t.longitude);
        t.altitude = sa / n;
    }
    return t;
}

}  // namespace Fusion
}  // namespace Polybolos
