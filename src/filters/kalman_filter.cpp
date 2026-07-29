#include "filters/kalman_filter.h"
#include <cmath>
#include <algorithm>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
namespace Polybolos::Fusion {
// ============================================================================
// Extended Kalman Filter Implementation
// ============================================================================
ExtendedKalmanFilter::ExtendedKalmanFilter(const Config& config)
    : config_(config) {
    
    // Initialize state to zero
    x_ = StateVector::Zero();
    
    // Initialize covariance (high uncertainty)
    P_ = CovarianceMatrix::Identity() * 100.0;
    P_.block<3,3>(0, 0) *= 1000.0;  // Position uncertainty high initially
    
    // Process noise (constant velocity model)
    Q_ = CovarianceMatrix::Zero();
    Q_.block<3,3>(0, 0) = Eigen::Matrix3d::Identity() * config_.process_noise_pos;
    Q_.block<3,3>(3, 3) = Eigen::Matrix3d::Identity() * config_.process_noise_vel;
    Q_.block<3,3>(6, 6) = Eigen::Matrix3d::Identity() * config_.process_noise_angle;
    
    // Measurement noise
    R_ = MeasurementCovarianceMatrix::Zero();
    R_(0, 0) = config_.measurement_noise_pos * config_.measurement_noise_pos;      // range
    R_(1, 1) = config_.measurement_noise_angle * config_.measurement_noise_angle;  // bearing
    R_(2, 2) = config_.measurement_noise_vel * config_.measurement_noise_vel;      // velocity
    R_(3, 3) = config_.measurement_noise_pos * config_.measurement_noise_pos;      // altitude
    R_(4, 4) = config_.measurement_noise_angle * config_.measurement_noise_angle;  // heading
    R_(5, 5) = 0.01;  // confidence measurement noise
}
void ExtendedKalmanFilter::initialize(const MeasurementVector& z, double timestamp) {
    // Initialize state from first measurement
    // z = [range, bearing, velocity, altitude, heading, confidence]
    (void)timestamp;
    
    // For simplicity, assume bearing is absolute direction
    double range = z(0);
    double bearing = z(1);
    double velocity = z(2);
    double altitude = z(3);
    double heading = z(4);
    
    // Assume reference frame at (0, 0) for now (would normally integrate with GPS)
    x_(0) = range * std::cos(bearing);  // lat (simplified)
    x_(1) = range * std::sin(bearing);  // lon (simplified)
    x_(2) = altitude;
    x_(3) = velocity * std::cos(heading) * 0.5;  // vx (rough estimate)
    x_(4) = velocity * std::sin(heading) * 0.5;  // vy
    x_(5) = 0.0;  // vz
    x_(6) = heading;
    x_(7) = 0.0;  // pitch
    x_(8) = 0.0;  // roll
    
    // Reset covariance to initial uncertainty
    P_ = CovarianceMatrix::Identity() * 100.0;
    P_.block<3,3>(0, 0) *= 50.0;  // Reduced after first measurement
}
ExtendedKalmanFilter::StateVector ExtendedKalmanFilter::stateTransition(const StateVector& x, double dt) const {
    StateVector x_new = x;
    
    // Constant velocity model: position += velocity * dt
    x_new(0) += x(3) * dt;  // lat += vx * dt
    x_new(1) += x(4) * dt;  // lon += vy * dt
    x_new(2) += x(5) * dt;  // alt += vz * dt
    
    // Velocity stays constant (process noise handles deviations)
    // Heading stays constant
    // Pitch, roll stay constant
    
    return x_new;
}
ExtendedKalmanFilter::MeasurementVector ExtendedKalmanFilter::measurementFunction(
    const StateVector& x) const {
    
    MeasurementVector z = MeasurementVector::Zero();
    
    // Compute range (Euclidean distance to (0,0,0))
    double range = std::sqrt(x(0)*x(0) + x(1)*x(1));
    z(0) = range;
    
    // Compute bearing
    double bearing = std::atan2(x(1), x(0));
    z(1) = bearing;
    
    // Compute velocity magnitude
    double velocity = std::sqrt(x(3)*x(3) + x(4)*x(4) + x(5)*x(5));
    z(2) = velocity;
    
    // Altitude is direct
    z(3) = x(2);
    
    // Heading is direct
    z(4) = x(6);
    
    // Confidence (not modeled, assumed 1.0)
    z(5) = 1.0;
    
    return z;
}
ExtendedKalmanFilter::JacobianMatrix ExtendedKalmanFilter::computeH() const {
    JacobianMatrix H = JacobianMatrix::Zero();
    
    double lat = x_(0);
    double lon = x_(1);
    double range = std::sqrt(lat*lat + lon*lon);
    
    if (range > 1e-6) {
        // ∂range/∂[lat, lon, ...]
        H(0, 0) = lat / range;
        H(0, 1) = lon / range;
        
        // ∂bearing/∂[lat, lon, ...]
        H(1, 0) = -lon / (range * range);
        H(1, 1) = lat / (range * range);
    }
    
    // ∂velocity/∂[vx, vy, vz]
    double velocity = std::sqrt(x_(3)*x_(3) + x_(4)*x_(4) + x_(5)*x_(5));
    if (velocity > 1e-6) {
        H(2, 3) = x_(3) / velocity;
        H(2, 4) = x_(4) / velocity;
        H(2, 5) = x_(5) / velocity;
    }
    
    // ∂altitude/∂alt
    H(3, 2) = 1.0;
    
    // ∂heading/∂heading
    H(4, 6) = 1.0;
    
    // ∂confidence/∂anything = 0 (constant)
    
    return H;
}
Eigen::Matrix<double, ExtendedKalmanFilter::STATE_SIZE, ExtendedKalmanFilter::STATE_SIZE>
ExtendedKalmanFilter::computeF(double dt) const {
    using FMatrix = Eigen::Matrix<double, STATE_SIZE, STATE_SIZE>;
    FMatrix F = FMatrix::Identity();
    
    // Position += velocity * dt
    F(0, 3) = dt;  // dlat/dvx
    F(1, 4) = dt;  // dlon/dvy
    F(2, 5) = dt;  // dalt/dvz
    
    return F;
}
void ExtendedKalmanFilter::predict(double dt) {
    // Predict state: x = f(x, dt)
    x_ = stateTransition(x_, dt);
    
    // Predict covariance: P = F*P*F^T + Q
    auto F = computeF(dt);
    P_ = F * P_ * F.transpose() + Q_;
    
    // Ensure symmetry
    P_ = (P_ + P_.transpose()) / 2.0;
}
void ExtendedKalmanFilter::update(const MeasurementVector& z,
                                  const MeasurementCovarianceMatrix& R) {
    // Compute innovation (measurement residual)
    MeasurementVector z_pred = measurementFunction(x_);
    
    // Handle circular (angle) residuals
    Eigen::VectorXd y = z - z_pred;
    
    // Bearing (y(1)) and heading (y(4)) are circular
    while (y(1) > M_PI) y(1) -= 2*M_PI;
    while (y(1) < -M_PI) y(1) += 2*M_PI;
    while (y(4) > M_PI) y(4) -= 2*M_PI;
    while (y(4) < -M_PI) y(4) += 2*M_PI;
    
    // Compute Jacobian
    JacobianMatrix H = computeH();
    
    // Compute innovation covariance: S = H*P*H^T + R
    MeasurementCovarianceMatrix S = H * P_ * H.transpose() + R;
    
    // Compute Kalman gain: K = P*H^T*S^-1
    Eigen::Matrix<double, STATE_SIZE, MEASUREMENT_SIZE> K = P_ * H.transpose() * S.inverse();
    
    // Update state: x = x + K*y
    x_ += K * y;
    
    // Update covariance: P = (I - K*H)*P
    CovarianceMatrix I = CovarianceMatrix::Identity();
    P_ = (I - K * H) * P_;
    
    // Ensure symmetry
    P_ = (P_ + P_.transpose()) / 2.0;
    
    // Adaptive noise covariance
    adaptNoiseCovariance(y);
}
double ExtendedKalmanFilter::mahalanobisDistance(const MeasurementVector& z,
                                                 const MeasurementCovarianceMatrix& R) {
    MeasurementVector z_pred = measurementFunction(x_);
    Eigen::VectorXd y = z - z_pred;
    
    // Handle angles
    while (y(1) > M_PI) y(1) -= 2*M_PI;
    while (y(1) < -M_PI) y(1) += 2*M_PI;
    while (y(4) > M_PI) y(4) -= 2*M_PI;
    while (y(4) < -M_PI) y(4) += 2*M_PI;
    
    JacobianMatrix H = computeH();
    MeasurementCovarianceMatrix S = H * P_ * H.transpose() + R;
    
    return std::sqrt(y.transpose() * S.inverse() * y);
}
void ExtendedKalmanFilter::adaptNoiseCovariance(const Eigen::VectorXd& residuals) {
    // Track recent residuals for adaptive filtering
    recent_residuals_.insert(recent_residuals_.end(),
                              residuals.data(),
                              residuals.data() + residuals.size());
    
    if (recent_residuals_.size() > RESIDUAL_WINDOW) {
        recent_residuals_.erase(recent_residuals_.begin(),
                                 recent_residuals_.begin() +
                                 (recent_residuals_.size() - RESIDUAL_WINDOW));
    }
    
    // Compute innovation variance from recent history
    if (recent_residuals_.size() > 10) {
        double mean = 0.0;
        for (double r : recent_residuals_) mean += r;
        mean /= recent_residuals_.size();
        
        double variance = 0.0;
        for (double r : recent_residuals_) variance += (r - mean) * (r - mean);
        variance /= recent_residuals_.size();
        
        // Slightly increase measurement noise if residuals are large
        if (variance > 10.0) {
            R_ *= 1.01;  // Slow adaptation
        }
    }
}
ExtendedKalmanFilter::MeasurementVector ExtendedKalmanFilter::predictMeasurement() const {
    return measurementFunction(x_);
}
double ExtendedKalmanFilter::circularMean(double a1, double a2, double w1, double w2) const {
    const double s = w1 * std::sin(a1) + w2 * std::sin(a2);
    const double c = w1 * std::cos(a1) + w2 * std::cos(a2);
    return std::atan2(s, c);
}
// ============================================================================
// Unscented Kalman Filter Implementation
// ============================================================================
UnscentedKalmanFilter::UnscentedKalmanFilter(const Config& config)
    : config_(config) {
    
    x_ = StateVector::Zero();
    P_ = CovarianceMatrix::Identity() * 100.0;
    P_.block<3,3>(0, 0) *= 1000.0;
    
    Q_ = CovarianceMatrix::Zero();
    Q_.block<3,3>(0, 0) = Eigen::Matrix3d::Identity() * config_.process_noise_pos;
    Q_.block<3,3>(3, 3) = Eigen::Matrix3d::Identity() * config_.process_noise_vel;
    Q_.block<3,3>(6, 6) = Eigen::Matrix3d::Identity() * config_.process_noise_angle;
    
    R_ = MeasurementCovarianceMatrix::Zero();
    R_(0, 0) = config_.measurement_noise_pos * config_.measurement_noise_pos;
    R_(1, 1) = config_.measurement_noise_angle * config_.measurement_noise_angle;
    R_(2, 2) = config_.measurement_noise_vel * config_.measurement_noise_vel;
    R_(3, 3) = config_.measurement_noise_pos * config_.measurement_noise_pos;
    R_(4, 4) = config_.measurement_noise_angle * config_.measurement_noise_angle;
    R_(5, 5) = 0.01;
    
    // Compute UKF weights
    lambda_ = config_.alpha * config_.alpha * (STATE_SIZE + config_.kappa) - STATE_SIZE;
    
    w_mean_.resize(2 * STATE_SIZE + 1);
    w_cov_.resize(2 * STATE_SIZE + 1);
    
    w_mean_[0] = lambda_ / (STATE_SIZE + lambda_);
    w_cov_[0] = lambda_ / (STATE_SIZE + lambda_) + (1 - config_.alpha * config_.alpha + config_.beta);
    
    for (int i = 1; i < 2 * STATE_SIZE + 1; i++) {
        w_mean_[i] = 1.0 / (2.0 * (STATE_SIZE + lambda_));
        w_cov_[i] = 1.0 / (2.0 * (STATE_SIZE + lambda_));
    }
}
void UnscentedKalmanFilter::initialize(const MeasurementVector& z, double timestamp) {
    (void)timestamp;
    // Same as EKF for initialization
    double range = z(0);
    double bearing = z(1);
    double velocity = z(2);
    double altitude = z(3);
    double heading = z(4);
    
    x_(0) = range * std::cos(bearing);
    x_(1) = range * std::sin(bearing);
    x_(2) = altitude;
    x_(3) = velocity * std::cos(heading) * 0.5;
    x_(4) = velocity * std::sin(heading) * 0.5;
    x_(5) = 0.0;
    x_(6) = heading;
    x_(7) = 0.0;
    x_(8) = 0.0;
    
    P_ = CovarianceMatrix::Identity() * 100.0;
    P_.block<3,3>(0, 0) *= 50.0;
}
std::vector<UnscentedKalmanFilter::StateVector>
UnscentedKalmanFilter::generateSigmaPoints(const StateVector& x, const CovarianceMatrix& P) {
    std::vector<StateVector> sigma_points(2 * STATE_SIZE + 1);
    
    // Compute square root of P
    Eigen::LLT<CovarianceMatrix> llt(P);
    CovarianceMatrix L = llt.matrixL();
    
    sigma_points[0] = x;
    
    for (int i = 0; i < STATE_SIZE; i++) {
        sigma_points[1 + i] = x + std::sqrt(STATE_SIZE + lambda_) * L.col(i);
        sigma_points[1 + STATE_SIZE + i] = x - std::sqrt(STATE_SIZE + lambda_) * L.col(i);
    }
    
    return sigma_points;
}
UnscentedKalmanFilter::StateVector
UnscentedKalmanFilter::weightedMean(const std::vector<StateVector>& sigma_points) {
    StateVector mean = StateVector::Zero();
    for (size_t i = 0; i < sigma_points.size(); i++) {
        mean += w_mean_[i] * sigma_points[i];
    }
    return mean;
}
UnscentedKalmanFilter::MeasurementVector
UnscentedKalmanFilter::weightedMeasurementMean(const std::vector<MeasurementVector>& sigma_measurements) {
    MeasurementVector mean = MeasurementVector::Zero();
    for (size_t i = 0; i < sigma_measurements.size(); i++) {
        mean += w_mean_[i] * sigma_measurements[i];
    }
    return mean;
}
UnscentedKalmanFilter::UnscentedKalmanFilter::StateVector UnscentedKalmanFilter::stateTransition(const StateVector& x, double dt) const {
    StateVector x_new = x;
    x_new(0) += x(3) * dt;
    x_new(1) += x(4) * dt;
    x_new(2) += x(5) * dt;
    return x_new;
}
UnscentedKalmanFilter::MeasurementVector
UnscentedKalmanFilter::measurementFunction(const StateVector& x) const {
    MeasurementVector z = MeasurementVector::Zero();
    double range = std::sqrt(x(0)*x(0) + x(1)*x(1));
    z(0) = range;
    z(1) = std::atan2(x(1), x(0));
    z(2) = std::sqrt(x(3)*x(3) + x(4)*x(4) + x(5)*x(5));
    z(3) = x(2);
    z(4) = x(6);
    z(5) = 1.0;
    return z;
}
void UnscentedKalmanFilter::predict(double dt) {
    // Generate sigma points
    auto sigma_points = generateSigmaPoints(x_, P_);
    
    // Propagate sigma points
    std::vector<StateVector> sigma_points_pred(sigma_points.size());
    for (size_t i = 0; i < sigma_points.size(); i++) {
        sigma_points_pred[i] = stateTransition(sigma_points[i], dt);
    }
    
    // Compute predicted mean and covariance
    x_ = weightedMean(sigma_points_pred);
    
    P_ = CovarianceMatrix::Zero();
    for (size_t i = 0; i < sigma_points_pred.size(); i++) {
        StateVector diff = sigma_points_pred[i] - x_;
        P_ += w_cov_[i] * diff * diff.transpose();
    }
    P_ += Q_;
}
void UnscentedKalmanFilter::update(const MeasurementVector& z,
                                   const MeasurementCovarianceMatrix& R) {
    // Generate sigma points
    auto sigma_points = generateSigmaPoints(x_, P_);
    
    // Propagate through measurement function
    std::vector<MeasurementVector> sigma_measurements(sigma_points.size());
    for (size_t i = 0; i < sigma_points.size(); i++) {
        sigma_measurements[i] = measurementFunction(sigma_points[i]);
    }
    
    // Compute predicted measurement mean
    MeasurementVector z_pred = weightedMeasurementMean(sigma_measurements);
    
    // Compute innovation covariance
    MeasurementCovarianceMatrix Pzz = MeasurementCovarianceMatrix::Zero();
    for (size_t i = 0; i < sigma_measurements.size(); i++) {
        MeasurementVector diff = sigma_measurements[i] - z_pred;
        Pzz += w_cov_[i] * diff * diff.transpose();
    }
    Pzz += R;
    
    // Compute cross-covariance
    Eigen::Matrix<double, STATE_SIZE, MEASUREMENT_SIZE> Pxz =
        Eigen::Matrix<double, STATE_SIZE, MEASUREMENT_SIZE>::Zero();
    for (size_t i = 0; i < sigma_points.size(); i++) {
        StateVector x_diff = sigma_points[i] - x_;
        MeasurementVector z_diff = sigma_measurements[i] - z_pred;
        Pxz += w_cov_[i] * x_diff * z_diff.transpose();
    }
    
    // Compute Kalman gain
    Eigen::Matrix<double, STATE_SIZE, MEASUREMENT_SIZE> K = Pxz * Pzz.inverse();
    
    // Update state and covariance
    x_ += K * (z - z_pred);
    P_ -= K * Pzz * K.transpose();
}
double UnscentedKalmanFilter::mahalanobisDistance(const MeasurementVector& z,
                                                  const MeasurementCovarianceMatrix& R) {
    // Simplified (would need full sigma point propagation for accuracy)
    MeasurementVector z_pred = measurementFunction(x_);
    Eigen::VectorXd y = z - z_pred;
    MeasurementCovarianceMatrix S = R + R;  // Placeholder
    return std::sqrt(y.transpose() * S.inverse() * y);
}
UnscentedKalmanFilter::MeasurementVector UnscentedKalmanFilter::predictMeasurement() const {
    return measurementFunction(x_);
}
double UnscentedKalmanFilter::getPositionUncertainty() const {
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es(
        P_.block<2,2>(0,0)
    );
    return std::sqrt(es.eigenvalues().maxCoeff());
}
}  // namespace Polybolos::Fusion
