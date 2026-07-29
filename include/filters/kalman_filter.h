#pragma once
#include <array>
#include <cmath>
#include <vector>
#include <optional>
#include <Eigen/Dense>
namespace Polybolos::Fusion {
// Extended Kalman Filter (EKF) for 9-DOF state
// State: [lat, lon, alt, vx, vy, vz, heading, pitch, roll]
class ExtendedKalmanFilter {
public:
    static constexpr int STATE_SIZE = 9;
    static constexpr int MEASUREMENT_SIZE = 6;  // range, bearing, velocity, alt, heading, confidence
    
    using StateVector = Eigen::Matrix<double, STATE_SIZE, 1>;
    using CovarianceMatrix = Eigen::Matrix<double, STATE_SIZE, STATE_SIZE>;
    using MeasurementVector = Eigen::Matrix<double, MEASUREMENT_SIZE, 1>;
    using MeasurementCovarianceMatrix = Eigen::Matrix<double, MEASUREMENT_SIZE, MEASUREMENT_SIZE>;
    using JacobianMatrix = Eigen::Matrix<double, MEASUREMENT_SIZE, STATE_SIZE>;
    
    struct Config {
        double process_noise_pos = 0.1;      // m²/s⁴
        double process_noise_vel = 0.05;     // m²/s²
        double process_noise_angle = 0.01;   // rad²/s²
        
        double measurement_noise_pos = 5.0;      // meters
        double measurement_noise_vel = 0.5;      // m/s
        double measurement_noise_angle = 0.017;  // radians (~1 degree)
    };
    
    ExtendedKalmanFilter(const Config& config = Config());
    
    // Initialize with first measurement
    void initialize(const MeasurementVector& z, double timestamp);
    
    // Predict step
    void predict(double dt);
    
    // Update step
    void update(const MeasurementVector& z, const MeasurementCovarianceMatrix& R);
    
    // Get current state estimate
    const StateVector& getState() const { return x_; }
    
    // Get covariance (uncertainty)
    const CovarianceMatrix& getCovariance() const { return P_; }
    
    // Mahalanobis distance for data association
    double mahalanobisDistance(const MeasurementVector& z, const MeasurementCovarianceMatrix& R);
    
    // Adaptive noise covariance (learns from residuals)
    void adaptNoiseCovariance(const Eigen::VectorXd& residuals);
    
    // Get predicted measurement (for gating)
    MeasurementVector predictMeasurement() const;
    
    // Getters for individual state components
    double getLatitude() const { return x_(0); }
    double getLongitude() const { return x_(1); }
    double getAltitude() const { return x_(2); }
    double getVelocityNorth() const { return x_(3); }
    double getVelocityEast() const { return x_(4); }
    double getVelocityVertical() const { return x_(5); }
    double getHeading() const { return x_(6); }
    double getPitch() const { return x_(7); }
    double getRoll() const { return x_(8); }
    
    double getSpeed() const {
        return std::sqrt(x_(3)*x_(3) + x_(4)*x_(4) + x_(5)*x_(5));
    }
    
    // Position uncertainty (ellipse semi-major axis)
    double getPositionUncertainty() const {
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es(
            P_.block<2,2>(0,0)
        );
        return std::sqrt(es.eigenvalues().maxCoeff());
    }
    
    double getVelocityUncertainty() const {
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(
            P_.block<3,3>(3,3)
        );
        return std::sqrt(es.eigenvalues().maxCoeff());
    }
    
private:
    // State: [lat, lon, alt, vx, vy, vz, heading, pitch, roll]
    StateVector x_;
    CovarianceMatrix P_;  // Estimate error covariance
    
    // Process noise covariance
    CovarianceMatrix Q_;
    
    // Measurement noise covariance (updated adaptively)
    MeasurementCovarianceMatrix R_;
    
    // Config
    Config config_;
    
    // Linearization
    JacobianMatrix computeH() const;  // Measurement Jacobian
    Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> computeF(double dt) const;  // State Jacobian
    
    // Helper: nonlinear state transition
    StateVector stateTransition(const StateVector& x, double dt) const;
    
    // Helper: nonlinear measurement function
    MeasurementVector measurementFunction(const StateVector& x) const;
    
    // Circular mean for angles (heading)
    double circularMean(double a1, double a2, double w1, double w2) const;
    
    // Adaptive covariance tracking
    std::vector<double> recent_residuals_;
    static constexpr int RESIDUAL_WINDOW = 50;
};
// Unscented Kalman Filter (UKF) for higher nonlinearity
class UnscentedKalmanFilter {
public:
    static constexpr int STATE_SIZE = 9;
    static constexpr int MEASUREMENT_SIZE = 6;
    
    using StateVector = Eigen::Matrix<double, STATE_SIZE, 1>;
    using CovarianceMatrix = Eigen::Matrix<double, STATE_SIZE, STATE_SIZE>;
    using MeasurementVector = Eigen::Matrix<double, MEASUREMENT_SIZE, 1>;
    using MeasurementCovarianceMatrix = Eigen::Matrix<double, MEASUREMENT_SIZE, MEASUREMENT_SIZE>;
    
    struct Config {
        double alpha = 1e-3;      // Spread of sigma points
        double beta = 2.0;        // 2.0 for Gaussian
        double kappa = 0.0;       // Secondary scaling
        
        double process_noise_pos = 0.1;
        double process_noise_vel = 0.05;
        double process_noise_angle = 0.01;
        
        double measurement_noise_pos = 5.0;
        double measurement_noise_vel = 0.5;
        double measurement_noise_angle = 0.017;
    };
    
    UnscentedKalmanFilter(const Config& config = Config());
    
    void initialize(const MeasurementVector& z, double timestamp);
    void predict(double dt);
    void update(const MeasurementVector& z, const MeasurementCovarianceMatrix& R);
    
    const StateVector& getState() const { return x_; }
    const CovarianceMatrix& getCovariance() const { return P_; }
    
    double mahalanobisDistance(const MeasurementVector& z, const MeasurementCovarianceMatrix& R);
    MeasurementVector predictMeasurement() const;
    
    // Getters (same as EKF)
    double getLatitude() const { return x_(0); }
    double getLongitude() const { return x_(1); }
    double getAltitude() const { return x_(2); }
    double getVelocityNorth() const { return x_(3); }
    double getVelocityEast() const { return x_(4); }
    double getVelocityVertical() const { return x_(5); }
    double getHeading() const { return x_(6); }
    double getSpeed() const {
        return std::sqrt(x_(3)*x_(3) + x_(4)*x_(4) + x_(5)*x_(5));
    }
    double getPositionUncertainty() const;
    
private:
    StateVector x_;
    CovarianceMatrix P_;
    CovarianceMatrix Q_;
    MeasurementCovarianceMatrix R_;
    Config config_;
    
    // Sigma point generation
    std::vector<StateVector> generateSigmaPoints(const StateVector& x, const CovarianceMatrix& P);
    
    // Weighted mean
    StateVector weightedMean(const std::vector<StateVector>& sigma_points);
    MeasurementVector weightedMeasurementMean(const std::vector<MeasurementVector>& sigma_measurements);
    
    // Weights
    std::vector<double> w_mean_;
    std::vector<double> w_cov_;
    
    // Nonlinear functions (same as EKF)
    StateVector stateTransition(const StateVector& x, double dt) const;
    MeasurementVector measurementFunction(const StateVector& x) const;
    
    double lambda_;  // Scaling parameter
};
}  // namespace Polybolos::Fusion
