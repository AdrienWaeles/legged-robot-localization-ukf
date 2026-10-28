#include "ukf_filter.hpp"
#include <cmath>
#include <iostream>

UKFFilter::UKFFilter() : initialized_(false), innovation_sum_(0.0), 
                        innovation_count_(0), last_innovation_(0.0) {
    // Use the STATE_DIM constant
    state_dim_ = STATE_DIM; // 6 dimensions
    
    // Initialize UKF parameters
    alpha_ = 0.001;    // Spread of sigma points
    beta_ = 2.0;       // Prior knowledge parameter
    kappa_ = 0.0;      // Secondary scaling parameter
    
    // Calculate derived parameters
    lambda_ = alpha_ * alpha_ * (state_dim_ + kappa_) - state_dim_;
    
    // Initialize weights
    calculateWeights();
    
    // Initialize state vector (6x1: x, y, theta, vx, vy, vyaw)
    state_ = Eigen::VectorXd::Zero(state_dim_);
    
    // Initialize covariance matrix (6x6)
    covariance_ = Eigen::MatrixXd::Identity(state_dim_, state_dim_) * 0.1;
    
    // Initialize process noise (6x6)
    process_noise_ = Eigen::MatrixXd::Identity(state_dim_, state_dim_);
    process_noise_.diagonal() << 0.01, 0.01, 0.01, 0.1, 0.1, 0.1; // pos, vel noise
    
    // Initialize measurement noise (3x3) - seulement position et orientation
    measurement_noise_ = Eigen::MatrixXd::Identity(3, 3) * 0.1;
}

UKFFilter::~UKFFilter() {
    // Cleanup if needed
}

// Interface qui correspond au main.cpp
void UKFFilter::initialize(const Eigen::VectorXd& initial_state, 
                          const Eigen::MatrixXd& initial_covariance) {
    if (initial_state.size() != state_dim_) {
        std::cerr << "Error: Initial state size mismatch. Expected " 
                  << state_dim_ << ", got " << initial_state.size() << std::endl;
        return;
    }
    
    if (initial_covariance.rows() != state_dim_ || initial_covariance.cols() != state_dim_) {
        std::cerr << "Error: Initial covariance size mismatch. Expected " 
                  << state_dim_ << "x" << state_dim_ << std::endl;
        return;
    }
    
    state_ = initial_state;
    covariance_ = initial_covariance;
    initialized_ = true;
    
    std::cout << "UKF initialized with state: [" 
              << state_(0) << ", " << state_(1) << ", " << state_(2) 
              << ", " << state_(3) << ", " << state_(4) << ", " << state_(5) << "]" << std::endl;
}

void UKFFilter::predict(double dt, const Eigen::VectorXd& control_input) {
    if (!initialized_) {
        std::cerr << "UKF not initialized!" << std::endl;
        return;
    }
    
    // Generate sigma points
    Eigen::MatrixXd sigma_points = generateSigmaPoints();
    
    // Predict each sigma point through motion model
    int num_sigma_points = 2 * state_dim_ + 1;
    Eigen::MatrixXd predicted_sigma_points(state_dim_, num_sigma_points);
    
    // Extract control inputs (linear and angular velocity)
    double linear_vel = (control_input.size() >= 1) ? control_input(0) : 0.0;
    double angular_vel = (control_input.size() >= 2) ? control_input(1) : 0.0;
    
    for (int i = 0; i < num_sigma_points; ++i) {
        predicted_sigma_points.col(i) = motionModel(sigma_points.col(i), dt, linear_vel, angular_vel);
    }
    
    // Calculate predicted mean and covariance
    calculateMeanAndCovariance(predicted_sigma_points);
    
    // Add process noise
    covariance_ += process_noise_;
}

void UKFFilter::update(const Eigen::VectorXd& measurement) {
    if (!initialized_) {
        std::cerr << "UKF not initialized!" << std::endl;
        return;
    }
    
    if (measurement.size() != 3) {
        std::cerr << "Error: Measurement size should be 3 (x, y, theta)" << std::endl;
        return;
    }
    
    // Generate sigma points from current state
    Eigen::MatrixXd sigma_points = generateSigmaPoints();
    
    // Transform sigma points through measurement model
    int num_sigma_points = 2 * state_dim_ + 1;
    Eigen::MatrixXd predicted_measurements(3, num_sigma_points);
    
    for (int i = 0; i < num_sigma_points; ++i) {
        predicted_measurements.col(i) = measurementModel(sigma_points.col(i));
    }
    
    // Calculate predicted measurement mean
    Eigen::VectorXd predicted_mean = Eigen::VectorXd::Zero(3);
    for (int i = 0; i < num_sigma_points; ++i) {
        predicted_mean += w_mean_[i] * predicted_measurements.col(i);
    }
    
    // Calculate measurement covariance
    Eigen::MatrixXd S = Eigen::MatrixXd::Zero(3, 3);
    for (int i = 0; i < num_sigma_points; ++i) {
        Eigen::VectorXd diff = predicted_measurements.col(i) - predicted_mean;
        diff(2) = normalizeAngle(diff(2)); // Normalize angle difference
        S += w_cov_[i] * diff * diff.transpose();
    }
    S += measurement_noise_;
    
    // Calculate cross-covariance
    Eigen::MatrixXd cross_covariance = Eigen::MatrixXd::Zero(state_dim_, 3);
    for (int i = 0; i < num_sigma_points; ++i) {
        Eigen::VectorXd state_diff = sigma_points.col(i) - state_;
        Eigen::VectorXd meas_diff = predicted_measurements.col(i) - predicted_mean;
        state_diff(2) = normalizeAngle(state_diff(2));
        meas_diff(2) = normalizeAngle(meas_diff(2));
        cross_covariance += w_cov_[i] * state_diff * meas_diff.transpose();
    }
    
    // Calculate Kalman gain
    Eigen::MatrixXd K = cross_covariance * S.inverse();
    
    // Calculate innovation
    Eigen::VectorXd innovation = measurement - predicted_mean;
    innovation(2) = normalizeAngle(innovation(2));
    
    // Update state and covariance
    state_ = state_ + K * innovation;
    state_(2) = normalizeAngle(state_(2));
    covariance_ = covariance_ - K * S * K.transpose();
    
    // Store innovation for statistics
    last_innovation_ = innovation.norm();
    innovation_sum_ += last_innovation_;
    innovation_count_++;
}

Eigen::VectorXd UKFFilter::getState() const {
    return state_;
}

Eigen::MatrixXd UKFFilter::getCovariance() const {
    return covariance_;
}

double UKFFilter::getInnovation() const {
    return last_innovation_;
}

void UKFFilter::resetStatistics() {
    innovation_sum_ = 0.0;
    innovation_count_ = 0;
}

double UKFFilter::getAverageInnovation() const {
    return (innovation_count_ > 0) ? (innovation_sum_ / innovation_count_) : 0.0;
}

void UKFFilter::calculateWeights() {
    int num_sigma_points = 2 * state_dim_ + 1;
    w_mean_.resize(num_sigma_points);
    w_cov_.resize(num_sigma_points);
    
    // Calculate weights for mean
    w_mean_[0] = lambda_ / (state_dim_ + lambda_);
    for (int i = 1; i < num_sigma_points; ++i) {
        w_mean_[i] = 1.0 / (2.0 * (state_dim_ + lambda_));
    }
    
    // Calculate weights for covariance
    w_cov_[0] = w_mean_[0] + (1.0 - alpha_ * alpha_ + beta_);
    for (int i = 1; i < num_sigma_points; ++i) {
        w_cov_[i] = w_mean_[i];
    }
}

Eigen::MatrixXd UKFFilter::generateSigmaPoints() {
    int num_sigma_points = 2 * state_dim_ + 1;
    Eigen::MatrixXd sigma_points(state_dim_, num_sigma_points);
    
    // Calculate matrix square root
    Eigen::MatrixXd sqrt_matrix = ((state_dim_ + lambda_) * covariance_).llt().matrixL();
    
    // Generate sigma points
    sigma_points.col(0) = state_;
    
    for (int i = 0; i < state_dim_; ++i) {
        sigma_points.col(i + 1) = state_ + sqrt_matrix.col(i);
        sigma_points.col(i + 1 + state_dim_) = state_ - sqrt_matrix.col(i);
    }
    
    return sigma_points;
}

Eigen::VectorXd UKFFilter::motionModel(const Eigen::VectorXd& state, double dt, 
                                      double linear_vel, double angular_vel) {
    Eigen::VectorXd new_state(state_dim_);
    
    double x = state(0);
    double y = state(1);
    double theta = state(2);
    double vx = state(3);
    double vy = state(4);
    double vyaw = state(5);
    
    // Motion model with velocity state
    new_state(0) = x + vx * dt;  // x position
    new_state(1) = y + vy * dt;  // y position
    new_state(2) = normalizeAngle(theta + vyaw * dt);  // orientation
    new_state(3) = linear_vel * cos(theta);  // vx from control
    new_state(4) = linear_vel * sin(theta);  // vy from control
    new_state(5) = angular_vel;  // vyaw from control
    
    return new_state;
}

Eigen::VectorXd UKFFilter::measurementModel(const Eigen::VectorXd& state) {
    // Measurement model: observe position and orientation
    Eigen::VectorXd measurement(3);
    measurement(0) = state(0);  // x
    measurement(1) = state(1);  // y
    measurement(2) = state(2);  // theta
    return measurement;
}

void UKFFilter::calculateMeanAndCovariance(const Eigen::MatrixXd& sigma_points) {
    int num_sigma_points = sigma_points.cols();
    
    // Calculate weighted mean
    state_ = Eigen::VectorXd::Zero(state_dim_);
    for (int i = 0; i < num_sigma_points; ++i) {
        state_ += w_mean_[i] * sigma_points.col(i);
    }
    state_(2) = normalizeAngle(state_(2));
    
    // Calculate weighted covariance
    covariance_ = Eigen::MatrixXd::Zero(state_dim_, state_dim_);
    for (int i = 0; i < num_sigma_points; ++i) {
        Eigen::VectorXd diff = sigma_points.col(i) - state_;
        diff(2) = normalizeAngle(diff(2));
        covariance_ += w_cov_[i] * diff * diff.transpose();
    }
}

double UKFFilter::normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}