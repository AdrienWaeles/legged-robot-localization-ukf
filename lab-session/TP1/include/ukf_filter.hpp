#ifndef UKF_FILTER_HPP
#define UKF_FILTER_HPP

#include <Eigen/Dense>
#include <vector>

// State vector: [x, y, theta, vx, vy, vyaw]
constexpr int STATE_DIM = 6;
constexpr int MEASUREMENT_DIM = 3; // [x, y, theta] from odometry
constexpr int AUGMENTED_DIM = STATE_DIM + STATE_DIM; // state + process noise

class UKFFilter {
public:
    UKFFilter();
    ~UKFFilter();
    
    // HARMONISÉ - Interface qui correspond au main.cpp
    void initialize(const Eigen::VectorXd& initial_state, 
                   const Eigen::MatrixXd& initial_covariance);
    
    // Main UKF functions
    void predict(double dt, const Eigen::VectorXd& control_input = Eigen::VectorXd::Zero(2));
    void update(const Eigen::VectorXd& measurement);
    
    // Getters
    Eigen::VectorXd getState() const;
    Eigen::MatrixXd getCovariance() const;
    double getInnovation() const;
    
    // Statistics
    void resetStatistics();
    double getAverageInnovation() const;

private:
    // AJOUTÉ - Membres manquants
    bool initialized_;
    int state_dim_;
    
    // State and covariance
    Eigen::VectorXd state_;
    Eigen::MatrixXd covariance_;
    
    // Noise matrices
    Eigen::MatrixXd process_noise_;
    Eigen::MatrixXd measurement_noise_;
    
    // UKF parameters
    double alpha_;  // spread of sigma points (typically 0.001)
    double beta_;   // prior knowledge parameter (2 for Gaussian)
    double kappa_;  // secondary scaling parameter (0 or 3-n)
    double lambda_; // composite scaling parameter
    
    // AJOUTÉ - Weights for sigma points
    Eigen::VectorXd w_mean_;
    Eigen::VectorXd w_cov_;
    
    // Innovation statistics
    double innovation_sum_;
    int innovation_count_;
    double last_innovation_;
    
    // Sigma point methods
    Eigen::MatrixXd generateSigmaPoints();
    void calculateWeights();
    void calculateMeanAndCovariance(const Eigen::MatrixXd& sigma_points);
    
    // Process and measurement models
    Eigen::VectorXd motionModel(const Eigen::VectorXd& state, double dt, 
                               double linear_vel = 0.0, double angular_vel = 0.0);
    Eigen::VectorXd measurementModel(const Eigen::VectorXd& state);
    
    // Utility functions
    double normalizeAngle(double angle);
};

#endif // UKF_FILTER_HPP