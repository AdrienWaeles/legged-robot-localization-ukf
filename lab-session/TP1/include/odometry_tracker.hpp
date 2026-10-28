#ifndef ODOMETRY_TRACKER_HPP
#define ODOMETRY_TRACKER_HPP

#include <vector>
#include <Eigen/Dense>
#include <deque>

struct OdometryReading {
    double x, y, theta;
    double timestamp;
    double vx, vy, vyaw; // velocities for process model
};

struct ErrorMeasurement {
    double position_error;
    double angle_error;
    double timestamp;
};

class OdometryTracker {
public:
    OdometryTracker();
    ~OdometryTracker();
    
    // Trajectory tracking
    void addOdometryReading(const OdometryReading& reading);
    void resetTracker();
    
    // Trajectory analysis methods
    ErrorMeasurement calculateLinearError(const OdometryReading& start_pose, 
                                        const OdometryReading& end_pose,
                                        double expected_displacement = 0.0);
    
    ErrorMeasurement calculateRotationalError(const OdometryReading& start_pose,
                                            const OdometryReading& end_pose,
                                            double expected_angle_change = 0.0);
    
    // Get trajectory statistics
    std::vector<OdometryReading> getTrajectory() const;
    double getTotalDistance() const;
    
    // Error analysis
    std::vector<ErrorMeasurement> getErrorHistory() const;
    double getAveragePositionError() const;
    double getAverageAngleError() const;
    Eigen::Vector2d getSystematicPositionBias() const;
    double getSystematicAngleBias() const;
    
    // Drift analysis
    double getPositionDriftRate() const; // m/s
    double getAngleDriftRate() const;    // rad/s

private:
    std::vector<OdometryReading> trajectory_;
    std::deque<ErrorMeasurement> error_history_;
    
    // Error statistics
    double total_distance_;
    double total_time_;
    Eigen::Vector2d cumulative_position_error_;
    double cumulative_angle_error_;
    size_t error_count_;
    
    // Helper methods
    double calculateDistance(const OdometryReading& pose1, const OdometryReading& pose2) const;
    double normalizeAngle(double angle) const;
    void updateErrorStatistics(const ErrorMeasurement& error);
    
    // Constants
    static constexpr size_t MAX_ERROR_HISTORY = 1000;
};

#endif // ODOMETRY_TRACKER_HPP