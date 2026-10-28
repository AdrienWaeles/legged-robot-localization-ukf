#ifndef UNITREE_CONTROLLER_HPP
#define UNITREE_CONTROLLER_HPP

#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unitree/robot/go2/robot_state/robot_state_client.hpp>
#include <thread>
#include <atomic>
#include <memory>
#include <mutex>
#include <chrono>
#include <cmath>
#include <iostream>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

struct RobotPose {
    double x, y, theta;
    double timestamp;
};

struct VelocityCommand {
    double vx, vy, vyaw;
};

class UnitreeController {
public:
    UnitreeController();
    ~UnitreeController();
    
    // Initialization and connection
    bool initialize(const std::string& robot_ip = "192.168.123.161");
    void shutdown();
    
    // Movement commands
    void sendVelocityCommand(const VelocityCommand& cmd);
    void stop();
    void standUp();
    void standDown();
    
    // State reading
    RobotPose getCurrentPose() const;
    bool isConnected() const;
    
    // Movement primitives for the TP
    void moveForward(double distance, double speed = 0.3);
    void moveBackward(double distance, double speed = 0.3);
    void rotateInPlace(double angle_rad, double angular_speed = 0.5);
    
    // Wait for movement completion
    bool waitForMovementCompletion(double timeout_seconds = 10.0);
    void debugSportClient();  // Add this line
    void enterSportMode();  // Add this line

private:
    // Unitree SDK objects
    std::unique_ptr<unitree::robot::go2::RobotStateClient> state_client_;
    std::unique_ptr<unitree::robot::go2::SportClient> sport_client_;
    
    // State management
    std::atomic<bool> is_connected_;
    std::atomic<bool> movement_completed_;
    std::atomic<bool> should_stop_thread_;
    
    // Robot state
    mutable std::mutex pose_mutex_;
    RobotPose current_pose_;
    RobotPose start_pose_;
    
    // Movement tracking
    double target_distance_;
    double target_angle_;
    double current_speed_;        // Add this line
    double current_angular_speed_; // Add this line
    std::chrono::steady_clock::time_point movement_start_time_;
    
    // Communication thread
    std::thread state_thread_;
    
    // Private methods
    void stateUpdateLoop();
    double calculateDistance(const RobotPose& pose1, const RobotPose& pose2) const;
    double normalizeAngle(double angle) const;
    
    // Constants
    static constexpr double POSITION_TOLERANCE = 0.05; // 5cm
    static constexpr double ANGLE_TOLERANCE = 0.05;    // ~3 degrees
};

#endif // UNITREE_CONTROLLER_HPP