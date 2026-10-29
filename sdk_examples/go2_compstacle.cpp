#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/common/time/time_tool.hpp>
#include <unitree/idl/ros2/PointStamped_.hpp>
#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#include <unistd.h>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <mutex>
#include <chrono>
#include <thread>
#include <vector>

#define TOPIC_RANGE_INFO "rt/utlidar/range_info"
#define TOPIC_HIGHSTATE "rt/sportmodestate"

using namespace unitree::robot;
using namespace unitree::common;

struct Position {
    double x, y, yaw;
};

// Complementary filter parameters
const double alpha_filter = 0.98;   // weight for the orientation
const double alpha_pos    = 0.5;     // weight for the position
const double g            = 9.81;      // gravity [m/s^2]

// Move() takes a body-frame velocity, so the speed is limited and the world-frame error is rotated
const double MAX_SPEED = 0.3;  // [m/s]

double Clamp(double value, double limit) {
    return std::max(-limit, std::min(limit, value));
}

class RobotController {
public:
    RobotController() : dt(0.01), 
                        f_x(0), f_y(0), f_z(0),
                        f_roll(0), f_pitch(0), f_yaw(0),
                        err_gr(0), err_gp(0), err_gy(0),
                        err_ax(0), err_ay(0), err_az(0),
                        err_vx(0), err_vy(0), err_vz(0)
    {
        sport_client.SetTimeout(10.0f);
        sport_client.Init();
        // Subscribe to the robot high-frequency state
        suber.reset(new ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>(TOPIC_HIGHSTATE));
        suber->InitChannel(std::bind(&RobotController::HighStateHandler, this, std::placeholders::_1), 1);
    }

    // Average 10 samples to estimate the sensor bias
    void CalibrateSensors() {
        double sum_gr = 0, sum_gp = 0, sum_gy = 0;
        double sum_ax = 0, sum_ay = 0, sum_az = 0;
        double sum_vx = 0, sum_vy = 0, sum_vz = 0;
        for (int i = 0; i < 10; i++) {
            unitree_go::msg::dds_::SportModeState_ s;
            {
                std::lock_guard<std::mutex> lock(state_mutex);
                s = state;
            }
            sum_gr += s.imu_state().gyroscope()[0];
            sum_gp += s.imu_state().gyroscope()[1];
            sum_gy += s.imu_state().gyroscope()[2];
            sum_ax += s.imu_state().accelerometer()[0];
            sum_ay += s.imu_state().accelerometer()[1];
            sum_az += s.imu_state().accelerometer()[2];
            sum_vx += s.velocity()[0];
            sum_vy += s.velocity()[1];
            sum_vz += s.velocity()[2];
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        std::lock_guard<std::mutex> lock(state_mutex);
        err_gr = sum_gr / 10.0;
        err_gp = sum_gp / 10.0;
        err_gy = sum_gy / 10.0;
        err_ax = sum_ax / 10.0;
        err_ay = sum_ay / 10.0;
        err_az = sum_az / 10.0;
        err_vx = sum_vx / 10.0;
        err_vy = sum_vy / 10.0;
        err_vz = sum_vz / 10.0;
    }

    // Save the initial (filtered) position
    void SaveInitialPosition() {
        std::lock_guard<std::mutex> lock(state_mutex);
        initial_position.x = f_x;
        initial_position.y = f_y;
        initial_position.yaw = f_yaw;
        std::cout << "Initial Position: x = " << initial_position.x 
                  << ", y = " << initial_position.y 
                  << ", yaw = " << initial_position.yaw << "\n";
    }

    // Complementary filter update for the orientation
    void updateOrientation(double gr, double gp, double a_roll, double a_pitch) {
        double roll_gyro  = f_roll + gr * dt;
        double pitch_gyro = f_pitch + gp * dt;
        double roll_acc   = atan2(a_pitch, sqrt(a_roll * a_roll + g * g));
        double pitch_acc  = atan2(-a_roll, g);
        f_roll  = alpha_filter * roll_gyro + (1 - alpha_filter) * roll_acc;
        f_pitch = alpha_filter * pitch_gyro + (1 - alpha_filter) * pitch_acc;
        // Yaw is taken directly from the IMU
        f_yaw = state.imu_state().rpy()[2];
    }

    // Complementary filter update for the position
    void updateTranslation(double vx, double vy, double vz, 
                           double ax, double ay, double az) {
        double x_vel = f_x + vx * dt;
        double y_vel = f_y + vy * dt;
        double z_vel = f_z + vz * dt;
        double x_acc = f_x + vx * dt + 0.5 * ax * dt * dt;
        double y_acc = f_y + vy * dt + 0.5 * ay * dt * dt;
        double z_acc = f_z + vz * dt + 0.5 * az * dt * dt;
        f_x = alpha_pos * x_vel + (1 - alpha_pos) * x_acc;
        f_y = alpha_pos * y_vel + (1 - alpha_pos) * y_acc;
        f_z = alpha_pos * z_vel + (1 - alpha_pos) * z_acc;
    }

    // Callback receiving the robot state and updating the filter
    void HighStateHandler(const void *message) {
        std::lock_guard<std::mutex> lock(state_mutex);
        state = *(unitree_go::msg::dds_::SportModeState_ *)message;
        // Remove the estimated bias from the measurements
        double gr = state.imu_state().gyroscope()[0] - err_gr;
        double gp = state.imu_state().gyroscope()[1] - err_gp;
        double a_roll  = state.imu_state().accelerometer()[0] - err_ax;
        double a_pitch = state.imu_state().accelerometer()[1] - err_ay;
        double vx = state.velocity()[0] - err_vx;
        double vy = state.velocity()[1] - err_vy;
        double vz = state.velocity()[2] - err_vz;
        double ax = state.imu_state().accelerometer()[0] - err_ax;
        double ay = state.imu_state().accelerometer()[1] - err_ay;
        double az = state.imu_state().accelerometer()[2] - err_az;
        updateOrientation(gr, gp, a_roll, a_pitch);
        updateTranslation(vx, vy, vz, ax, ay, az);
    }

    // Obstacle avoidance from the lidar ranges (same logic as go2_obstacle.cpp)
    void AvoidObstacle(const geometry_msgs::msg::dds_::PointStamped_ *range_msg) {
        if (range_msg->point().x() < 0.5) {
            std::cout << "Obstacle ahead: " << range_msg->point().x() << "\n";
            sport_client.Move(-0.3, 0, 0);
        }
        if (range_msg->point().y() < 0.1) {
            std::cout << "Obstacle on the left: " << range_msg->point().y() << "\n";
            sport_client.Move(0, -0.3, 0);
        }
        if (range_msg->point().z() < 0.1) {
            std::cout << "Obstacle on the right: " << range_msg->point().z() << "\n";
            sport_client.Move(0, 0.3, 0);
        }
    }

    // Return to the initial position using the filtered values
    void ReturnToInitialPosition() {
        double dx, dy, yaw;
        {
            std::lock_guard<std::mutex> lock(state_mutex);
            dx = initial_position.x - f_x;
            dy = initial_position.y - f_y;
            yaw = f_yaw;
        }
        double bx = std::cos(yaw) * dx + std::sin(yaw) * dy;
        double by = -std::sin(yaw) * dx + std::cos(yaw) * dy;
        sport_client.Move(Clamp(bx, MAX_SPEED), Clamp(by, MAX_SPEED), 0);
        // TODO: correct the heading as well
    }

    // Membres
    unitree::robot::go2::SportClient sport_client;
    unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::SportModeState_> suber;
    unitree_go::msg::dds_::SportModeState_ state;
    Position initial_position;
    std::mutex state_mutex;  // protects state and the filter variables, written by the DDS thread
    
    // Complementary filter variables
    double dt;
    double f_x, f_y, f_z;
    double f_roll, f_pitch, f_yaw;
    
    // Calibration biases
    double err_gr, err_gp, err_gy;
    double err_ax, err_ay, err_az;
    double err_vx, err_vy, err_vz;
};

// Callback handling the lidar messages
void Handler(const void *message, RobotController &controller) {
    const geometry_msgs::msg::dds_::PointStamped_ *range_msg = 
            (const geometry_msgs::msg::dds_::PointStamped_ *)message;
    std::cout << "Received range info: "
              << "\n\tstamp = " << range_msg->header().stamp().sec() << "." << range_msg->header().stamp().nanosec()
              << "\n\tframe = " << range_msg->header().frame_id()
              << "\n\trange front = " << range_msg->point().x()
              << "\n\trange left  = " << range_msg->point().y()
              << "\n\trange right = " << range_msg->point().z()
              << "\n" << std::endl;
    controller.AvoidObstacle(range_msg);
}

int main(int argc, const char **argv) {
    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " networkInterface" << std::endl;
        exit(-1);
    }
    unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);
    
    RobotController controller;
    sleep(1); // Wait for a stable state
    
    controller.CalibrateSensors();     // Sensor calibration
    controller.SaveInitialPosition();    // Save the filtered initial position

    // Subscribe to the range messages (lidar)
    ChannelSubscriber<geometry_msgs::msg::dds_::PointStamped_> subscriber(TOPIC_RANGE_INFO);
    subscriber.InitChannel([&](const void *msg) { Handler(msg, controller); });
    
    // Main loop: the robot periodically returns to its initial position
    while (true) {
        controller.ReturnToInitialPosition();
        sleep(2);
    }
    return 0;
}

