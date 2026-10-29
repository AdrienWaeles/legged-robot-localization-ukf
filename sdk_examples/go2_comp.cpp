#include <chrono>
#include <cmath>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <mutex>
#include <thread>

#include <unistd.h>

#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>

#define TOPIC_HIGHSTATE "rt/sportmodestate"

// Keeps the latest robot state received from the SDK
class StateListener
{
public:
    StateListener()
    {
        suber.reset(new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>(TOPIC_HIGHSTATE));
        suber->InitChannel(std::bind(&StateListener::HighStateHandler, this, std::placeholders::_1), 1);
    }

    // The callback runs in a DDS thread, so the state is copied under a lock
    unitree_go::msg::dds_::SportModeState_ Get()
    {
        std::lock_guard<std::mutex> lock(mutex);
        return state;
    }

private:
    void HighStateHandler(const void *message)
    {
        std::lock_guard<std::mutex> lock(mutex);
        state = *(unitree_go::msg::dds_::SportModeState_ *)message;
    }

    std::mutex mutex;
    unitree_go::msg::dds_::SportModeState_ state;
    unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::SportModeState_> suber;
};

// Complementary filter parameters
const double alpha = 0.98;  // Weight between the data sources
const double g = 9.81;      // Gravity [m/s^2]

// Complementary filter update
void updateOrientation(double &roll, double &pitch, double dt,
                       double gr, double gp, // Gyroscope (rad/s)
                       double aroll, double apitch) // Accelerometer [m/s^2]
{
    // Orientation from the gyroscope (integration)
    double roll_gyro = roll + gr * dt;
    double pitch_gyro = pitch + gp * dt;

    // Orientation from the accelerometer
    double roll_acc = atan2(apitch, sqrt(aroll * aroll + g * g));
    double pitch_acc = atan2(-aroll, g);

    // Data fusion
    roll = alpha * roll_gyro + (1 - alpha) * roll_acc;
    pitch = alpha * pitch_gyro + (1 - alpha) * pitch_acc;
}

const double alpha_pos = 0.5; // Complementary filter weight

void updateTranslation(double &x, double &y, double &z, double dt,
                       double vx, double vy, double vz,
                       double ax, double ay, double az)
{
    // Position from the integrated velocity
    double x_vel = x + vx * dt;
    double y_vel = y + vy * dt;
    double z_vel = z + vz * dt;

    // Position from the double-integrated acceleration
    double x_acc = x + vx * dt + 0.5 * ax * dt * dt;
    double y_acc = y + vy * dt + 0.5 * ay * dt * dt;
    double z_acc = z + vz * dt + 0.5 * az * dt * dt;

    // Complementary filtering
    x = alpha_pos * x_vel + (1 - alpha_pos) * x_acc;
    y = alpha_pos * y_vel + (1 - alpha_pos) * y_acc;
    z = alpha_pos * z_vel + (1 - alpha_pos) * z_acc;
}

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        std::cout << "Usage: " << argv[0] << " networkInterface" << std::endl;
        exit(-1);
    }

    unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);
    StateListener listener;

    sleep(1); // Wait 1 second for a stable state

    // Average 10 samples at rest to estimate the sensor biases
    double errgr = 0, errgp = 0;
    double errax = 0, erray = 0, erraz = 0;
    double errvx = 0, errvy = 0, errvz = 0;
    for (int i = 0; i < 10; i++)
    {
        auto s = listener.Get();
        errgr += s.imu_state().gyroscope()[0];
        errgp += s.imu_state().gyroscope()[1];
        errax += s.imu_state().accelerometer()[0];
        erray += s.imu_state().accelerometer()[1];
        erraz += s.imu_state().accelerometer()[2];
        errvx += s.velocity()[0];
        errvy += s.velocity()[1];
        errvz += s.velocity()[2];
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    errgr /= 10;
    errgp /= 10;
    errax /= 10;
    erray /= 10;
    erraz /= 10;
    errvx /= 10;
    errvy /= 10;
    errvz /= 10;

    double roll = 0.0, pitch = 0.0;
    double dt = 0.05; // loop period [s], matches the sleep below
    auto start = listener.Get();
    double x = start.position()[0];
    double y = start.position()[1];
    double z = start.position()[2];
    std::cout << "initial position: x0: " << x << ", y0: " << y << ", z0: " << z << std::endl;

    while (true)
    {
        auto s = listener.Get();

        // Bias-corrected data
        double gr = s.imu_state().gyroscope()[0] - errgr;
        double gp = s.imu_state().gyroscope()[1] - errgp;
        double vx = s.velocity()[0] - errvx;
        double vy = s.velocity()[1] - errvy;
        double vz = s.velocity()[2] - errvz;
        double ax = s.imu_state().accelerometer()[0] - errax;
        double ay = s.imu_state().accelerometer()[1] - erray;
        double az = s.imu_state().accelerometer()[2] - erraz;

        updateTranslation(x, y, z, dt, vx, vy, vz, ax, ay, az);
        updateOrientation(roll, pitch, dt, gr, gp, ax, ay);

        std::cout << "roll: " << roll << " pitch: " << pitch << std::endl;
        std::cout << "xcomp: " << x << " ycomp: " << y << " zcomp: " << z << std::endl;
        std::cout << "x: " << s.position()[0] << " y: " << s.position()[1] << " z: " << s.position()[2] << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(50)); // Loop period
    }

    return 0;
}
