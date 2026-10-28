#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/common/time/time_tool.hpp>
#include <unitree/idl/ros2/PointStamped_.hpp>
#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#include <unistd.h>
#include <cmath>
#include <iostream>
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

// Paramètres pour le filtre complémentaire
const double alpha_filter = 0.98;   // pondération pour l'orientation
const double alpha_pos    = 0.5;     // pondération pour la position
const double g            = 9.81;      // gravité (m/s²)

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
        // Souscription à l'état haute fréquence du robot
        suber.reset(new ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>(TOPIC_HIGHSTATE));
        suber->InitChannel(std::bind(&RobotController::HighStateHandler, this, std::placeholders::_1), 1);
    }

    // Calibration sur 10 échantillons pour estimer le biais des capteurs
    void CalibrateSensors() {
        double sum_gr = 0, sum_gp = 0, sum_gy = 0;
        double sum_ax = 0, sum_ay = 0, sum_az = 0;
        double sum_vx = 0, sum_vy = 0, sum_vz = 0;
        for (int i = 0; i < 10; i++) {
            sum_gr += state.imu_state().gyroscope()[0];
            sum_gp += state.imu_state().gyroscope()[1];
            sum_gy += state.imu_state().gyroscope()[2];
            sum_ax += state.imu_state().accelerometer()[0];
            sum_ay += state.imu_state().accelerometer()[1];
            sum_az += state.imu_state().accelerometer()[2];
            sum_vx += state.velocity()[0];
            sum_vy += state.velocity()[1];
            sum_vz += state.velocity()[2];
            sleep(0.1);  // 100 ms
        }
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

    // sauvegarde de la position initiale (filtrée)
    void SaveInitialPosition() {
        initial_position.x = f_x;
        initial_position.y = f_y;
        initial_position.yaw = f_yaw;
        std::cout << "Initial Position: x = " << initial_position.x 
                  << ", y = " << initial_position.y 
                  << ", yaw = " << initial_position.yaw << "\n";
    }

    // Mise a jour du filtre complementaire pour l'orientation
    void updateOrientation(double gr, double gp, double a_roll, double a_pitch) {
        double roll_gyro  = f_roll + gr * dt;
        double pitch_gyro = f_pitch + gp * dt;
        double roll_acc   = atan2(a_pitch, sqrt(a_roll * a_roll + g * g)) * 180.0 / M_PI;
        double pitch_acc  = atan2(-a_roll, g) * 180.0 / M_PI;
        f_roll  = alpha_filter * roll_gyro + (1 - alpha_filter) * roll_acc;
        f_pitch = alpha_filter * pitch_gyro + (1 - alpha_filter) * pitch_acc;
        // Pour le yaw, on se fie directement à l'IMU
        f_yaw = state.imu_state().rpy()[2];
    }

    // Mise a jour du filtre complémentaire pour la position
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

    // Callback recevant l'état du robot et mettant a jour le filtre
    void HighStateHandler(const void *message) {
        state = *(unitree_go::msg::dds_::SportModeState_ *)message;
        // Correction des mesures avec le biais estimé
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

    // Méthode d'évitement d'obstacles via le lidar (meme logique que dans ton premier code)
    void AvoidObstacle(const geometry_msgs::msg::dds_::PointStamped_ *range_msg) {
        if (range_msg->point().x() < 0.5) {
            std::cout << "Obstacle devant: " << range_msg->point().x() << "\n";
            sport_client.Move(-0.3, 0, 0);
        }
        if (range_msg->point().y() < 0.1) {
            std::cout << "Obstacle gauche: " << range_msg->point().y() << "\n";
            sport_client.Move(0, -0.3, 0);
        }
        if (range_msg->point().z() < 0.1) {
            std::cout << "Obstacle droit: " << range_msg->point().z() << "\n";
            sport_client.Move(0, 0.3, 0);
        }
    }

    // Retour à la position initiale en utilisant les valeurs filtrées
    void ReturnToInitialPosition() {
        double dx = initial_position.x - f_x;
        double dy = initial_position.y - f_y;
        sport_client.Move(dx/2, dy/2, 0); //on divise en deux fois, car Move va trop vite sur des grandes distances
        sleep(0.5);
        sport_client.Move(dx/2, dy/2, 0);
        sleep(0.5);
        /* à améliorer puis décommenter
        double dyaw = initial_position.yaw - f_yaw;
        sport_client.Move(0,0,dyaw); //le troisième paramètre de Move est le lacet (yaw)
        */
        
    }

    // Membres
    unitree::robot::go2::SportClient sport_client;
    unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::SportModeState_> suber;
    unitree_go::msg::dds_::SportModeState_ state;
    Position initial_position;
    
    // Variables du filtre complémentaire
    double dt;
    double f_x, f_y, f_z;
    double f_roll, f_pitch, f_yaw;
    
    // Variables de calibration
    double err_gr, err_gp, err_gy;
    double err_ax, err_ay, err_az;
    double err_vx, err_vy, err_vz;
};

// Callback pour traiter les messages du lidar
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
    sleep(1); // Attendre un peu pour obtenir un état stable
    
    controller.CalibrateSensors();     // Calibration des capteurs
    controller.SaveInitialPosition();    // Sauvegarde de la position initiale filtrée

    // Souscription aux messages de distance (lidar)
    ChannelSubscriber<geometry_msgs::msg::dds_::PointStamped_> subscriber(TOPIC_RANGE_INFO);
    subscriber.InitChannel([&](const void *msg) { Handler(msg, controller); });
    
    // Boucle principale : le robot retourne périodiquement à sa position initiale
    while (true) {
        controller.ReturnToInitialPosition();
        sleep(2);
    }
    return 0;
}

