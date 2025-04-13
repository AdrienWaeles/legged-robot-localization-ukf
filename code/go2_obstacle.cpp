#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/common/time/time_tool.hpp>
#include <unitree/idl/ros2/PointStamped_.hpp>
#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unistd.h>
#include <unitree/idl/go2/SportModeState_.hpp>

#define TOPIC_RANGE_INFO "rt/utlidar/range_info"
#define TOPIC_HIGHSTATE "rt/sportmodestate"

using namespace unitree::robot;
using namespace unitree::common;

struct Position {
    double x, y, yaw;
};

class RobotController {
public:
    RobotController() {
        sport_client.SetTimeout(10.0f);
        sport_client.Init();
        suber.reset(new ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>(TOPIC_HIGHSTATE));
        suber->InitChannel(std::bind(&RobotController::HighStateHandler, this, std::placeholders::_1), 1);
    }

    void SaveInitialPosition() {
        initial_position.x = state.position()[0];
        initial_position.y = state.position()[1];
        initial_position.yaw = state.imu_state().rpy()[2];
        std::cout << "Initial Position: x = " << initial_position.x << ", y = " << initial_position.y << "\n";
    }

    void AvoidObstacle(const geometry_msgs::msg::dds_::PointStamped_ *range_msg) {
     	if (range_msg->point().x()<0.5){
	     std::cout << "AAAAAAAAAAAAAAAAAA\n";
	     sport_client.Move(-0.5, 0, 0);
	     std::cout <<range_msg->point().x() <<"\n";
    	}
	if (range_msg->point().y()<0.1){
	     std::cout << "AAAAAAAAAAAAAAAAAA\n";
	     sport_client.Move(0, -0.3, 0);
	     std::cout <<range_msg->point().y() <<"\n";
	}
	 if (range_msg->point().z()<0.1){
	     std::cout << "AAAAAAAAAAAAAAAAAA\n";
	     sport_client.Move(0, 0.3, 0);
	     std::cout <<range_msg->point().z() <<"\n";
   	}
    }

    void ReturnToInitialPosition() {
    
        double dx = initial_position.x - state.position()[0];
        double dy = initial_position.y - state.position()[1];
        sport_client.Move(dx, dy, 0);
        
        
        double dyaw = initial_position.yaw - state.imu_state().rpy()[2];
        //rotation:
        sport_client.Euler(0,0,dyaw);
    }

    void HighStateHandler(const void *message) {
        state = *(unitree_go::msg::dds_::SportModeState_ *)message;
    }

    unitree::robot::go2::SportClient sport_client;
    unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::SportModeState_> suber;
    unitree_go::msg::dds_::SportModeState_ state;
    Position initial_position;
};

void Handler(const void *message, RobotController &controller) {
    const geometry_msgs::msg::dds_::PointStamped_ *range_msg = (const geometry_msgs::msg::dds_::PointStamped_ *)message;
      // print msg info
    std::cout << "Received a range info message here!"
            << "\n\tstamp = " << range_msg->header().stamp().sec() << "." << range_msg->header().stamp().nanosec() 
            << "\n\tframe = " << range_msg->header().frame_id()
            << "\n\trange front = " << range_msg->point().x()
            << "\n\trange left = " << range_msg->point().y()
            << "\n\trange right = " << range_msg->point().z()
            << std::endl << std::endl;
    controller.AvoidObstacle(range_msg);
}

int main(int argc, const char **argv) {
    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " networkInterface" << std::endl;
        exit(-1);
    }
    unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);
    RobotController controller;
    sleep(1);
    controller.SaveInitialPosition();
    ChannelSubscriber<geometry_msgs::msg::dds_::PointStamped_> subscriber(TOPIC_RANGE_INFO);
    subscriber.InitChannel([&](const void *msg) { Handler(msg, controller); });
    while (true) {
    	controller.ReturnToInitialPosition();
        sleep(2);
    }
    return 0;
}

