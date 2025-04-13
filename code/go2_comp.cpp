#include <cmath>

#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>

#define TOPIC_HIGHSTATE "rt/sportmodestate"

using namespace unitree::common;

enum test_mode
{
  /*---Basic motion---*/
  normal_stand,
  balance_stand,
  velocity_move,
  trajectory_follow,
  stand_down,
  stand_up,
  damp,
  recovery_stand,
  /*---Special motion ---*/
  sit,
  rise_sit,
  stretch,
  wallow,
  //content,
  pose,
  scrape,
  front_flip,
  front_jump,
  front_pounce,
  stop_move = 99
};

const int TEST_MODE = trajectory_follow;

class Custom
{
public:
  Custom()
  {
    sport_client.SetTimeout(10.0f);
    sport_client.Init();

    suber.reset(new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>(TOPIC_HIGHSTATE));
    suber->InitChannel(std::bind(&Custom::HighStateHandler, this, std::placeholders::_1), 1);
  };

  void RobotControl()
  {
    ct += dt;
    double px_local, py_local, yaw_local;
    double vx_local, vy_local, vyaw_local;
    double px_err, py_err, yaw_err;
    double time_seg, time_temp;

    unitree::robot::go2::PathPoint path_point_tmp;
    std::vector<unitree::robot::go2::PathPoint> path;

    switch (TEST_MODE)
    {
    case normal_stand:            // 0. idle, default stand
      sport_client.SwitchGait(0); // 0:idle; 1:tort; 2:tort running; 3:climb stair; 4:tort obstacle
      sport_client.StandUp();
      break;

    case balance_stand:                  // 1. Balance stand (controlled by dBodyHeight + rpy)
      sport_client.Euler(0.1, 0.2, 0.3); // roll, pitch, yaw
      sport_client.BodyHeight(0.0);      // relative height [-0.18~0.03]
      sport_client.BalanceStand();
      break;

    case velocity_move: // 2. target velocity walking (controlled by velocity + yawSpeed)
      sport_client.Move(0.3, 0, 0.3);
      break;

    case trajectory_follow: // 3. path mode walking
      time_seg = 0.2;
      time_temp = ct - time_seg;
      for (int i = 0; i < 30; i++)
      {
        time_temp += time_seg;

        px_local = 0.5 * sin(0.5 * time_temp);
        py_local = 0;
        yaw_local = 0;
        vx_local = 0.5 * cos(0.5 * time_temp);
        vy_local = 0;
        vyaw_local = 0;

        path_point_tmp.timeFromStart = i * time_seg;
        path_point_tmp.x = px_local * cos(yaw0) - py_local * sin(yaw0) + px0;
        path_point_tmp.y = px_local * sin(yaw0) + py_local * cos(yaw0) + py0;
        path_point_tmp.yaw = yaw_local + yaw0;
        path_point_tmp.vx = vx_local * cos(yaw0) - vy_local * sin(yaw0);
        path_point_tmp.vy = vx_local * sin(yaw0) + vy_local * cos(yaw0);
        path_point_tmp.vyaw = vyaw_local;
        path.push_back(path_point_tmp);
      }
      sport_client.TrajectoryFollow(path);
      break;

    case stand_down: // 4. position stand down.
      sport_client.StandDown();
      break;

    case stand_up: // 5. position stand up
      sport_client.StandUp();
      break;

    case damp: // 6. damping mode
      sport_client.Damp();
      break;

    case recovery_stand: // 7. recovery stand
      sport_client.RecoveryStand();
      break;

    case sit:
      if (flag == 0)
      {
        sport_client.Sit();
        flag = 1;
      }
      break;

    case rise_sit:
      if (flag == 0)
      {
        sport_client.RiseSit();
        flag = 1;
      }
      break;

    case stretch:
      if (flag == 0)
      {
        sport_client.Stretch();
        flag = 1;
      }
      break;

    case wallow:
      if (flag == 0)
      {
        sport_client.Wallow();
        flag = 1;
      }
      break;
    /*
    case content:
      if (flag == 0)
      {
        sport_client.Content();
        flag = 1;
      }
      break;
    */
    case pose:
      if (flag == 0)
      {
        sport_client.Pose(true);
        flag = 1;
      }
      break;

    case scrape:
      if (flag == 0)
      {
        sport_client.Scrape();
        flag = 1;
      }
      break;

    case front_flip:
      if (flag == 0)
      {
        sport_client.FrontFlip();
        flag = 1;
      }
      break;

    case front_jump:
      if (flag == 0)
      {
        sport_client.FrontJump();
        flag = 1;
      }
      break;
    case front_pounce:
      if (flag == 0)
      {
        sport_client.FrontPounce();
        flag = 1;
      }
      break;

    case stop_move: // stop move
      sport_client.StopMove();
      break;

    default:
      sport_client.StopMove();
    }
  };

  // get initial position
  void GetInitState()
  {
    px0 = state.position()[0];
    py0 = state.position()[1];
    yaw0 = state.imu_state().rpy()[2];
    std::cout << "initial position: x0: " << px0 << ", y0: " << py0 << ", yaw0: " << yaw0 << std::endl;
  };

  void HighStateHandler(const void *message)
  {
    state = *(unitree_go::msg::dds_::SportModeState_ *)message;
  };

  unitree_go::msg::dds_::SportModeState_ state;
  unitree::robot::go2::SportClient sport_client;
  unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::SportModeState_> suber;

  double px0, py0, yaw0; // 初始状态的位置和偏航
  double ct = 0;         // 运行时间
  int flag = 0;          // 特殊动作执行标志
  float dt = 0.005;      // 控制步长0.001~0.01
};



#include <cmath>

// Paramètres du filtre complémentaire
const double alpha = 0.98;  // Pondération entre les sources de données
const double g = 9.81;      // Gravité (m/s²)

// Fonction de mise à jour du filtre complémentaire
void updateOrientation(double &roll, double &pitch, double dt,
                       double gr, double gp, // Gyroscope (rad/s)
                       double aroll, double apitch) // Accéléromètre (m/s²)
{
    // Estimation de l'orientation à partir du gyroscope (intégration)
    double roll_gyro = roll + gr * dt;
    double pitch_gyro = pitch + gp * dt;

    // Estimation de l'orientation à partir de l'accéléromètre
    double roll_acc = atan2(apitch, sqrt(aroll * aroll + 9.81 * 9.81)) * 180.0 / M_PI;
    double pitch_acc = atan2(-aroll, 9.81) * 180.0 / M_PI;

    // Fusion des données
    roll = alpha * roll_gyro + (1 - alpha) * roll_acc;
    pitch = alpha * pitch_gyro + (1 - alpha) * pitch_acc;
}


const double alpha_pos = 0.5; // Coefficient du filtre complémentaire

void updateTranslation(double &x, double &y, double &z, double dt,
                       double vx, double vy, double vz, 
                       double ax, double ay, double az)
{
    // Intégration des vitesses pour estimer la position
    double x_vel = x + vx * dt;
    double y_vel = y + vy * dt;
    double z_vel = z + vz * dt;

    // Double intégration de l'accélération pour estimer la position
    double x_acc = x + vx * dt + 0.5 * ax * dt * dt;
    double y_acc = y + vy * dt + 0.5 * ay * dt * dt;
    double z_acc = z + vz * dt + 0.5 * az * dt * dt;
    

    // Filtrage complémentaire
    x = alpha_pos * x_vel + (1 - alpha_pos) * x_acc;
    y = alpha_pos * y_vel + (1 - alpha_pos) * y_acc;
    z = alpha_pos * z_vel + (1 - alpha_pos) * z_acc;
}





#include <iostream>
#include <chrono>
#include <thread>

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        std::cout << "Usage: " << argv[0] << " networkInterface" << std::endl;
        exit(-1);
    }

    unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);
    Custom custom;

    sleep(1); // Attendre 1 seconde pour obtenir un état stable

    custom.GetInitState(); // Récupérer la position initiale
    
    double errgr = 0;
    double errgp = 0;
    double errgy = 0;
    double errax = 0;
    double erray = 0;
    double erraz = 0;
    double errvx = 0;
    double errvy = 0;
    double errvz = 0;
    
	for (int i=0;i<10;i++){
	     errgr += custom.state.imu_state().gyroscope()[0];
	     errgp += custom.state.imu_state().gyroscope()[1];
	     errgy += custom.state.imu_state().gyroscope()[2];
	     errax += custom.state.imu_state().accelerometer()[0];
	     erray += custom.state.imu_state().accelerometer()[1];
	     erraz += custom.state.imu_state().accelerometer()[2];
	     errvx += custom.state.velocity()[0];
	     errvy += custom.state.velocity()[1];
	     errvz += custom.state.velocity()[2];
	     std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
	     errgr /=10;
	     errgp /=10;
	     errgy /=10;
	     errax /=10;
	     erray /=10;
	     erraz /=10;
	     errvx /=10;
	     errvy /=10;
	     errvz /=10;
	     
	double roll = 0.0, pitch = 0.0, yaw = 0.0;
	double dt = 0.01;  // 10 ms
	double x = custom.state.position()[0];
	double y = custom.state.position()[1];
	double z = custom.state.position()[2];

	while (true)
	{

	    // Récupération des données filtrées
	    double gr = custom.state.imu_state().gyroscope()[0]-errgr;
	    double gp = custom.state.imu_state().gyroscope()[1]-errgp;
	    double gy = custom.state.imu_state().gyroscope()[2]-errgy;
	    
	    double vx = custom.state.velocity()[0]-errvx;
	    double vy = custom.state.velocity()[1]-errvy;
	    double vz = custom.state.velocity()[2]-errvz;

	    double ax = custom.state.imu_state().accelerometer()[0] - errax;
	    double ay = custom.state.imu_state().accelerometer()[1] - erray;
	    double az = custom.state.imu_state().accelerometer()[2] - erraz;

	  
	  	/* sans soustraction de l'erreur:
	    double gr = custom.state.imu_state().gyroscope()[0];
	    double gp = custom.state.imu_state().gyroscope()[1];
	    double gy = custom.state.imu_state().gyroscope()[2];
	    
	    double vx = custom.state.velocity()[0];
	    double vy = custom.state.velocity()[1];
	    double vz = custom.state.velocity()[2];

	    double ax = custom.state.imu_state().accelerometer()[0];
	    double ay = custom.state.imu_state().accelerometer()[1];
	    double az = custom.state.imu_state().accelerometer()[2];
	    	  */

	    // Mise à jour du filtre pour la position
	    updateTranslation(x, y, z, dt, vx, vy, vz, ax, ay, az);

	    // Mise à jour du filtre pour l'orientation
	    updateOrientation(roll, pitch, dt, gr, gp, ax, ay);

/*
	    // Affichage des résultats
	    std::cout << "roll: " << roll << " pitch: " << pitch << std::endl;
	    std::cout << "roll: " << custom.state.imu_state().rpy()[0] << " pitch: " << custom.state.imu_state().rpy()[1] << std::endl;
	    	*/
	    std::cout << "xcomp: " << x << " ycomp: " << y << " zcomp: " << z << std::endl;
	    std::cout << "x: " << custom.state.position()[0] << " y: " << custom.state.position()[1] << " z: " << custom.state.position()[2] << std::endl;
	    
	    std::this_thread::sleep_for(std::chrono::milliseconds(50)); // Pause 10 ms
	}



    
  



    return 0;
}

