#include <ros/ros.h>
#include <humanoid_catching/HumanFall.h>
#include <gazebo_msgs/ApplyBodyWrench.h>
#include <geometry_msgs/PoseStamped.h>

namespace {

using namespace std;
using namespace ros;

static const double NOTIFICATION_DELAY_DEFAULT = 0.1;
static const double DURATION_DEFAULT = 0.001;
class FallStarter {
private:

   //! Node handle
   ros::NodeHandle nh;

   //! Private nh
   ros::NodeHandle pnh;

    //! Gazebo client
    ros::ServiceClient gazeboClient;

   //! Catching controller notifier
   ros::Publisher catchNotifier;

   //! X torque
   double x;

   //! Y torque
   double y;

   //! Z torque
   double z;

   //! Torque Duration
   double duration;

   //! Notification delay
   double delay;
public:
	FallStarter() :
		pnh("~") {
            gazeboClient = nh.serviceClient<gazebo_msgs::ApplyBodyWrench>("/gazebo/apply_body_wrench", true /* persistent */);
            catchNotifier = nh.advertise<humanoid_catching::HumanFall>("/human/fall", 1);
            pnh.param("x", x, 0.0);
            pnh.param("y", y, 0.0);
            pnh.param("z", z, 0.0);
            pnh.param("duration", duration, DURATION_DEFAULT);
            pnh.param("delay", delay, NOTIFICATION_DELAY_DEFAULT);
    }

    void fall() {
        ROS_INFO("Initiating fall");
        gazebo_msgs::ApplyBodyWrench wrench;
        wrench.request.body_name = "human::link";
        wrench.request.wrench.torque.x = x;
        wrench.request.wrench.torque.y = y;
        wrench.request.wrench.torque.z = z;
        wrench.request.duration = ros::Duration(duration);

        if (!gazeboClient.call(wrench)) {
            ROS_ERROR("Failed to apply wrench");
            return;
        }

        ROS_INFO("Pausing before notifying");
        ros::Duration(delay).sleep();
        ROS_INFO("Notifying fall catcher");
        catchNotifier.publish(humanoid_catching::HumanFall());
        ROS_INFO("Fall initiation complete");
    }
};
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "fall_starter");
	FallStarter fs;
	fs.fall();
}
