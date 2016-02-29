#include <ros/ros.h>
#include <humanoid_catching/HumanFall.h>
#include <gazebo_msgs/ApplyBodyWrench.h>
#include <geometry_msgs/PoseStamped.h>

namespace {

using namespace std;
using namespace ros;

static const float NOTIFICATION_DELAY = 0.1;

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

   //! Delay before notifying
   ros::Duration notificationDelay;
public:
	FallStarter() :
		pnh("~"), notificationDelay(NOTIFICATION_DELAY) {
            gazeboClient = nh.serviceClient<gazebo_msgs::ApplyBodyWrench>("/gazebo/apply_body_wrench", true /* persistent */);
            catchNotifier = nh.advertise<humanoid_catching::HumanFall>("/human/fall", 1);
    }

    void fall() {
        ROS_INFO("Initiating fall");
        gazebo_msgs::ApplyBodyWrench wrench;
        wrench.request.body_name = "human::link";
        wrench.request.wrench.torque.x = 0;
        wrench.request.wrench.torque.y = 500;
        wrench.request.wrench.torque.z = 0;
        wrench.request.duration = ros::Duration(0.001);

        if (!gazeboClient.call(wrench)) {
            ROS_ERROR("Failed to apply wrench");
            return;
        }

        ROS_INFO("Pausing before notifying");
        notificationDelay.sleep();
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
