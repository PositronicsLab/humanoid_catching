#include <ros/ros.h>
#include <std_msgs/Header.h>

namespace
{

using namespace std;
using namespace ros;

class FallStarter
{
private:

    //! Node handle
    ros::NodeHandle nh;

    //! Private nh
    ros::NodeHandle pnh;

    //! Catching controller notifier
    ros::Publisher catchNotifier;

    //! Do not wait for robot to come online
    bool waitForBalancer;

public:
    FallStarter() :
        pnh("~")
    {
        catchNotifier = nh.advertise<std_msgs::Header>("/human/fall", 1, true);
    }

    void fall()
    {
        ROS_DEBUG("Notifying fall catcher");
        std_msgs::Header header;
        header.stamp = ros::Time::now();
        catchNotifier.publish(header);
        ROS_DEBUG("Fall initiation complete");
    }
};
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "fall_starter");
    FallStarter fs;
    fs.fall();
    ros::Duration(3.0).sleep();
}
