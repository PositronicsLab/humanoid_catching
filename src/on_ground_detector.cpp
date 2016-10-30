#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Header.h>
#include <boost/math/constants/constants.hpp>
#include <tf/tf.h>

using namespace std;

static const double EPSILON = 0.01;
static const double PI = boost::math::constants::pi<double>();

namespace
{
class OnGroundDetector
{

private:
    //! Node handle
    ros::NodeHandle nh;

    //! Private nh
    ros::NodeHandle pnh;

    //! Human IMU subscriber
    auto_ptr<message_filters::Subscriber<sensor_msgs::Imu> > humanIMUSub;

    //! On ground publisher
    ros::Publisher pub;

public:
    OnGroundDetector() : pnh("~")
    {
        ROS_DEBUG("Initializing the on ground detector");

        // Don't subscribe until the fall starts
        humanIMUSub.reset(new message_filters::Subscriber<sensor_msgs::Imu>(nh, "/in", 1));
        humanIMUSub->registerCallback(boost::bind(&OnGroundDetector::update, this, _1));

        pub = nh.advertise<std_msgs::Header>("/human/on_ground", 1, true);
    }

private:

    bool isOnGround(const geometry_msgs::Quaternion& orientation) {
        double roll, pitch, yaw;
        tf::Matrix3x3(tf::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w)).getRPY(roll, pitch, yaw);
        return (pitch >= -EPSILON || pitch < -PI + EPSILON || abs(yaw) > PI / 2 - EPSILON);
    }

    void update(const sensor_msgs::ImuConstPtr& imuData)
    {
        if (isOnGround(imuData->orientation))
        {
            ROS_DEBUG("Human is on ground!");
            std_msgs::Header header;
            header.stamp = ros::Time::now();
            pub.publish(header);
        }
    }
};
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "on_ground_detector");
    OnGroundDetector ogd;
    ros::spin();
}

