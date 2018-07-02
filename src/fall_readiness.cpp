#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/Imu.h>
#include <tf/tf.h>
#include <boost/math/constants/constants.hpp>

namespace
{

using namespace std;
using namespace ros;

static const double ANGULAR_VELOCITY_THRESHOLD = 0.01; // radians per second
static const double ANGULAR_TOLERANCE = 2 * boost::math::constants::pi<double>() / 100.0;

class FallReadiness
{
private:

    //! Node handle
    ros::NodeHandle nh;

    //! Private nh
    ros::NodeHandle pnh;

    //! IMU subscription
    auto_ptr<message_filters::Subscriber<sensor_msgs::Imu> > imuSub;

    //! Whether the pole is currently stopped
    bool isStopped;

    //! Whether the pole is allowably close to vertical
    bool isWithinAngularTolerance;

    //! Whether this is the first iteration
    bool isFirstIteration;

public:
    FallReadiness() :
        pnh("~")
    {
        imuSub.reset(new message_filters::Subscriber<sensor_msgs::Imu>(nh, "/human/imu", 1));
        imuSub->registerCallback(boost::bind(&FallReadiness::imuCallback, this, _1));
        imuSub->subscribe();

        isStopped = false;
        isWithinAngularTolerance = false;
        isFirstIteration = true;

        ROS_DEBUG("Fall readiness initialized successfully");
    }

    void imuCallback(sensor_msgs::ImuConstPtr data)
    {
        ROS_DEBUG("Angular velocity [%f] [%f] [%f]", data->angular_velocity.x, data->angular_velocity.y, data->angular_velocity.z);
        bool previousIsStopped = isStopped;
        isStopped = fabs(data->angular_velocity.x) < ANGULAR_VELOCITY_THRESHOLD && fabs(data->angular_velocity.y) < ANGULAR_VELOCITY_THRESHOLD
                    && fabs(data->angular_velocity.z) < ANGULAR_VELOCITY_THRESHOLD;

        tf::Quaternion q;
        tf::quaternionMsgToTF(data->orientation, q);
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        ROS_DEBUG("Roll, Pitch, Yaw [%f] [%f] [%f]", roll, pitch, yaw);

        bool previousWithinTolerance = isWithinAngularTolerance;
        isWithinAngularTolerance = pitch - boost::math::constants::pi<double>() / 2.0 < ANGULAR_TOLERANCE
          && fabs(yaw) < ANGULAR_TOLERANCE;

        if (isFirstIteration || isStopped != previousIsStopped) {
            ROS_INFO("Currently stopped: %s", isStopped ? "true" : "false");
        }
        if (isFirstIteration || previousWithinTolerance != isWithinAngularTolerance) {
            ROS_INFO("Is within angular tolerance: %s", isWithinAngularTolerance ? "true" : "false");
        }
        isFirstIteration = false;
    }
};
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "fall_readiness");
    FallReadiness fs;
    ros::spin();
}
