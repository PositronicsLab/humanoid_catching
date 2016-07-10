#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <message_filters/subscriber.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <tf/tf.h>

namespace
{
using namespace std;
using namespace geometry_msgs;
using namespace std_msgs;

static const double GRAVITY = 9.80665;

class ImuIntegrator
{
private:
    //! Publisher for the human pose
    ros::Publisher posePub;

    //! Node handle
    ros::NodeHandle nh;

    //! Private nh
    ros::NodeHandle pnh;

    //! IMU subscription
    auto_ptr<message_filters::Subscriber<sensor_msgs::Imu> > imuSub;

    //! Publisher for the pose visualization
    ros::Publisher poseVizPub;

    //! Publisher for the velocity visualization
    ros::Publisher velocityVizPub;
public:
    ImuIntegrator() :
        pnh("~")
    {
        posePub = nh.advertise<sensor_msgs::Imu>("out", 1);
        imuSub.reset(new message_filters::Subscriber<sensor_msgs::Imu>(nh, "/in", 1));
        imuSub->registerCallback(boost::bind(&ImuIntegrator::imuCallback, this, _1));

        poseVizPub = nh.advertise<geometry_msgs::PoseStamped>("imu/pose", 1);
        velocityVizPub = nh.advertise<geometry_msgs::WrenchStamped>("/imu/velocity", 1);
    }

private:

    void visualizePose(const Header& header, const Quaternion& orientation) {
        PoseStamped poseStamped;
        poseStamped.pose.orientation = orientation;
        poseStamped.header = header;
        poseVizPub.publish(poseStamped);
    }

    void visualizeVelocity(const Header& header, const Vector3& angular) {
        WrenchStamped wrench;
        wrench.header = header;
        wrench.wrench.torque = angular;
        velocityVizPub.publish(wrench);
    }

    void imuCallback(sensor_msgs::ImuConstPtr data)
    {
        sensor_msgs::Imu imu;
        imu.header = data->header;
        imu.orientation = data->orientation;
        imu.linear_acceleration = data->linear_acceleration;
        imu.angular_velocity = data->angular_velocity;

        if (poseVizPub.getNumSubscribers() > 0) {
            visualizePose(imu.header, imu.orientation);
        }

        if (velocityVizPub.getNumSubscribers() > 0) {
            visualizeVelocity(imu.header, imu.angular_velocity);
        }

        // Publish the event
        ROS_DEBUG_STREAM("Publishing a human IMU event: " << imu);
        posePub.publish(imu);
    }
};
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "imu_integrator");

    ImuIntegrator ii;
    ros::spin();
}
