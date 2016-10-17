#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <message_filters/subscriber.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <tf/transform_listener.h>

namespace
{
using namespace std;
using namespace geometry_msgs;
using namespace std_msgs;

class ImuVisualizer
{
private:
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
    ImuVisualizer() :
        pnh("~")
    {
        imuSub.reset(new message_filters::Subscriber<sensor_msgs::Imu>(nh, "/in", 1));
        imuSub->registerCallback(boost::bind(&ImuVisualizer::imuCallback, this, _1));

        poseVizPub = nh.advertise<geometry_msgs::PoseStamped>("imu/pose", 1);
        velocityVizPub = nh.advertise<geometry_msgs::WrenchStamped>("imu/velocity", 1);
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

        // Print out the roll pitch yaw values
        tf::Quaternion q;
        tf::quaternionMsgToTF(imu.orientation, q);
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        ROS_INFO("RPY: [%f] [%f] [%f]", roll, pitch, yaw);

        if (poseVizPub.getNumSubscribers() > 0) {
            visualizePose(imu.header, imu.orientation);
        }

        if (velocityVizPub.getNumSubscribers() > 0) {
            visualizeVelocity(imu.header, imu.angular_velocity);
        }
    }
};
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "imu_visualizer");

    ImuVisualizer ii;
    ros::spin();
}
