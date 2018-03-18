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
        ROS_INFO("Initializing the on ground detector");

        // Don't subscribe until the fall starts
        humanIMUSub.reset(new message_filters::Subscriber<sensor_msgs::Imu>(nh, "/in", 1));
        humanIMUSub->registerCallback(boost::bind(&OnGroundDetector::update, this, _1));

        pub = nh.advertise<std_msgs::Header>("/human/on_ground", 1, true);

        ROS_INFO("On ground detector initialized successfully");
    }

private:

    bool isOnGround(const geometry_msgs::Quaternion& orientation) {
        // Rotate the quaternion to compensate for the initial rotation of the pole
        tf::Quaternion correctedOrientation = orientPose(orientation);

        // Compute a vector from the quaternion
        tf::Vector3 up(0, 0, 1);
        tf::Vector3 poseVector = tf::quatRotate(correctedOrientation, up).normalize();

        // Now compute a vector that is the projection onto the ground plane
        tf::Vector3 ground(poseVector.x(), poseVector.y(), 0.0);
        ground.normalized();

        tfScalar angle = poseVector.angle(ground);
        return angle < EPSILON;
    }

    // Compensate for the model being initial aligned to the x axis and oriented
    // up
    static tf::Quaternion orientPose(const geometry_msgs::Quaternion& orientationMsg)
    {
        tf::Quaternion rotation = tf::createQuaternionFromRPY(0, PI / 2.0, 0);

        tf::Quaternion orientation;
        tf::quaternionMsgToTF(orientationMsg, orientation);
        orientation *= rotation;
        return orientation;
    }

    void update(const sensor_msgs::ImuConstPtr& imuData)
    {
        if (isOnGround(imuData->orientation))
        {
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
    ROS_INFO("On ground detector exiting");
}

