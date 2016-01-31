#include <ros/ros.h>
#include <humanoid_catching/IMU.h>
#include <sensor_msgs/Imu.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/subscriber.h>
#include <geometry_msgs/PoseStamped.h>

namespace {
using namespace std;
using namespace geometry_msgs;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Imu, geometry_msgs::PoseStamped> ImuPoseSyncPolicy;
typedef message_filters::Synchronizer<ImuPoseSyncPolicy> ImuPoseSync;

class ImuIntegrator {
private:
    //! Publisher for the human pose
	ros::Publisher humanPosePub;

	//! Node handle
	ros::NodeHandle nh;

	//! Private nh
	ros::NodeHandle pnh;

	auto_ptr<ImuPoseSync> sync;

    auto_ptr<message_filters::Subscriber<geometry_msgs::PoseStamped> > poseSub;
    auto_ptr<message_filters::Subscriber<sensor_msgs::Imu> > imuSub;
public:
	ImuIntegrator() :
		pnh("~") {
         humanPosePub = nh.advertise<humanoid_catching::IMU>(
				"out", 1);
         poseSub.reset(new message_filters::Subscriber<geometry_msgs::PoseStamped>(nh, "imu/pose", 1));
         imuSub.reset(new message_filters::Subscriber<sensor_msgs::Imu>(nh, "imu/data", 1));
         sync.reset(new ImuPoseSync(ImuPoseSyncPolicy(10), *imuSub, *poseSub));
         sync->registerCallback(boost::bind(&ImuIntegrator::imuCallback, this, _1, _2));
	}

private:

    void imuCallback(sensor_msgs::ImuConstPtr data, geometry_msgs::PoseStampedConstPtr pose){

        humanoid_catching::IMU imu;
        imu.header = pose->header;
        imu.pose = pose->pose;
        imu.linear_acceleration = data->linear_acceleration;
        imu.angular_velocity = data->angular_velocity;

        // Publish the event
        ROS_DEBUG_STREAM("Publishing a human IMU event: " << imu);
        humanPosePub.publish(imu);
      }
};
}
int main(int argc, char** argv) {
	ros::init(argc, argv, "imu_integrator");

	ImuIntegrator ii;
	ros::spin();
}
