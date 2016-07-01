#include <ros/ros.h>
#include <humanoid_catching/IMU.h>
#include <sensor_msgs/Imu.h>
#include <message_filters/subscriber.h>
#include <geometry_msgs/PoseStamped.h>

namespace {
using namespace std;
using namespace geometry_msgs;

class ImuIntegrator {
private:
    //! Publisher for the human pose
	ros::Publisher humanPosePub;

	//! Node handle
	ros::NodeHandle nh;

	//! Private nh
	ros::NodeHandle pnh;

	//! Last observation time
	ros::Time lastTime;

	//! Integrated velocity
	geometry_msgs::Twist velocity;

    auto_ptr<message_filters::Subscriber<sensor_msgs::Imu> > imuSub;
public:
	ImuIntegrator() :
		pnh("~") {
         humanPosePub = nh.advertise<humanoid_catching::IMU>(
				"out", 1);
         imuSub.reset(new message_filters::Subscriber<sensor_msgs::Imu>(nh, "imu/data", 1));
         imuSub->registerCallback(boost::bind(&ImuIntegrator::imuCallback, this, _1));
	}

private:

    void imuCallback(sensor_msgs::ImuConstPtr data){

        double dt = data->header.stamp.toSec() - lastTime.toSec();
        if(lastTime.toSec() == 0){
            dt = 0.0;
        }

        lastTime = data->header.stamp;
        velocity.linear.x += dt * data->linear_acceleration.x;
        velocity.linear.y += dt * data->linear_acceleration.y;
        velocity.linear.z += dt * data->linear_acceleration.z;

        humanoid_catching::IMU imu;
        imu.header =data->header;
        imu.pose.orientation = data->orientation;
        imu.acceleration.linear = data->linear_acceleration;
        imu.velocity.angular = data->angular_velocity;
        imu.velocity.linear = velocity.linear;

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
