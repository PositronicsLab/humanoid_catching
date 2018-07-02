#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <message_filters/subscriber.h>

namespace
{
using namespace std;
using namespace geometry_msgs;
using namespace std_msgs;

//! Estimated sampling rate in hz
static const double SAMPLE_RATE = 100;
static const double FC = 10 / SAMPLE_RATE;
static const double B1 = exp(-2.0 * M_PI * FC);
static const double A0 = 1.0 - B1;

class ImuFilter
{
private:
    //! Node handle
    ros::NodeHandle nh;

    //! Private nh
    ros::NodeHandle pnh;

    //! IMU subscription
    auto_ptr<message_filters::Subscriber<sensor_msgs::Imu> > imuSub;

    //! Publisher for the filterered IMU
    ros::Publisher filteredPub;

    //! Previous values
    vector<double> z;
public:
    ImuFilter() :
        pnh("~"), z(3)
    {
        ROS_INFO("Constructing IMU filter with B1: %f and A0: %f", B1, A0);
        imuSub.reset(new message_filters::Subscriber<sensor_msgs::Imu>(nh, "/in", 1));
        imuSub->registerCallback(boost::bind(&ImuFilter::imuCallback, this, _1));

        filteredPub = nh.advertise<sensor_msgs::Imu>("/out", 1);
    }

private:

    void imuCallback(sensor_msgs::ImuConstPtr data)
    {
        // Copy over all fields
        sensor_msgs::ImuPtr output(new sensor_msgs::Imu(*data));

        ROS_DEBUG("Unfiltered angular velocity [%f] [%f] [%f]", output->angular_velocity.x, output->angular_velocity.y, output->angular_velocity.z);

        // Update angular velocity
        output->angular_velocity.x = z[0] = output->angular_velocity.x * A0 + z[0] * B1;
        output->angular_velocity.y = z[1] = output->angular_velocity.x * A0 + z[1] * B1;
        output->angular_velocity.z = z[2] = output->angular_velocity.x * A0 + z[2] * B1;

        ROS_DEBUG("Filtered angular velocity [%f] [%f] [%f]", output->angular_velocity.x, output->angular_velocity.y, output->angular_velocity.z);
        filteredPub.publish(output);
    }
};
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "imu_filter");

    ImuFilter imuFilter;
    ros::spin();
}
