#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <message_filters/subscriber.h>
#include <boost/math/distributions/normal.hpp>
#include <boost/random.hpp>
#include <tf/transform_listener.h>

namespace
{
using namespace std;
using namespace geometry_msgs;
using namespace std_msgs;

static const double MEAN = 0.0;
static const double STD_DEV = 2.0;

class ImuGaussianNoise
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
    boost::normal_distribution<double> dist;

    boost::mt19937 engine;

public:
    ImuGaussianNoise() :
        pnh("~"), dist(MEAN, STD_DEV)
    {
        imuSub.reset(new message_filters::Subscriber<sensor_msgs::Imu>(nh, "/in", 1));
        imuSub->registerCallback(boost::bind(&ImuGaussianNoise::imuCallback, this, _1));

        engine.seed(0);
        filteredPub = nh.advertise<sensor_msgs::Imu>("/out", 1);
    }

private:

    void imuCallback(sensor_msgs::ImuConstPtr data)
    {
        ROS_DEBUG("Unfiltered angular velocity [%f] [%f] [%f]", data->angular_velocity.x, data->angular_velocity.y, data->angular_velocity.z);

        // Copy over all fields
        sensor_msgs::ImuPtr output(new sensor_msgs::Imu(*data));

        // Compute random xyz
        boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > randNormal(engine, dist);

        double x = randNormal();
        double y = randNormal();
        double z = randNormal();

        ROS_DEBUG("Generated random parameters for x [%f], y [%f], z [%f]", x, y, z);
        output->angular_velocity.x += x;
        output->angular_velocity.y += y;
        output->angular_velocity.z += z;

        filteredPub.publish(output);
    }
};
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "imu_gaussian_noise");

    ImuGaussianNoise imuFilter;
    ros::spin();
}
