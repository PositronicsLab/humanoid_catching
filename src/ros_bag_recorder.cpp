#include <boost/filesystem.hpp>
#include <ros/ros.h>
#include <rosbag/recorder.h>

using namespace std;

namespace
{
class RosBagRecorder
{

private:
    //! Node handle
    ros::NodeHandle nh;

    //! Private nh
    ros::NodeHandle pnh;

    //! Recorder
    auto_ptr<rosbag::Recorder> recorder;
public:
    RosBagRecorder() : pnh("~")
    {
        ROS_DEBUG("Initializing the ros bag recorder");

        rosbag::RecorderOptions options;
        options.topics.push_back("/human/imu");
        options.topics.push_back("/human/fall");
        options.topics.push_back("/human/on_ground");
        options.prefix = boost::filesystem::current_path().string() + "/bags/";
        options.name = "human_imu.bag";
        recorder.reset(new rosbag::Recorder(options));
        recorder->run();
    }
};
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ros_bag_recorder");

    RosBagRecorder rbc;
    ros::spin();
}

