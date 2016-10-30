#include <iostream>
#include <fstream>
#include <vector>
#include <boost/filesystem.hpp>
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <std_msgs/Header.h>
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

    //! Human IMU subscriber
    auto_ptr<message_filters::Subscriber<std_msgs::Header> > humanOnGroundSub;

    //! Human fall subscriber
    auto_ptr<message_filters::Subscriber<std_msgs::Header> > humanFallSub;

    //! Recorder
    auto_ptr<rosbag::Recorder> recorder;
public:
    RosBagRecorder() : pnh("~")
    {
        ROS_DEBUG("Initializing the ros bag recorder");

        humanFallSub.reset(new message_filters::Subscriber<std_msgs::Header>(nh, "/human/fall", 1));
        humanFallSub->registerCallback(boost::bind(&RosBagRecorder::fallDetected, this, _1));

        // Don't subscribe until the fall starts
        humanOnGroundSub.reset(new message_filters::Subscriber<std_msgs::Header>(nh, "/human/on_ground", 1));
        humanOnGroundSub->unsubscribe();

        rosbag::RecorderOptions options;
        options.topics.push_back("/human/imu");
        options.prefix = getPrefix();
        options.name = "human_imu.bag";
        recorder.reset(new rosbag::Recorder(options));
    }

private:

    string getPrefix()
    {
        // Get the name of the folder to store the result in
        const char* bagFolder = std::getenv("BAG_FOLDER");
        if(bagFolder == NULL)
        {
            ROS_INFO_STREAM("Results folder not set. Using 'bags'.");
            bagFolder = "bags";
        }

        const char* scenarioNumberStr = std::getenv("i");
        if(scenarioNumberStr == NULL)
        {
            ROS_INFO_STREAM("Scenario number not set. Using 0.");
            scenarioNumberStr = "0";
        }

        const string resultsPrefix = boost::filesystem::current_path().string() + "/" + string(bagFolder) + "/";
        return resultsPrefix;

    }

    void fallDetected(const std_msgs::HeaderConstPtr& fallingMsg) {
        ROS_INFO("Human fall detected at @ %f", fallingMsg->stamp.toSec());

        recorder->run();

        humanOnGroundSub->registerCallback(boost::bind(&RosBagRecorder::onGroundDetected, this, _1));
        humanOnGroundSub->subscribe();
    }

    void onGroundDetected(const std_msgs::HeaderConstPtr& imuData)
    {
        recorder.reset(NULL);
    }
};
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ros_bag_recorder");

    RosBagRecorder rbc;
    ros::spin();
}

