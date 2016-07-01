#include <iostream>
#include <fstream>
#include <vector>
#include <boost/filesystem.hpp>
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <humanoid_catching/IMU.h>
#include <geometry_msgs/PoseStamped.h>
#include <humanoid_catching/HumanFall.h>

using namespace std;
using namespace humanoid_catching;

namespace
{
class FallStatsRecorder
{

private:
    //! Node handle
    ros::NodeHandle nh;

    //! Private nh
    ros::NodeHandle pnh;

    //! Human IMU subscriber
    auto_ptr<message_filters::Subscriber<humanoid_catching::IMU> > humanIMUSub;

    //! Human fall subscriber
    auto_ptr<message_filters::Subscriber<HumanFall> > humanFallSub;

    //! Start time
    ros::Time startTime;

public:
    FallStatsRecorder() : pnh("~")
    {
        ROS_DEBUG("Initializing the fall stats recorder");

        humanFallSub.reset(new message_filters::Subscriber<HumanFall>(nh, "/human/fall", 1));
        humanFallSub->registerCallback(boost::bind(&FallStatsRecorder::fallDetected, this, _1));

        // Don't subscribe until the fall starts
        humanIMUSub.reset(new message_filters::Subscriber<humanoid_catching::IMU>(nh, "/human/imu", 1));
        humanIMUSub->unsubscribe();
    }

private:
    void writeHeader(ofstream& outputCSV)
    {
        outputCSV << "Ground Contact Time(s)" << endl;
    }

    void printResults(const ros::Duration& contactTime)
    {
        // Get the name of the folder to store the result in
        const char* resultsFolder = std::getenv("RESULTS_FOLDER");
        if(resultsFolder == NULL)
        {
            cout << "Results folder not set. Using current directory." << endl;
            resultsFolder = "";
        }

        const string resultsFileName = boost::filesystem::current_path().string() + "/" + string(resultsFolder) + "/" + "results.csv";
        ROS_INFO_STREAM("Using results file: " << resultsFileName);
        bool exists = boost::filesystem::exists(resultsFileName);
        ofstream outputCSV;
        outputCSV.open(resultsFileName.c_str(), ios::out | ios::app);
        assert(outputCSV.is_open());

        if(!exists)
        {
            writeHeader(outputCSV);
        }

        outputCSV << contactTime.toSec() << ", " << endl;
        outputCSV.close();
        ROS_INFO_STREAM("Printing output file complete");
    }

    void fallDetected(const HumanFallConstPtr& fallingMsg) {
        ROS_INFO("Human fall detected at @ %f", fallingMsg->header.stamp.toSec());
        humanFallSub->unsubscribe();

        startTime = ros::Time::now();
        humanIMUSub->registerCallback(boost::bind(&FallStatsRecorder::worldUpdate, this, _1));
        humanIMUSub->subscribe();
    }

    void worldUpdate(const humanoid_catching::IMUConstPtr& imuData)
    {

        ros::Duration contactTime = ros::Time::now() - startTime;
        if (imuData->pose.position.z <= 0.025 + 0.01 || contactTime.toSec() >= 30.0)
        {
            ROS_INFO_STREAM("Setting contact time to: " << contactTime << ". Current z = " << imuData->pose.position.z);
            printResults(contactTime);
            humanIMUSub->unsubscribe();
        }
    }
};
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "fall_stats_recorder");

    FallStatsRecorder fsr;
    ros::spin();
}

