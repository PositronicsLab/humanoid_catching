#include <iostream>
#include <fstream>
#include <vector>
#include <boost/filesystem.hpp>
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <std_msgs/Header.h>

using namespace std;

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
    auto_ptr<message_filters::Subscriber<std_msgs::Header> > humanOnGroundSub;

    //! Human fall subscriber
    auto_ptr<message_filters::Subscriber<std_msgs::Header> > humanFallSub;

    //! Start time
    ros::Time startTime;

    //! Whether the result was written
    bool resultsWritten;

public:
    FallStatsRecorder() : pnh("~")
    {
        ROS_DEBUG("Initializing the fall stats recorder");

        resultsWritten = false;

        humanFallSub.reset(new message_filters::Subscriber<std_msgs::Header>(nh, "/human/fall", 1));
        humanFallSub->registerCallback(boost::bind(&FallStatsRecorder::fallDetected, this, _1));

        // Don't subscribe until the fall starts
        humanOnGroundSub.reset(new message_filters::Subscriber<std_msgs::Header>(nh, "/human/on_ground", 1));
        humanOnGroundSub->unsubscribe();
    }

    ~FallStatsRecorder() {
        if(!resultsWritten) {
            ros::Duration contactTime = ros::Time::now() - startTime;
            ROS_INFO_STREAM("No on ground message received. Setting contact time to max: " << contactTime);
            printResults(contactTime);
        }
    }

private:
    void writeHeader(ofstream& outputCSV)
    {
        outputCSV << "Scenario Number, Ground Contact Time(s)" << endl;
    }

    void printResults(const ros::Duration& contactTime)
    {
        ROS_INFO("Printing results of trial");

        // Get the name of the folder to store the result in
        const char* resultsFolder = std::getenv("RESULTS_FOLDER");
        if(resultsFolder == NULL)
        {
            ROS_INFO_STREAM("Results folder not set. Using current directory.");
            resultsFolder = "";
        }

        const char* scenarioNumberStr = std::getenv("i");
        if(scenarioNumberStr == NULL)
        {
            ROS_INFO_STREAM("Scenario number not set. Using 0.");
            scenarioNumberStr = "0";
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

        outputCSV << scenarioNumberStr << ", " << contactTime.toSec() << endl;
        outputCSV.close();
        ROS_INFO_STREAM("Printing output file complete");
    }

    void fallDetected(const std_msgs::HeaderConstPtr& fallingMsg) {
        ROS_INFO("Human fall detected");
        humanFallSub->unsubscribe();

        startTime = ros::Time::now();
        humanOnGroundSub->registerCallback(boost::bind(&FallStatsRecorder::update, this, _1));
        humanOnGroundSub->subscribe();
    }

    void update(const std_msgs::HeaderConstPtr& imuData)
    {
        ros::Duration contactTime = ros::Time::now() - startTime;
        ROS_INFO_STREAM("Received an on ground message. Setting contact time to: " << contactTime);
        printResults(contactTime);
        humanOnGroundSub->unsubscribe();
        resultsWritten = true;
    }
};
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "fall_stats_recorder");

    FallStatsRecorder fsr;
    ros::spin();
}

