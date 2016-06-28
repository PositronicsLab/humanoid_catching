#include <iostream>
#include <fstream>
#include <vector>
#include <boost/filesystem.hpp>
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <humanoid_catching/IMU.h>
#include <geometry_msgs/PoseStamped.h>

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
    auto_ptr<message_filters::Subscriber<humanoid_catching::IMU> > humanIMUSub;
public:
    FallStatsRecorder() : pnh("~")
    {
        ROS_INFO("Initializing the fall stats recorder");

        humanIMUSub.reset(
            new message_filters::Subscriber<humanoid_catching::IMU>(nh, "/human/imu", 1));
        humanIMUSub->registerCallback(boost::bind(&FallStatsRecorder::worldUpdate, this, _1));
    }

private:
    void writeHeader(ofstream& outputCSV)
    {
        outputCSV << "Ground Contact Time(s)" << endl;
    }

    void printResults(const ros::Time& contactTime)
    {
        // Get the name of the folder to store the result in
        const char* resultsFolder = std::getenv("RESULTS_FOLDER");
        if(resultsFolder == NULL)
        {
            cout << "Results folder not set. Using current directory." << endl;
            resultsFolder = boost::filesystem::current_path().c_str();
        }

        const string resultsFileName = string(resultsFolder) + "/" + "results.csv";
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

    void worldUpdate(const humanoid_catching::IMUConstPtr& imuData)
    {
        ros::Time contactTime = ros::Time::now();
        if (imuData->pose.position.z <= 0.025 + 0.01 || contactTime.toSec() >= 30.0)
        {
            ROS_INFO_STREAM("Setting contact time to: " << contactTime << ". Current z = " << imuData->pose.position.z);
            printResults(contactTime);

        }
        humanIMUSub->unsubscribe();
    }
};
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "fall_stats_recorder");

    FallStatsRecorder fsr;
    ros::spin();
}

