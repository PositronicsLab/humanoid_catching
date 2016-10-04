#include <iostream>
#include <fstream>
#include <vector>
#include <boost/filesystem.hpp>
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
class FallStatsRecorder
{

private:
    //! Node handle
    ros::NodeHandle nh;

    //! Private nh
    ros::NodeHandle pnh;

    //! Human IMU subscriber
    auto_ptr<message_filters::Subscriber<sensor_msgs::Imu> > humanIMUSub;

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
        humanIMUSub.reset(new message_filters::Subscriber<sensor_msgs::Imu>(nh, "/human/imu", 1));
        humanIMUSub->unsubscribe();
    }

    ~FallStatsRecorder() {
        if(!resultsWritten) {
            ros::Duration contactTime = ros::Time::now() - startTime;
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
        // Get the name of the folder to store the result in
        const char* resultsFolder = std::getenv("RESULTS_FOLDER");
        if(resultsFolder == NULL)
        {
            cout << "Results folder not set. Using current directory." << endl;
            resultsFolder = "";
        }

        const char* scenarioNumberStr = std::getenv("i");
        if(scenarioNumberStr == NULL)
        {
            cout << "Scenario number not set. Using 0." << endl;
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

        outputCSV << scenarioNumberStr << ", " << contactTime.toSec() << ", " << endl;
        outputCSV.close();
        ROS_INFO_STREAM("Printing output file complete");
    }

    void fallDetected(const std_msgs::HeaderConstPtr& fallingMsg) {
        ROS_INFO("Human fall detected at @ %f", fallingMsg->stamp.toSec());
        humanFallSub->unsubscribe();

        startTime = ros::Time::now();
        humanIMUSub->registerCallback(boost::bind(&FallStatsRecorder::worldUpdate, this, _1));
        humanIMUSub->subscribe();
    }

    bool isOnGround(const geometry_msgs::Quaternion& orientation) {
        double roll, pitch, yaw;
        tf::Matrix3x3(tf::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w)).getRPY(roll, pitch, yaw);
        return (pitch >= -EPSILON || pitch < -PI + EPSILON || abs(yaw) > PI / 2 - EPSILON);
    }

    void worldUpdate(const sensor_msgs::ImuConstPtr& imuData)
    {

        ros::Duration contactTime = ros::Time::now() - startTime;
        if (isOnGround(imuData->orientation) || contactTime.toSec() >= 30.0)
        {
            ROS_INFO_STREAM("Setting contact time to: " << contactTime);
            printResults(contactTime);
            humanIMUSub->unsubscribe();
            resultsWritten = true;
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

