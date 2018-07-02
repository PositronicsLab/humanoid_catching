#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <std_msgs/Duration.h>
#include <iostream>
#include <fstream>
#include <boost/filesystem.hpp>
#include <humanoid_catching/IKMetric.h>

using namespace std;

inline double square(const double a) {
    return a * a;
}

class MetricsRecorder {
private:
    ros::NodeHandle nh;
    ros::NodeHandle pnh;

    double initialReactionTime;

    double meanReactionTime;
    double meanSquaredReactionTime;
    unsigned int nRT;

    double meanBalanceCalcTime;
    double meanSquaredBalanceCalcTime;
    unsigned int nBC;

    double meanInterceptCalcTime;
    double meanSquaredInterceptCalcTime;
    unsigned int nIC;

    double meanFallPredictTime;
    double meanSquaredFallPredictTime;
    unsigned int nFP;

    double meanIKTime;
    double meanSquaredIKTime;
    double meanIKDistance;
    double meanSquaredIKDistance;
    unsigned int nIK;
    unsigned int nSuccessfulIK;

    auto_ptr<message_filters::Subscriber<std_msgs::Duration> > reactionTimeSub;
    auto_ptr<message_filters::Subscriber<std_msgs::Duration> > balanceCalcTimeSub;
    auto_ptr<message_filters::Subscriber<std_msgs::Duration> > interceptCalcTimeSub;
    auto_ptr<message_filters::Subscriber<std_msgs::Duration> > fallPredictTimeSub;
    auto_ptr<message_filters::Subscriber<humanoid_catching::IKMetric> > ikMetricSub;

public:
    MetricsRecorder() :
            pnh("~"),
            initialReactionTime(0),
            meanReactionTime(0),
            meanSquaredReactionTime(0),
            nRT(0),
            meanBalanceCalcTime(0),
            meanSquaredBalanceCalcTime(0),
            nBC(0),
            meanInterceptCalcTime(0),
            meanSquaredInterceptCalcTime(0),
            nIC(0),
            meanFallPredictTime(0),
            meanSquaredFallPredictTime(0),
            nFP(0),
            meanIKTime(0),
            meanSquaredIKTime(0),
            meanIKDistance(0),
            meanSquaredIKDistance(0),
            nIK(0),
            nSuccessfulIK(0)
             {

        // Setup the subscribers
        reactionTimeSub.reset(
                new message_filters::Subscriber<std_msgs::Duration>(nh,
                        "reaction_time", 1));
        reactionTimeSub->registerCallback(boost::bind(&MetricsRecorder::reactionTimeCallback, this, _1));

        balanceCalcTimeSub.reset(
                new message_filters::Subscriber<std_msgs::Duration>(nh,
                        "balance_calc_time", 1));
        balanceCalcTimeSub->registerCallback(boost::bind(&MetricsRecorder::balanceCalcCallback, this, _1));

        interceptCalcTimeSub.reset(
                new message_filters::Subscriber<std_msgs::Duration>(nh,
                        "intercept_calc_time", 1));
        interceptCalcTimeSub->registerCallback(boost::bind(&MetricsRecorder::interceptCalcCallback, this, _1));

        fallPredictTimeSub.reset(
                new message_filters::Subscriber<std_msgs::Duration>(nh,
                        "fall_predict_time", 1));
        fallPredictTimeSub->registerCallback(boost::bind(&MetricsRecorder::fallPredictCalcCallback, this, _1));

        ikMetricSub.reset(
                new message_filters::Subscriber<humanoid_catching::IKMetric>(nh,
                        "ik_metric", 1));
        ikMetricSub->registerCallback(boost::bind(&MetricsRecorder::ikMetricCallback, this, _1));
    }

private:

    void ikMetricCallback(const humanoid_catching::IKMetricConstPtr& metric) {

        // Increase number of samples
        nIK++;

        if (metric->was_successful) {
            nSuccessfulIK++;

            // Metrics are only meaningful when the query is successful
            double ikTime = metric->time.toSec();
            double deltaP = ikTime - meanIKTime;
            meanIKTime += deltaP / double(nSuccessfulIK);
            meanSquaredIKTime += square(deltaP);

            double ikDistance = metric->distance;
            double deltaPD = ikDistance - meanIKDistance;
            meanIKDistance += deltaPD / double(nSuccessfulIK);
            meanSquaredIKDistance += square(deltaPD);
        }
    }

    void reactionTimeCallback(const std_msgs::DurationConstPtr& duration) {

        if (nRT == 0) {
            initialReactionTime = duration->data.toSec();
        }

        // Increase number of samples
        nRT++;

        double reactionTime = duration->data.toSec();

        double deltaP = reactionTime - meanReactionTime;
        meanReactionTime += deltaP / double(nRT);
        meanSquaredReactionTime += square(deltaP);
    }

    void balanceCalcCallback(const std_msgs::DurationConstPtr& duration) {

        // Increase number of samples
        nBC++;

        double bcTime = duration->data.toSec();

        double deltaP = bcTime - meanBalanceCalcTime;
        meanBalanceCalcTime += deltaP / double(nBC);
        meanSquaredBalanceCalcTime += square(deltaP);
    }

    void interceptCalcCallback(const std_msgs::DurationConstPtr& duration) {

        // Increase number of samples
        nIC++;

        double icTime = duration->data.toSec();

        double deltaP = icTime - meanInterceptCalcTime;
        meanInterceptCalcTime += deltaP / double(nIC);
        meanSquaredInterceptCalcTime += square(deltaP);
    }

    void fallPredictCalcCallback(const std_msgs::DurationConstPtr& duration) {

        // Increase number of samples
        nFP++;

        double fpTime = duration->data.toSec();

        double deltaP = fpTime - meanFallPredictTime;
        meanFallPredictTime += deltaP / double(nFP);
        meanSquaredFallPredictTime += square(deltaP);
    }

    void writeHeader(ofstream& outputCSV)
    {
        outputCSV << "Scenario Number, initial reaction time, mean reaction time, reaction time variance, n, "
        << "mean balance calc time, balance calc variance, n, "
        << "mean intercept calc time, intercept calc variance, n, "
        << "mean fall prediction time, fall prediction variance, n, "
        << "IK success rate, n, "
        << "mean IK time, IK time variance, n, "
        << "mean IK distance, IK distance variance, n "
        << endl;
    }

public:
    ~MetricsRecorder() {

        // Get the name of the folder to store the result in
        const char* resultsFolder = std::getenv("RESULTS_FOLDER");
        if(resultsFolder == NULL)
        {
            ROS_DEBUG_STREAM("Results folder not set. Using current directory.");
            resultsFolder = "";
        }

        const char* scenarioNumberStr = std::getenv("i");
        if(scenarioNumberStr == NULL)
        {
            ROS_DEBUG_STREAM("Scenario number not set. Using 0.");
            scenarioNumberStr = "0";
        }

        const string resultsFileName = boost::filesystem::current_path().string() + "/" + string(resultsFolder) + "/" + "metrics.csv";
        ROS_DEBUG_STREAM("Using results file: " << resultsFileName);
        bool exists = boost::filesystem::exists(resultsFileName);
        ofstream outputCSV;
        outputCSV.open(resultsFileName.c_str(), ios::out | ios::app);
        assert(outputCSV.is_open());

        if(!exists)
        {
            writeHeader(outputCSV);
        }

        outputCSV << scenarioNumberStr << ", " << initialReactionTime << ", ";

        double reactionTimeVariance = nRT > 1 ? meanSquaredReactionTime / (nRT - 1) : 0;
        outputCSV << meanReactionTime << ", " << reactionTimeVariance  << ", " << nRT << ", ";

        double balanceCalcTimeVariance = nBC > 1 ? meanSquaredBalanceCalcTime / (nBC - 1) : 0;
        outputCSV << meanBalanceCalcTime << ", " << balanceCalcTimeVariance  << ", " << nBC << ", ";

        double interceptCalcTimeVariance = nIC > 1 ? meanSquaredInterceptCalcTime / (nIC - 1) : 0;
        outputCSV << meanInterceptCalcTime << ", " << interceptCalcTimeVariance  << ", " << nIC << ", ";

        double fallPredictTimeVariance = nFP > 1 ? meanSquaredFallPredictTime / (nFP - 1) : 0;
        outputCSV << meanFallPredictTime << ", " << fallPredictTimeVariance  << ", " << nFP << ", ";

        outputCSV << (nIK > 0 ? nSuccessfulIK / double(nIK) : 0.0) << ", " << nIK << ", ";

        double ikTimeVariance = nSuccessfulIK > 1 ? meanSquaredIKTime / (nSuccessfulIK - 1) : 0;
        outputCSV << meanIKTime << ", " << ikTimeVariance  << ", " << nSuccessfulIK << ",";

        double ikDistanceVariance = nSuccessfulIK > 1 ? meanSquaredIKDistance / (nSuccessfulIK - 1) : 0;
        outputCSV << meanIKDistance << ", " << ikDistanceVariance  << ", " << nSuccessfulIK;

        outputCSV << endl;
        outputCSV.close();
        ROS_DEBUG_STREAM("Printing output file complete");
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "metrics_recorder");
    MetricsRecorder mr;
    ros::spin();
    return 0;
}

