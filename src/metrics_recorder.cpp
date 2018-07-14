#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <humanoid_catching/DurationStamped.h>
#include <iostream>
#include <fstream>
#include <boost/filesystem.hpp>
#include <humanoid_catching/IKMetric.h>
#include <map>

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
    unsigned int nBCStarts;

    ros::Time balancingStart;
    ros::Duration maxBalancingTime;
    double meanBalancingTime;
    double meanSquaredBalancingTime;

    double meanInterceptCalcTime;
    double meanSquaredInterceptCalcTime;
    unsigned int nIC;
    unsigned int nICStarts;

    double meanFallPredictTime;
    double meanSquaredFallPredictTime;
    unsigned int nFP;

    double meanMovementPlanTime;
    double meanSquaredMovementPlanTime;
    unsigned int nMP;

    double meanMovementExecutionTime;
    double meanSquaredMovementExecutionTime;
    unsigned int nME;

    double meanIKTime;
    double meanSquaredIKTime;
    double meanIKCount;
    double meanSquaredIKCount;
    unsigned int nIK;
    unsigned int nSuccessfulIK;

    auto_ptr<message_filters::Subscriber<humanoid_catching::DurationStamped> > reactionTimeSub;
    auto_ptr<message_filters::Subscriber<humanoid_catching::DurationStamped> > balanceCalcTimeSub;
    auto_ptr<message_filters::Subscriber<humanoid_catching::DurationStamped> > interceptCalcTimeSub;
    auto_ptr<message_filters::Subscriber<humanoid_catching::DurationStamped> > fallPredictTimeSub;
    auto_ptr<message_filters::Subscriber<humanoid_catching::DurationStamped> > movePlanTimeSub;
    auto_ptr<message_filters::Subscriber<humanoid_catching::DurationStamped> > moveExecuteTimeSub;
    auto_ptr<message_filters::Subscriber<humanoid_catching::IKMetric> > ikMetricSub;

    enum Mode {
        Initial,
        Intercepting,
        Balancing
    };
    map<string, Mode> controllerMode;

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
            nBCStarts(0),
            meanBalancingTime(0),
            meanSquaredBalancingTime(0),
            meanInterceptCalcTime(0),
            meanSquaredInterceptCalcTime(0),
            nIC(0),
            nICStarts(0),
            meanFallPredictTime(0),
            meanSquaredFallPredictTime(0),
            nFP(0),
            meanMovementPlanTime(0),
            meanSquaredMovementPlanTime(0),
            nMP(0),
            meanMovementExecutionTime(0),
            meanSquaredMovementExecutionTime(0),
            nME(0),
            meanIKTime(0),
            meanSquaredIKTime(0),
            meanIKCount(0),
            meanSquaredIKCount(0),
            nIK(0),
            nSuccessfulIK(0)
             {

        // Setup the subscribers
        reactionTimeSub.reset(
                new message_filters::Subscriber<humanoid_catching::DurationStamped>(nh,
                        "reaction_time", 1));
        reactionTimeSub->registerCallback(boost::bind(&MetricsRecorder::reactionTimeCallback, this, _1));

        balanceCalcTimeSub.reset(
                new message_filters::Subscriber<humanoid_catching::DurationStamped>(nh,
                        "balance_calc_time", 50000));
        balanceCalcTimeSub->registerCallback(boost::bind(&MetricsRecorder::balanceCalcCallback, this, _1));

        interceptCalcTimeSub.reset(
                new message_filters::Subscriber<humanoid_catching::DurationStamped>(nh,
                        "intercept_calc_time", 50000));
        interceptCalcTimeSub->registerCallback(boost::bind(&MetricsRecorder::interceptCalcCallback, this, _1));

        fallPredictTimeSub.reset(
                new message_filters::Subscriber<humanoid_catching::DurationStamped>(nh,
                        "fall_predict_time", 50000));
        fallPredictTimeSub->registerCallback(boost::bind(&MetricsRecorder::fallPredictCalcCallback, this, _1));

        movePlanTimeSub.reset(
                new message_filters::Subscriber<humanoid_catching::DurationStamped>(nh,
                        "movement_plan_metric", 50000));
        movePlanTimeSub->registerCallback(boost::bind(&MetricsRecorder::movementPlanCallback, this, _1));

        moveExecuteTimeSub.reset(
                new message_filters::Subscriber<humanoid_catching::DurationStamped>(nh,
                        "movement_execute_metric", 50000));
        moveExecuteTimeSub->registerCallback(boost::bind(&MetricsRecorder::movementExecuteCallback, this, _1));

        ikMetricSub.reset(
                new message_filters::Subscriber<humanoid_catching::IKMetric>(nh,
                        "ik_metric", 100000));
        ikMetricSub->registerCallback(boost::bind(&MetricsRecorder::ikMetricCallback, this, _1));

        controllerMode["left_arm"] = Initial;
        controllerMode["right_arm"] = Initial;
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

            unsigned int ikCount = metric->count;
            double deltaPD = ikCount - meanIKCount;
            meanIKCount += deltaPD / double(nSuccessfulIK);
            meanSquaredIKCount += square(deltaPD);
        }
    }

    void reactionTimeCallback(const humanoid_catching::DurationStampedConstPtr& duration) {

        if (nRT == 0) {
            initialReactionTime = duration->duration.toSec();
        }

        // Increase number of samples
        nRT++;

        double reactionTime = duration->duration.toSec();

        double deltaP = reactionTime - meanReactionTime;
        meanReactionTime += deltaP / double(nRT);
        meanSquaredReactionTime += square(deltaP);
    }

    void balanceCalcCallback(const humanoid_catching::DurationStampedConstPtr& duration) {

        // Increase number of samples
        nBC++;

        // If the current arm mode is intercepting, increment the number of balancing starts
        if (controllerMode.at(duration->arm) != Balancing) {
            nBCStarts++;
            controllerMode[duration->arm] = Balancing;
            balancingStart = duration->header.stamp;
        }

        double bcTime = duration->duration.toSec();

        double deltaP = bcTime - meanBalanceCalcTime;
        meanBalanceCalcTime += deltaP / double(nBC);
        meanSquaredBalanceCalcTime += square(deltaP);
    }

    void interceptCalcCallback(const humanoid_catching::DurationStampedConstPtr& duration) {

        // Increase number of samples
        nIC++;

        // If the current arm mode is not intercepting, increment the number of intercept starts
        if (controllerMode.at(duration->arm) != Intercepting) {

            if (controllerMode.at(duration->arm) == Balancing) {
                ros::Duration balancingTime = (duration->header.stamp - balancingStart);
                maxBalancingTime = max(balancingTime, maxBalancingTime);
                double deltaP = balancingTime.toSec() - meanBalancingTime;
                meanBalancingTime += deltaP / double(nBCStarts);
                meanSquaredBalancingTime += square(deltaP);
            }

            nICStarts++;
            controllerMode[duration->arm] = Intercepting;
        }

        double icTime = duration->duration.toSec();

        double deltaP = icTime - meanInterceptCalcTime;
        meanInterceptCalcTime += deltaP / double(nIC);
        meanSquaredInterceptCalcTime += square(deltaP);
    }

    void fallPredictCalcCallback(const humanoid_catching::DurationStampedConstPtr& duration) {

        // Increase number of samples
        nFP++;

        double fpTime = duration->duration.toSec();

        double deltaP = fpTime - meanFallPredictTime;
        meanFallPredictTime += deltaP / double(nFP);
        meanSquaredFallPredictTime += square(deltaP);
    }

    void movementPlanCallback(const humanoid_catching::DurationStampedConstPtr& duration) {

        // Increase number of samples
        nMP++;

        double mpTime = duration->duration.toSec();

        double deltaP = mpTime - meanMovementPlanTime;
        meanMovementPlanTime += deltaP / double(nMP);
        meanSquaredMovementPlanTime += square(deltaP);
    }

    void movementExecuteCallback(const humanoid_catching::DurationStampedConstPtr& duration) {

        // Increase number of samples
        nME++;

        double meTime = duration->duration.toSec();

        double deltaP = meTime - meanMovementExecutionTime;
        meanMovementExecutionTime += deltaP / double(nME);
        meanSquaredMovementExecutionTime += square(deltaP);
    }

    void writeHeader(ofstream& outputCSV)
    {
        outputCSV << "Scenario Number, initial reaction time, mean reaction time, reaction time variance, n, "
        << "mean balance calc time, balance calc variance, n, n-starts, "
        << "mean balancing time, balancing time variance, maximum balancing time, "
        << "mean intercept calc time, intercept calc variance, n, n-starts, "
        << "mean fall prediction time, fall prediction variance, n, "
        << "mean movement planning time, movement planning variance, n, "
        << "mean movement execution time, mean movement execution variance, n, "
        << "IK success rate, n, "
        << "mean IK time, IK time variance, n, "
        << "mean IK count, IK count variance, n "
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
        outputCSV << meanBalanceCalcTime << ", " << balanceCalcTimeVariance  << ", " << nBC << ", " << nBCStarts << ", ";

        double balancingTimeVariance = nBCStarts > 1 ? meanSquaredBalancingTime / (nBCStarts - 1) : 0;
        outputCSV << meanBalancingTime << ", " << balancingTimeVariance  << ", " << maxBalancingTime.toSec() << ", ";

        double interceptCalcTimeVariance = nIC > 1 ? meanSquaredInterceptCalcTime / (nIC - 1) : 0;
        outputCSV << meanInterceptCalcTime << ", " << interceptCalcTimeVariance  << ", " << nIC << ", " << nICStarts << ", ";

        double fallPredictTimeVariance = nFP > 1 ? meanSquaredFallPredictTime / (nFP - 1) : 0;
        outputCSV << meanFallPredictTime << ", " << fallPredictTimeVariance  << ", " << nFP << ", ";

        double movementPlanTimeVariance = nMP > 1 ? meanSquaredMovementPlanTime / (nMP - 1) : 0;
        outputCSV << meanMovementPlanTime << ", " << movementPlanTimeVariance  << ", " << nMP << ", ";

        double movementExecutionTimeVariance = nME > 1 ? meanSquaredMovementExecutionTime / (nME - 1) : 0;
        outputCSV << meanMovementExecutionTime << ", " << movementExecutionTimeVariance  << ", " << nME << ", ";

        outputCSV << (nIK > 0 ? nSuccessfulIK / double(nIK) : 0.0) << ", " << nIK << ", ";

        double ikTimeVariance = nSuccessfulIK > 1 ? meanSquaredIKTime / (nSuccessfulIK - 1) : 0;
        outputCSV << meanIKTime << ", " << ikTimeVariance  << ", " << nSuccessfulIK << ", ";

        double ikCountVariance = nSuccessfulIK > 1 ? meanSquaredIKCount / (nSuccessfulIK - 1) : 0;
        outputCSV << meanIKCount << ", " << ikCountVariance  << ", " << nSuccessfulIK;

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

