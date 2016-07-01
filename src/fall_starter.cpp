#include <ros/ros.h>
#include <humanoid_catching/HumanFall.h>
#include <gazebo_msgs/ApplyBodyWrench.h>
#include <geometry_msgs/PoseStamped.h>
#include <boost/random.hpp>
#define USE_FIXED_SEED 1

namespace
{

using namespace std;
using namespace ros;

static const double NOTIFICATION_DELAY_DEFAULT = 0.1;
static const double DURATION_DEFAULT = 0.001;
static const unsigned int FIXED_SEED = 0;

class FallStarter
{
private:

    //! Node handle
    ros::NodeHandle nh;

    //! Private nh
    ros::NodeHandle pnh;

    //! Gazebo client
    ros::ServiceClient gazeboClient;

    //! Catching controller notifier
    ros::Publisher catchNotifier;

    //! X torque
    double x;

    //! Y torque
    double y;

    //! Z torque
    double z;

    //! Torque Duration
    double duration;

    //! Set random values for x-y
    bool random;

    //! Do not wait for robot to come online
    bool waitForBalancer;

    //! Notify only
    bool notifyOnly;

    //! Notification delay
    double delay;

    //! rng
    boost::mt19937 rng;
public:
    FallStarter() :
        pnh("~")
    {
        gazeboClient = nh.serviceClient<gazebo_msgs::ApplyBodyWrench>("/gazebo/apply_body_wrench", true /* persistent */);
        catchNotifier = nh.advertise<humanoid_catching::HumanFall>("/human/fall", 1, true);
        pnh.param("x", x, 0.0);
        pnh.param("y", y, 0.0);
        pnh.param("z", z, 0.0);
        pnh.param("duration", duration, DURATION_DEFAULT);
        pnh.param("delay", delay, NOTIFICATION_DELAY_DEFAULT);
        pnh.param("waitForBalancer", waitForBalancer, true);
        pnh.param("random", random, false);
        pnh.param("notifyOnly", notifyOnly, false);

        if (random && notifyOnly)
        {
            ROS_WARN("Cannot combine random and notify only settings");
            random = false;
        }

        if (waitForBalancer)
        {
            ros::service::waitForService("/balancer/torques");
        }

        if (random)
        {
            unsigned int seed = 0;
#if(!USE_FIXED_SEED)
            seed = static_cast<unsigned int>(std::time(NULL));
#else
            const char* scenarioNumberStr = std::getenv("i");
            unsigned int scenarioNumber = 0;
            if(scenarioNumberStr != NULL)
            {
                scenarioNumber = boost::lexical_cast<unsigned int>(scenarioNumberStr);
                seed = scenarioNumber + 1000;
            }
            else
            {
                cout << "No scenario number set. Using 0" << endl;
                seed = FIXED_SEED;
            }
#endif
            rng.seed(seed);


            bool found = false;
            while (!found)
            {
                // X generator
                boost::uniform_real<double> xRange(0, 200);
                boost::variate_generator<boost::mt19937&, boost::uniform_real<double> > genX(rng, xRange);
                x = genX();

                // Y generator
                boost::uniform_real<double> yRange(-150, 150);
                boost::variate_generator<boost::mt19937&, boost::uniform_real<double> > genY(rng, yRange);
                y = genY();
                if (fabs(x) >= 100 || fabs(y) >= 100)
                {
                    found = true;
                }
            }
        }
    }

    void fall()
    {
        if (!notifyOnly)
        {
            ROS_INFO("Initiating fall with forces [%f, %f, %f]", x, y, z);
            gazebo_msgs::ApplyBodyWrench wrench;
            wrench.request.body_name = "human::link";
            wrench.request.wrench.torque.x = x;
            wrench.request.wrench.torque.y = y;
            wrench.request.wrench.torque.z = z;
            wrench.request.duration = ros::Duration(duration);

            if (!gazeboClient.call(wrench))
            {
                ROS_ERROR("Failed to apply wrench");
                return;
            }
        }

        ROS_DEBUG("Notifying fall catcher");
        catchNotifier.publish(humanoid_catching::HumanFall());
        ROS_DEBUG("Fall initiation complete");
    }
};
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "fall_starter");
    FallStarter fs;
    fs.fall();
    ros::Duration(3.0).sleep();
}
