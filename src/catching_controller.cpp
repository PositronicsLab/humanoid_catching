#include <ros/ros.h>
#include <human_catching/HumanFall.h>
#include <message_filters/subscriber.h>
#include <geometry_msgs/TwistStamped.h>

namespace {
using namespace std;
using namespace human_catching;

class CatchingController {
private:
    
	//! Node handle
	ros::NodeHandle nh;

	//! Private nh
	ros::NodeHandle pnh;
    
    //! Human fall subscriber
    auto_ptr<message_filters::Subscriber<HumanFall> > humanFallSub;
    
    //! Human twist subscriber
    auto_ptr<message_filters::Subscriber<geometry_msgs::TwistStamped> > humanTwistSub;
public:
	CatchingController() :
		pnh("~") {
        ROS_INFO("Initializing the catching controller");
        humanFallSub.reset(
                new message_filters::Subscriber<HumanFall>(nh, "/human/fall", 1));
        humanFallSub->registerCallback(boost::bind(&CatchingController::fallDetected, this, _1));
        
        // Construct but don't initialize
        humanTwistSub.reset(
                new message_filters::Subscriber<geometry_msgs::TwistStamped>(nh, "/human/twist", 1));
        ROS_INFO("Catching controller initialized successfully");
	}

private:
    void fallDetected(const HumanFallConstPtr& fallingMsg) {
        ROS_INFO("Human fall detected at @ %f", fallingMsg->header.stamp.toSec());
        
        // Unsubscribe from further fall notifications
        humanFallSub->unsubscribe();
        
        // Begin listening for twist notifications
        humanTwistSub->registerCallback(boost::bind(&CatchingController::twistDetected, this, _1));
        
        // Wait to receive a twist notification to take action so we are
        // aware of any initial velocity
    }
    
    void twistDetected(const geometry_msgs::TwistStampedConstPtr& twistMsg) {
        ROS_INFO("Human twist detected at @ %f", twistMsg->header.stamp.toSec());
        
        // TODO: Magic here
        // TODO: Project catch position
        // TODO: Move arms to catch position
        // TODO: Wait for contact or reaching target
        // TODO: Activate the force control action
    }
};
}
int main(int argc, char** argv) {
	ros::init(argc, argv, "catching_controller");

	CatchingController cc;
	ros::spin();
}
