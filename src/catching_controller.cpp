#include <ros/ros.h>
#include <human_catching/HumanFall.h>

namespace {
using namespace std;
using namespace human_catching;

class CatchingController {
private:
    
	//! Node handle
	ros::NodeHandle nh;

	//! Private nh
	ros::NodeHandle pnh;
public:
	CatchingController() :
		pnh("~") {
	}

private:
};
}
int main(int argc, char** argv) {
	ros::init(argc, argv, "catching_controller");

	CatchingController cc;
	ros::spin();
}
