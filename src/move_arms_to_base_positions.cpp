#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <boost/math/constants/constants.hpp>

namespace {
    using namespace std;
    using namespace ros;

    class MoveArmsToBasePositions {
        private:
            NodeHandle nh;
            move_group_interface::MoveGroup rightArm;
            move_group_interface::MoveGroup leftArm;
        public:
            MoveArmsToBasePositions():rightArm("right_arm"), leftArm("left_arm"){
        }

        void move(){
            ROS_INFO("Moving arms to base position");
            vector<double> positions(7);

            ROS_INFO("Moving right arm to base position");
            positions[0] = 0;
            positions[1] = 0.0103108212;
            positions[2] = 0;
            positions[3] = -0.2939007067;
            positions[4] = 0;
            positions[5] = -0.4135609944;
            positions[6] = 0;
            rightArm.setJointValueTarget(positions);
            rightArm.move();

            ROS_INFO("Moving left arm to base position");
            positions[0] = 0;
            positions[1] = 0.0102741812;
            positions[2] = 0;
            positions[3] = -0.2930562803;
            positions[4] = 0;
            positions[5] = -0.4147517898;
            positions[6] = 0;
            leftArm.setJointValueTarget(positions);
            leftArm.move();
            ROS_INFO("Moving arms complete");
        }
    };
}

int main(int argc, char** argv){
  ros::init(argc, argv, "move_arms_to_base_positions");
  MoveArmsToBasePositions ma;
  ma.move();
}
