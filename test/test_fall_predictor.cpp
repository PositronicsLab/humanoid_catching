#include <gtest/gtest.h>
#include <ros/ros.h>
#include <humanoid_catching/PredictFall.h>
#include <boost/math/constants/constants.hpp>
#include <tf/transform_listener.h>
#include <geometry_msgs/Quaternion.h>

static const double pi = boost::math::constants::pi<double>();

static humanoid_catching::PredictFall callFallPredictor(const geometry_msgs::Quaternion& orientation,
                                                        const geometry_msgs::Vector3& angularVelocity) {
    int argc = 0;
    ros::init(argc, NULL, "test_fall_predictor");
    ros::NodeHandle nh;
    ros::ServiceClient fallPredictor;
    fallPredictor = nh.serviceClient<humanoid_catching::PredictFall>("/fall_predictor/predict_fall", true /* persistent */);
    humanoid_catching::PredictFall predictFall;
    predictFall.request.header.frame_id = "/odom_combined";
    predictFall.request.header.stamp = ros::Time::now();
    predictFall.request.orientation = orientation;
    predictFall.request.velocity.angular = angularVelocity;
    predictFall.request.max_time = ros::Duration(5.0);
    predictFall.request.step_size = ros::Duration(0.01);

    predictFall.request.visualize = false;

    bool result = fallPredictor.call(predictFall);
    EXPECT_TRUE(result);
    return predictFall;
}

TEST(TestSuite, testVerticalPosition)
{
    geometry_msgs::Quaternion orientation = tf::createQuaternionMsgFromRollPitchYaw(0, -pi / 2.0, 0);
    geometry_msgs::Vector3 velocity;
    humanoid_catching::PredictFall fall = callFallPredictor(orientation, velocity);

    // Initial orientation should be the same as passed in
    EXPECT_DOUBLE_EQ(orientation.x, fall.response.initial.orientation.x);
    EXPECT_DOUBLE_EQ(orientation.y, fall.response.initial.orientation.y);
    EXPECT_DOUBLE_EQ(orientation.z, fall.response.initial.orientation.z);
    EXPECT_DOUBLE_EQ(orientation.w, fall.response.initial.orientation.w);

    // Initial position should be base, base + height / 2.0, base
    EXPECT_DOUBLE_EQ(fall.response.base.x, fall.response.initial.position.x);
    EXPECT_DOUBLE_EQ(fall.response.base.y, fall.response.initial.position.y);
    EXPECT_DOUBLE_EQ(fall.response.base.z + fall.response.height / 2.0, fall.response.initial.position.z);

    // Check that the results are the correct size
    EXPECT_EQ(501, fall.response.points.size());

    // Check that the first point has the correct pose
    // Initial orientation should be the same as passed in
    EXPECT_NEAR(orientation.x, fall.response.points[0].pose.orientation.x, 0.0001);
    EXPECT_NEAR(orientation.y, fall.response.points[0].pose.orientation.y, 0.0001);
    EXPECT_NEAR(orientation.z, fall.response.points[0].pose.orientation.z, 0.0001);
    EXPECT_NEAR(orientation.w, fall.response.points[0].pose.orientation.w, 0.0001);

    // Initial position should be base, base + height / 2.0, base
    EXPECT_NEAR(fall.response.base.x, fall.response.points[0].pose.position.x, 0.0001);
    EXPECT_NEAR(fall.response.base.y, fall.response.points[0].pose.position.y, 0.0001);
    EXPECT_NEAR(fall.response.base.z + fall.response.height / 2.0, fall.response.points[0].pose.position.z, 0.0001);

    // Final orientation should be the same as passed in
    EXPECT_NEAR(orientation.x, fall.response.points.back().pose.orientation.x, 0.0001);
    EXPECT_NEAR(orientation.y, fall.response.points.back().pose.orientation.y, 0.0001);
    EXPECT_NEAR(orientation.z, fall.response.points.back().pose.orientation.z, 0.0001);
    EXPECT_NEAR(orientation.w, fall.response.points.back().pose.orientation.w, 0.0001);

    // Initial position should be base, base + height / 2.0, base
    EXPECT_NEAR(fall.response.base.x, fall.response.points.back().pose.position.x, 0.0001);
    EXPECT_NEAR(fall.response.base.y, fall.response.points.back().pose.position.y, 0.0001);
    EXPECT_NEAR(fall.response.base.z + fall.response.height / 2.0, fall.response.points.back().pose.position.z, 0.0001);
}

TEST(TestSuite, testHorizonalPosition)
{
    geometry_msgs::Quaternion orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, 0.0);
    geometry_msgs::Vector3 velocity;
    humanoid_catching::PredictFall fall = callFallPredictor(orientation, velocity);

    // Initial orientation should be the same as passed in
    EXPECT_DOUBLE_EQ(orientation.x, fall.response.initial.orientation.x);
    EXPECT_DOUBLE_EQ(orientation.y, fall.response.initial.orientation.y);
    EXPECT_DOUBLE_EQ(orientation.z, fall.response.initial.orientation.z);
    EXPECT_DOUBLE_EQ(orientation.w, fall.response.initial.orientation.w);

    // Initial position should be base + height / 2.0, base, base
    EXPECT_DOUBLE_EQ(fall.response.base.x + fall.response.height / 2.0, fall.response.initial.position.x);
    EXPECT_DOUBLE_EQ(fall.response.base.y, fall.response.initial.position.y);
    EXPECT_DOUBLE_EQ(fall.response.base.z, fall.response.initial.position.z);

    // Check that the results are the correct size
    EXPECT_EQ(1, fall.response.points.size());

    // Check that the first point has the correct pose
    // Initial orientation should be the same as passed in
    EXPECT_NEAR(orientation.x, fall.response.points[0].pose.orientation.x, 0.01);
    EXPECT_NEAR(orientation.y, fall.response.points[0].pose.orientation.y, 0.01);
    EXPECT_NEAR(orientation.z, fall.response.points[0].pose.orientation.z, 0.01);
    EXPECT_NEAR(orientation.w, fall.response.points[0].pose.orientation.w, 0.01);

    // Initial position should be base + height / 2.0, base, base
    EXPECT_NEAR(fall.response.base.x + fall.response.height / 2.0, fall.response.points[0].pose.position.x, 0.01);
    EXPECT_NEAR(fall.response.base.y, fall.response.points[0].pose.position.y, 0.01);
    EXPECT_NEAR(fall.response.base.z, fall.response.points[0].pose.position.z, 0.01);

    // Final orientation should be the same as passed in
    EXPECT_NEAR(orientation.x, fall.response.points.back().pose.orientation.x, 0.01);
    EXPECT_NEAR(orientation.y, fall.response.points.back().pose.orientation.y, 0.01);
    EXPECT_NEAR(orientation.z, fall.response.points.back().pose.orientation.z, 0.01);
    EXPECT_NEAR(orientation.w, fall.response.points.back().pose.orientation.w, 0.01);

    // Final position should be base + height / 2.0, base, base
    EXPECT_NEAR(fall.response.base.x + fall.response.height / 2.0, fall.response.points.back().pose.position.x, 0.01);
    EXPECT_NEAR(fall.response.base.y, fall.response.points.back().pose.position.y, 0.01);
    EXPECT_NEAR(fall.response.base.z, fall.response.points.back().pose.position.z, 0.01);
}

TEST(TestSuite, testPitch)
{
    geometry_msgs::Quaternion orientation = tf::createQuaternionMsgFromRollPitchYaw(0, -pi / 2.0, 0);
    geometry_msgs::Vector3 velocity;
    velocity.y = 2.0;

    humanoid_catching::PredictFall fall = callFallPredictor(orientation, velocity);

    // Initial orientation should be the same as passed in
    EXPECT_DOUBLE_EQ(orientation.x, fall.response.initial.orientation.x);
    EXPECT_DOUBLE_EQ(orientation.y, fall.response.initial.orientation.y);
    EXPECT_DOUBLE_EQ(orientation.z, fall.response.initial.orientation.z);
    EXPECT_DOUBLE_EQ(orientation.w, fall.response.initial.orientation.w);


    // Angular velocity should be the same
    EXPECT_DOUBLE_EQ(0.0, fall.response.initial_velocity.angular.x);
    EXPECT_DOUBLE_EQ(2.0, fall.response.initial_velocity.angular.y);
    EXPECT_DOUBLE_EQ(0.0, fall.response.initial_velocity.angular.z);

    // Linear velocity should be positive in the X direction and zero in others
    EXPECT_TRUE(fall.response.initial_velocity.linear.x > 0);
    EXPECT_NEAR(0, fall.response.initial_velocity.linear.y, 0.0001);
    EXPECT_NEAR(0, fall.response.initial_velocity.linear.z, 0.0001);

    // Angular velocity should be the same
    EXPECT_NEAR(0.0, fall.response.points[0].velocity.angular.x, 0.0001);
    EXPECT_NEAR(2.0, fall.response.points[0].velocity.angular.y, 0.0001);
    EXPECT_NEAR(0.0, fall.response.points[0].velocity.angular.z, 0.0001);

    // Linear velocity should be positive in the X direction and zero in others
    EXPECT_TRUE(fall.response.points[0].velocity.linear.x > 0);
    EXPECT_NEAR(0, fall.response.points[0].velocity.linear.y, 0.0001);
    EXPECT_NEAR(0, fall.response.points[0].velocity.linear.z, 0.0001);

    // Determine it is properly constrainted
    for (unsigned int i = 0; i < fall.response.points.size(); ++i) {
        EXPECT_NEAR(0, fall.response.points[i].pose.position.y, 0.0001);
        EXPECT_NEAR(fall.response.height / 2.0, sqrt(pow(fall.response.points[i].pose.position.x - fall.response.base.x, 2)
                    + pow(fall.response.points[i].pose.position.z, 2)), 0.01);
    }

    // Check that z velocity is continually increasing negatively
    for (unsigned int i = 1; i < fall.response.points.size(); ++i) {
        EXPECT_NEAR(0, fall.response.points[i].velocity.angular.x, 0.0001);
        EXPECT_NEAR(0, fall.response.points[i].velocity.angular.z, 0.0001);
        EXPECT_NEAR(0, fall.response.points[i].velocity.linear.y, 0.0001);

        EXPECT_TRUE(fall.response.points[i].velocity.linear.x > 0);
        EXPECT_TRUE(fall.response.points[i].velocity.angular.y > 0);
        EXPECT_TRUE(fall.response.points[i].velocity.linear.z < 0);
        EXPECT_TRUE(fall.response.points[i].velocity.linear.z <= fall.response.points[i - 1].velocity.linear.z);
    }

    // Determine that it fell straight forward
    // X should be height over 2
    EXPECT_NEAR(fall.response.height / 2.0 + fall.response.base.x, fall.response.points.back().pose.position.x, 0.01);
    EXPECT_NEAR(0, fall.response.points.back().pose.position.z, 0.01);
    EXPECT_NEAR(0, fall.response.points.back().pose.position.y, 0.0001);

    geometry_msgs::Quaternion horiz = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);
    EXPECT_NEAR(fall.response.points.back().pose.orientation.x, horiz.x, 0.01);
    EXPECT_NEAR(fall.response.points.back().pose.orientation.y, horiz.y, 0.01);
    EXPECT_NEAR(fall.response.points.back().pose.orientation.z, horiz.z, 0.01);
    EXPECT_NEAR(fall.response.points.back().pose.orientation.w, horiz.w, 0.01);
}

TEST(TestSuite, testRoll)
{
    geometry_msgs::Quaternion orientation = tf::createQuaternionMsgFromRollPitchYaw(0, -pi / 2.0, 0);
    geometry_msgs::Vector3 velocity;
    velocity.x = 2.0;

    humanoid_catching::PredictFall fall = callFallPredictor(orientation, velocity);

    // Initial orientation should be the same as passed in
    EXPECT_DOUBLE_EQ(orientation.x, fall.response.initial.orientation.x);
    EXPECT_DOUBLE_EQ(orientation.y, fall.response.initial.orientation.y);
    EXPECT_DOUBLE_EQ(orientation.z, fall.response.initial.orientation.z);
    EXPECT_DOUBLE_EQ(orientation.w, fall.response.initial.orientation.w);


    // Angular velocity should be the same
    EXPECT_DOUBLE_EQ(2.0, fall.response.initial_velocity.angular.x);
    EXPECT_DOUBLE_EQ(0.0, fall.response.initial_velocity.angular.y);
    EXPECT_DOUBLE_EQ(0.0, fall.response.initial_velocity.angular.z);

    // Linear velocity should be positive in the Y direction and zero in others
    EXPECT_TRUE(fall.response.initial_velocity.linear.y < 0);
    EXPECT_NEAR(0, fall.response.initial_velocity.linear.x, 0.0001);
    EXPECT_NEAR(0, fall.response.initial_velocity.linear.z, 0.0001);

    // Angular velocity should be the same
    EXPECT_NEAR(2.0, fall.response.points[0].velocity.angular.x, 0.0001);
    EXPECT_NEAR(0.0, fall.response.points[0].velocity.angular.y, 0.0001);
    EXPECT_NEAR(0.0, fall.response.points[0].velocity.angular.z, 0.0001);

    // Linear velocity should be positive in the Y direction and zero in others
    EXPECT_NEAR(0, fall.response.points[0].velocity.linear.x, 0.0001);
    EXPECT_TRUE(fall.response.points[0].velocity.linear.y < 0);
    EXPECT_NEAR(0, fall.response.points[0].velocity.linear.z, 0.0001);

    // Determine it is properly constrainted
    for (unsigned int i = 0; i < fall.response.points.size(); ++i) {
        EXPECT_NEAR(fall.response.base.x, fall.response.points[i].pose.position.x, 0.0001);
        EXPECT_NEAR(fall.response.height / 2.0, sqrt(pow(fall.response.points[i].pose.position.y - fall.response.base.y, 2)
                    + pow(fall.response.points[i].pose.position.z, 2)), 0.01);
    }

    // Check that z velocity is continually increasing negatively
    for (unsigned int i = 1; i < fall.response.points.size(); ++i) {
        EXPECT_NEAR(0, fall.response.points[i].velocity.angular.y, 0.0001);
        EXPECT_NEAR(0, fall.response.points[i].velocity.angular.z, 0.0001);
        EXPECT_NEAR(0, fall.response.points[i].velocity.linear.x, 0.0001);

        EXPECT_TRUE(fall.response.points[i].velocity.linear.y < 0);
        EXPECT_TRUE(fall.response.points[i].velocity.angular.x > 0);
        EXPECT_TRUE(fall.response.points[i].velocity.linear.z < 0);
        EXPECT_TRUE(fall.response.points[i].velocity.linear.z <= fall.response.points[i - 1].velocity.linear.z);
    }

    // Determine that it fell straight sideways
    // Y should be height over 2
    EXPECT_NEAR(fall.response.base.x, fall.response.points.back().pose.position.x, 0.0001);
    EXPECT_NEAR(-fall.response.height / 2.0 + fall.response.base.y, fall.response.points.back().pose.position.y, 0.01);
    EXPECT_NEAR(0, fall.response.points.back().pose.position.z, 0.01);

    geometry_msgs::Quaternion horiz = tf::createQuaternionMsgFromRollPitchYaw(-pi  / 2.0, -pi  / 2.0, 0);
    EXPECT_NEAR(fall.response.points.back().pose.orientation.x, horiz.x, 0.01);
    EXPECT_NEAR(fall.response.points.back().pose.orientation.y, horiz.y, 0.01);
    EXPECT_NEAR(fall.response.points.back().pose.orientation.z, horiz.z, 0.01);
    EXPECT_NEAR(fall.response.points.back().pose.orientation.w, horiz.w, 0.01);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
