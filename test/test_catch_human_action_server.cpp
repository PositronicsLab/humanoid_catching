#include <humanoid_catching/catch_human_action_server.h>
#include <gtest/gtest.h>

using namespace humanoid_catching;

TEST(TestSuite, testZeroNoVelocity)
{
    Limits limit;
    limit.velocity = 1;
    limit.acceleration = 1;
    double result = CatchHumanActionServer::calcJointExecutionTime(limit, 0.0 /* distance */, 0.0 /* initial */);
    EXPECT_DOUBLE_EQ(0.0, result);
}

TEST(TestSuite, testZeroPositiveVelocity)
{
    Limits limit;
    limit.velocity = 1;
    limit.acceleration = 1;
    double result = CatchHumanActionServer::calcJointExecutionTime(limit, 0.0 /* distance */, 1.0 /* initial */);
    EXPECT_DOUBLE_EQ(1.0, result);
}

TEST(TestSuite, testZeroNegativeVelocity)
{
    Limits limit;
    limit.velocity = 1;
    limit.acceleration = 1;
    double result = CatchHumanActionServer::calcJointExecutionTime(limit, 0.0 /* distance */, -0.5 /* initial */);
    EXPECT_DOUBLE_EQ(2 * sqrt(1.0 / 8.0) + 0.5, result);
}

TEST(TestSuite, testTriangular)
{
    Limits limit;
    limit.velocity = 2;
    limit.acceleration = 1;
    double result = CatchHumanActionServer::calcJointExecutionTime(limit, 4.0 /* distance */, 0.0 /* initial */);
    EXPECT_DOUBLE_EQ(4.0, result);
}

TEST(TestSuite, testTriangularPositiveVelocity)
{
    Limits limit;
    limit.velocity = 3;
    limit.acceleration = 2;
    double result = CatchHumanActionServer::calcJointExecutionTime(limit, 1.75 /* distance */, 1.0 /* initial */);
    EXPECT_DOUBLE_EQ(1.5, result);
}

TEST(TestSuite, testTriangularNegativeVelocity)
{
    Limits limit;
    limit.velocity = 3;
    limit.acceleration = 2;
    double result = CatchHumanActionServer::calcJointExecutionTime(limit, 1.75 /* distance */, -1.0 /* initial */);
    EXPECT_DOUBLE_EQ(2.5, result);
}

TEST(TestSuite, testTrapezoidalZeroVelocity)
{
    Limits limit;
    limit.velocity = 0.5;
    limit.acceleration = 0.125;
    double result = CatchHumanActionServer::calcJointExecutionTime(limit, 40.0 /* distance */, 0.0 /* initial */);
    EXPECT_DOUBLE_EQ(8.0 /* triangular */ + 72.0, result);
}

TEST(TestSuite, testTrapezoidalNegativeVelocity)
{
    Limits limit;
    limit.velocity = 0.5;
    limit.acceleration = 0.125;
    double result = CatchHumanActionServer::calcJointExecutionTime(limit, 36.0 /* distance */, -1.0 /* initial */);
    EXPECT_DOUBLE_EQ(8.0 /* deaccel */ + 8.0 /* triangular */ + 72.0, result);
}

TEST(TestSuite, testTrapezoidalPositiveVelocity)
{
    Limits limit;
    limit.velocity = 0.5;
    limit.acceleration = 0.125;
    double result = CatchHumanActionServer::calcJointExecutionTime(limit, 33.0 /* distance */, 0.5 /* initial */);
    EXPECT_DOUBLE_EQ(4.0 /* triangular */ + 64.0, result);
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
