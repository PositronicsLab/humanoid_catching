#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/bodies.h>
#include <geometric_shapes/shape_operations.h>

namespace
{
using namespace std;

class CalcEEBB
{
private:

    //! Node handle
    ros::NodeHandle nh;

    //! Private nh
    ros::NodeHandle pnh;

    //! TF listener
    tf::TransformListener tf;

public:
    CalcEEBB() :
        pnh("~"){}

    void calc() {
        // Allow the tf cache to load
        ros::Duration(1).sleep();

        // Determine the bounding box of the end effectors. Transform to the r_wrist_roll_link frame
        // to provide common frame with the same orientation as the models
        const ros::Time stamp = ros::Time::now();
        const string frames[] = {"r_wrist_roll_link", "r_wrist_roll_link", "r_gripper_l_finger_link", "r_gripper_l_finger_tip_link", "r_gripper_r_finger_link", "r_gripper_l_finger_tip_link"};
        const string stls[] = {"/opt/ros/hydro/share/pr2_description/meshes/gripper_v0/gripper_palm.stl",
        "/opt/ros/hydro/share/pr2_description/meshes/forearm_v0/wrist_roll_L.stl",
        "/opt/ros/hydro/share/pr2_description/meshes/gripper_v0/l_finger.stl",
        "/opt/ros/hydro/share/pr2_description/meshes/gripper_v0/l_finger_tip.stl",
        "/opt/ros/hydro/share/pr2_description/meshes/gripper_v0/l_finger.stl",
        "/opt/ros/hydro/share/pr2_description/meshes/gripper_v0/l_finger_tip.stl" };
        assert(boost::size(frames) == boost::size(stls));

        // Store base frame
        geometry_msgs::Pose base = tfFrameToPose(frames[0], stamp, frames[0]);
        geometry_msgs::Point currMin, currMax = base.position;

        for (unsigned int i = 0; i < boost::size(frames); ++i) {
            if (!tf.waitForTransform(frames[0], frames[i], stamp, ros::Duration(10))) {
                ROS_ERROR("Failed to get transform from %s to %s", frames[0].c_str(), frames[i].c_str());
            }
            geometry_msgs::Pose pose = tfFrameToPose(frames[i], stamp, frames[0]);

            ROS_DEBUG("Position: %f, %f, %f", pose.position.x, pose.position.y, pose.position.z);

            // TODO: Assert orientations are equal
            Eigen::Vector3d dims = computeBoundingBoxOfMesh(stls[i]);
            ROS_DEBUG("Dimensions: %f, %f, %f", dims[0], dims[1], dims[2]);

            currMin.x = min(currMin.x, pose.position.x - dims[0] / 2);
            currMin.y = min(currMin.y, pose.position.y - dims[1] / 2);
            currMin.z = min(currMin.z, pose.position.z - dims[2] / 2);
            currMax.x = max(currMax.x, pose.position.x + dims[0] / 2);
            currMax.y = max(currMax.y, pose.position.y + dims[1] / 2);
            currMax.z = max(currMax.z, pose.position.z + dims[2] / 2);
        }

        ROS_INFO("Dimensions %f %f %f", currMax.x - currMin.x, currMax.y - currMin.y, currMax.z - currMin.z);
    }

private:

    geometry_msgs::Pose tfFrameToPose(const string& tfFrame, const ros::Time& stamp, const string& base = "/map") const
    {
        tf::StampedTransform tfStampedTransform;
        tf.lookupTransform(base, tfFrame, stamp, tfStampedTransform);
        geometry_msgs::TransformStamped stampedTransform;
        transformStampedTFToMsg(tfStampedTransform, stampedTransform);
        geometry_msgs::Pose pose;
        pose.position.x = stampedTransform.transform.translation.x;
        pose.position.y = stampedTransform.transform.translation.y;
        pose.position.z = stampedTransform.transform.translation.z;
        pose.orientation = stampedTransform.transform.rotation;
        return pose;
    }

    Eigen::Vector3d computeBoundingBoxOfMesh(const string& meshFile)
    {
        boost::shared_ptr<shapes::Mesh> mesh(shapes::createMeshFromResource("file://" + meshFile));
        if (mesh.get() == NULL)
        {
            ROS_ERROR("Failed to load mesh '%s'", meshFile.c_str());
            return Eigen::Vector3d();
        }
        ROS_DEBUG("Loaded mesh successfully");

        double maxX = -std::numeric_limits<double>::infinity(), maxY = -std::numeric_limits<double>::infinity(), maxZ = -std::numeric_limits<double>::infinity();
        double minX =  std::numeric_limits<double>::infinity(), minY =  std::numeric_limits<double>::infinity(), minZ  = std::numeric_limits<double>::infinity();

        for(unsigned int i = 0; i < mesh->vertex_count ; ++i)
        {
            double vx = mesh->vertices[3 * i    ];
            double vy = mesh->vertices[3 * i + 1];
            double vz = mesh->vertices[3 * i + 2];

            if (maxX < vx) maxX = vx;
            if (maxY < vy) maxY = vy;
            if (maxZ < vz) maxZ = vz;

            if (minX > vx) minX = vx;
            if (minY > vy) minY = vy;
            if (minZ > vz) minZ = vz;
        }

        if (maxX < minX) maxX = minX = 0.0;
        if (maxY < minY) maxY = minY = 0.0;
        if (maxZ < minZ) maxZ = minZ = 0.0;

        return Eigen::Vector3d(maxX - minX, maxY - minY, maxZ - minZ);
    }
};
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "calc_ee_bb");
    CalcEEBB eebb;
    eebb.calc();
}
