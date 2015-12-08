#include <ros/ros.h>
#include <human_catching/PredictFall.h>
#include <visualization_msgs/Marker.h>
#include <ode/ode.h>
#include <iostream>

namespace {
using namespace std;

//! Default weight in kg
static const double MASS_DEFAULT = 50;
static const double HEIGHT_DEFAULT = 1.5;

static const double STEP_SIZE = 0.01;
static const double DURATION = 2.0;
static const unsigned int MAX_CONTACTS = 3;

struct Humanoid {
    dBodyID body;  // the dynamics body
    dGeomID geom[1];  // geometries representing this body
};

class FallPredictor {
private:
    //! Publisher for the fall visualization
    ros::Publisher fallVizPub;

    //! Node handle
    ros::NodeHandle nh;

    //! Private nh
    ros::NodeHandle pnh;

    //! Fall prediction Service
    ros::ServiceServer fallPredictionService;

    //! Height of the center of mass
    double humanoidHeight;

    //! Mass of the humanoid
    double humanoidMass;

    //! Ground plane
    dGeomID ground;

    //! Simulation space
    dSpaceID space;

    //! Ground link for joint
    dBodyID groundLink;

    //! World
    dWorldID world;

    //! Joint group for collisions
    dJointGroupID contactgroup;

    //! Joint group for joint between ground and humanoid
    dJointGroupID groundToBodyJG;

    //! Joint between ground and humanoid
    dJointID groundJoint;

    //! Humanoid
    Humanoid object;
public:
   FallPredictor() :
     pnh("~") {
       pnh.param("humanoid_height", humanoidHeight, HEIGHT_DEFAULT);
       pnh.param("humanoid_mass", humanoidMass, MASS_DEFAULT);

       fallVizPub = nh.advertise<visualization_msgs::Marker>(
         "/fall_predictor/projected_path", 1);

       fallPredictionService = nh.advertiseService("/fall_predictor/predict_fall",
            &FallPredictor::predict, this);
	}

private:
    void collisionCallback(dGeomID o1, dGeomID o2) {
        dBodyID b1 = dGeomGetBody(o1);
        dBodyID b2 = dGeomGetBody(o2);
        dContact contact[MAX_CONTACTS];
        for (int i = 0; i < MAX_CONTACTS; i++){
            contact[i].surface.mode = dContactBounce | dContactSoftCFM;
            contact[i].surface.mu = dInfinity;
            contact[i].surface.mu2 = 0;
            contact[i].surface.bounce = 0.01;
            contact[i].surface.bounce_vel = 0.1;
            contact[i].surface.soft_cfm = 0.01;
        }

        if (int numc = dCollide(o1, o2, MAX_CONTACTS, &contact[0].geom, sizeof(dContact))) {
            for (int i = 0; i < numc; i++) {
                dJointID c = dJointCreateContact(world, contactgroup, contact + i);
                dJointAttach(c, b1, b2);
            }
        }
    }

    void initODE() {
        dInitODE();
    }

    void initWorld() {
        world = dWorldCreate();
        space = dSimpleSpaceCreate(0);
        contactgroup = dJointGroupCreate(0);
        ground = dCreatePlane(space, 0, 0, 1, 0);
        dWorldSetGravity(world, 0, 0, -9.81);
        dWorldSetERP(world, 0.2);
        dWorldSetCFM(world, 1e-5);
        dWorldSetContactMaxCorrectingVel(world, 0.9);
        dWorldSetContactSurfaceLayer(world, 0.001);
        dWorldSetAutoDisableFlag(world, 1);
    }

    void initHumanoid(const geometry_msgs::Pose& pose, const geometry_msgs::Twist& twist) {

        // Create the object
        object.body = dBodyCreate(world);

        dBodySetPosition(object.body, pose.position.x, pose.position.y, pose.position.z);
        dBodySetLinearVel(object.body, twist.linear.x, twist.linear.y, twist.linear.z);
        dBodySetAngularVel(object.body, twist.angular.x, twist.angular.y, twist.angular.z);

        const dReal q[] = {pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w};
        dBodySetQuaternion(object.body, q);

        dMass m;

        // TODO: Use a true point mass here
        // TODO: Determine if we need to adjust the COM.
        // Use the inertia matrix for a point mass rotating about the ground.
        dMassSetSphereTotal(&m, humanoidMass, 0.01);
        dBodySetMass(object.body, &m);
        object.geom[0] = dCreateCylinder(space, 0.01 /* radius */, humanoidHeight);
        dGeomSetBody(object.geom[0], object.body);
    }

    static void staticNearCallback(void* data, dGeomID o1, dGeomID o2){
      FallPredictor* self = static_cast<FallPredictor*>(data);
      self->collisionCallback(o1, o2);
    }

    void destroyODE() {
        dJointGroupDestroy(contactgroup);
        dJointGroupDestroy(groundToBodyJG);
        dSpaceDestroy(space);
        // dWorldDestroy(world);
        dCloseODE();
    }

    void simLoop(double stepSize) {
        dSpaceCollide(space, this, &FallPredictor::staticNearCallback);
        dWorldStep(world, stepSize);
        dJointGroupEmpty(contactgroup);
    }

    void publishPathViz(const vector<geometry_msgs::Pose>& path, const string& frame) const {
        visualization_msgs::Marker points;
        points.header.frame_id = frame;
        points.header.stamp = ros::Time::now();
        points.ns = "fall_prediction";
        points.id = 0;
        points.type = visualization_msgs::Marker::POINTS;
        points.pose.orientation.w = 1.0;
        points.scale.x = 0.01;
        points.scale.y = 0.01;

        // Points are green
        points.color.g = 1.0f;
        points.color.a = 1.0;

        for (unsigned int i = 0; i < path.size(); ++i) {
            points.points.push_back(path[i].position);
        }
        fallVizPub.publish(points);
    }

    void initGroundJoint(const geometry_msgs::Pose& humanPose) {
      // Create the ground object
      groundLink = dBodyCreate(world);
      groundToBodyJG = dJointGroupCreate(0);

      // TODO: This is wrong if the human starts in a non-identity pose
      dBodySetPosition(groundLink, humanPose.position.x, humanPose.position.y, 0.0 /* No height in base frame */);

      // Set it as unresponsive to forces
      dBodySetKinematic(groundLink);

      groundJoint = dJointCreateBall(world, groundToBodyJG);
      dJointAttach(groundJoint, object.body, groundLink);

      // TODO: This is wrong if the human starts in a non-identity pose
      dJointSetBallAnchor(groundJoint, humanPose.position.x, humanPose.position.y, 0.0);
    }

    bool predict(human_catching::PredictFall::Request& req,
               human_catching::PredictFall::Response& res) {
      ROS_INFO("Predicting fall in frame %s", req.header.frame_id.c_str());
      res.header = req.header;

      // Initialize ODE
      initODE();

      initWorld();

      ROS_INFO("Initializing humanoid with pose: %f %f %f", req.pose.position.x, req.pose.position.y, req.pose.position.z);
      initHumanoid(req.pose, req.twist);

      initGroundJoint(req.pose);

      // Execute the simulation loop for 2 seconds
      for (double t = 0; t <= DURATION; t += STEP_SIZE) {
        // Step forward
        simLoop(STEP_SIZE);

        // Get the location of the body for the current iteration
        const dReal* position = dBodyGetPosition(object.body);
        const dReal* orientation = dBodyGetQuaternion(object.body);
        geometry_msgs::Pose pose;
        pose.position.x = position[0];
        pose.position.y = position[1];
        pose.position.z = position[2];
        pose.orientation.x = orientation[0];
        pose.orientation.y = orientation[1];
        pose.orientation.z = orientation[2];
        pose.orientation.w = orientation[3];
        res.path.push_back(pose);

        const dReal* linearVelocity = dBodyGetLinearVel(object.body);
        const dReal* angularVelocity = dBodyGetAngularVel(object.body);
        geometry_msgs::Twist twist;
        twist.linear.x = linearVelocity[0];
        twist.linear.y = linearVelocity[1];
        twist.linear.z = linearVelocity[2];
        twist.angular.x = angularVelocity[0];
        twist.angular.y = angularVelocity[1];
        twist.angular.z = angularVelocity[2];
        res.velocityPath.push_back(twist);
        res.times.push_back(t);
      }

      // Publish the path
      ROS_INFO("Publishing estimated path");
      publishPathViz(res.path, res.header.frame_id);

      // Clean up
      dWorldDestroy (world);
      destroyODE();
      ROS_INFO("Completed fall prediction");
      return true;
    }
  };
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "fall_predictor");
  FallPredictor fp;
  ros::spin();
}
