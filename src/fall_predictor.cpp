#include <ros/ros.h>
#include <humanoid_catching/PredictFall.h>
#include <humanoid_catching/FallPoint.h>
#include <visualization_msgs/Marker.h>
#include <ode/ode.h>
#include <iostream>
#include <bullet/LinearMath/btVector3.h>
#include <tf/transform_listener.h>

namespace {
using namespace std;
using namespace humanoid_catching;

//! Default weight in kg
static const double MASS_DEFAULT = 5;
static const double HEIGHT_DEFAULT = 1.5;

static const double STEP_SIZE = 0.01;
static const double DURATION = 2.0;
static const unsigned int MAX_CONTACTS = 3;

// TOOD: Figure these out
static const double END_EFFECTOR_WIDTH = 0.4;
static const double END_EFFECTOR_LENGTH = 0.6;
static const double END_EFFECTOR_DEPTH = 0.1;

struct Model {
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
    Model humanoid;

    //! End effectors
    vector<Model> endEffectors;

    //! Contacts with end effectors
    vector<dContactGeom> eeContacts;

    //! Whether contact occurred
    vector<bool> hasEeContacts;

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

            // Determine if this contact should be saved
            int which = whichEndEffector(b1, b2);
            if (b1 == humanoid.body || b2 == humanoid.body && which != -1) {
                cout << "Located an end effector contact" << endl;

                // Only save one concact per end effector
                eeContacts[which] = contact[0].geom;
                hasEeContacts[which] = true;
            }
        }
    }

    int whichEndEffector(const dBodyID b1, const dBodyID b2) const {
        for (unsigned int i = 0; i < endEffectors.size(); ++i) {
            if (endEffectors[i].body == b1 || endEffectors[i].body == b2) {
                return i;
            }
        }
        return -1;
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

    void initHumanoid(const geometry_msgs::Pose& pose, const geometry_msgs::Twist& velocity) {

        // Create the object
        humanoid.body = dBodyCreate(world);

        dBodySetPosition(humanoid.body, pose.position.x, pose.position.y, pose.position.z);
        dBodySetLinearVel(humanoid.body, velocity.linear.x, velocity.linear.y, velocity.linear.z);
        dBodySetAngularVel(humanoid.body, velocity.angular.x, velocity.angular.y,velocity.angular.z);

        const dReal q[] = {pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w};
        dBodySetQuaternion(humanoid.body, q);

        dMass m;

        // Use the inertia matrix for a point mass rotating about the ground.
        // Use a very small sphere to mimic a point mass, which was not working properly.
        dMassSetSphereTotal(&m, humanoidMass, 0.01);
        dBodySetMass(humanoid.body, &m);
        humanoid.geom[0] = dCreateCylinder(space, 0.01 /* radius */, humanoidHeight);
        dGeomSetBody(humanoid.geom[0], humanoid.body);
    }

    static void staticNearCallback(void* data, dGeomID o1, dGeomID o2){
      FallPredictor* self = static_cast<FallPredictor*>(data);
      self->collisionCallback(o1, o2);
    }

    void destroyODE() {
        dJointGroupDestroy(contactgroup);
        dJointGroupDestroy(groundToBodyJG);
        dSpaceDestroy(space);
        dWorldDestroy(world);
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

      // Determine the intersection between the human pose and the ground plane.
      btVector3 groundNormal(0, 0, 1);
      btVector3 groundOrigin(0, 0, 0);

      tf::Quaternion tfHumanOrientation;
      tf::quaternionMsgToTF(humanPose.orientation, tfHumanOrientation);
      tf::Vector3 tfHumanVector = tf::quatRotate(tfHumanOrientation, tf::Vector3(0, 0, 1));

      btVector3 humanVector(tfHumanVector.x(), tfHumanVector.y(), tfHumanVector.z());
      btVector3 humanOrigin(humanPose.position.x, humanPose.position.y, humanPose.position.z);

      btScalar d = ((groundOrigin - humanOrigin).dot(groundNormal)) / (humanVector.dot(groundNormal));
      btVector3 intersection = d * humanVector + humanOrigin;

      // Assert intersection is at ground plane.
      assert(abs(intersection.z()) < 0.001);

      dBodySetPosition(groundLink, intersection.x(), intersection.y(), intersection.z());

      // Set it as unresponsive to forces
      dBodySetKinematic(groundLink);

      groundJoint = dJointCreateBall(world, groundToBodyJG);
      dJointAttach(groundJoint, humanoid.body, groundLink);

      dJointSetBallAnchor(groundJoint, intersection.x(), intersection.y(), intersection.z());
    }

    Model initEndEffector(const geometry_msgs::Pose& endEffector){

        // Create the object
        Model object;
        object.body = dBodyCreate(world);

        dBodySetPosition(object.body, endEffector.position.x, endEffector.position.y, endEffector.position.z);
        dBodySetLinearVel(object.body, 0, 0, 0);
        dBodySetAngularVel(object.body, 0, 0, 0);

        const dReal q[] = {endEffector.orientation.x, endEffector.orientation.y, endEffector.orientation.z, endEffector.orientation.w};
        dBodySetQuaternion(object.body, q);

        object.geom[0] = dCreateBox(space, END_EFFECTOR_LENGTH, END_EFFECTOR_WIDTH, END_EFFECTOR_DEPTH);
        dGeomSetBody(object.geom[0], object.body);

        // Set it as unresponsive to forces
        dBodySetKinematic(object.body);

        return object;
    }

    bool predict(humanoid_catching::PredictFall::Request& req,
               humanoid_catching::PredictFall::Response& res) {
      ROS_INFO("Predicting fall in frame %s", req.header.frame_id.c_str());
      res.header = req.header;

      // Initialize ODE
      initODE();

      initWorld();

      ROS_INFO("Initializing humanoid with pose: %f %f %f", req.pose.position.x, req.pose.position.y, req.pose.position.z);
      initHumanoid(req.pose, req.velocity);

      initGroundJoint(req.pose);

      for (unsigned int i = 0; i < req.end_effectors.size(); ++i) {
        endEffectors.push_back(initEndEffector(req.end_effectors[i]));
      }

      // Execute the simulation loop for 2 seconds
      for (double t = 0; t <= DURATION; t += STEP_SIZE) {
        // Clear end effector contacts
        eeContacts.clear();
        hasEeContacts.clear();
        eeContacts.resize(req.end_effectors.size());
        hasEeContacts.resize(req.end_effectors.size());

        // Step forward
        simLoop(STEP_SIZE);

        FallPoint curr;

        // Get the location of the body for the current iteration
        const dReal* position = dBodyGetPosition(humanoid.body);
        const dReal* orientation = dBodyGetQuaternion(humanoid.body);
        geometry_msgs::Pose pose;
        pose.position.x = position[0];
        pose.position.y = position[1];
        pose.position.z = position[2];
        pose.orientation.x = orientation[0];
        pose.orientation.y = orientation[1];
        pose.orientation.z = orientation[2];
        pose.orientation.w = orientation[3];
        curr.pose = pose;

        const dReal* linearVelocity = dBodyGetLinearVel(humanoid.body);
        const dReal* angularVelocity = dBodyGetAngularVel(humanoid.body);
        geometry_msgs::Twist twist;
        twist.linear.x = linearVelocity[0];
        twist.linear.y = linearVelocity[1];
        twist.linear.z = linearVelocity[2];
        twist.angular.x = angularVelocity[0];
        twist.angular.y = angularVelocity[1];
        twist.angular.z = angularVelocity[2];

        curr.velocity = twist;
        curr.time = ros::Duration(t);

        curr.contacts.resize(eeContacts.size());
        for (unsigned int i = 0; i < eeContacts.size(); ++i) {
            curr.contacts[i].is_in_contact = hasEeContacts[i];
            if (hasEeContacts[i]) {
                curr.contacts[i].position.x = eeContacts[i].pos[0];
                curr.contacts[i].position.y = eeContacts[i].pos[1];
                curr.contacts[i].position.z = eeContacts[i].pos[2];
                curr.contacts[i].normal.x = eeContacts[i].normal[0];
                curr.contacts[i].normal.y = eeContacts[i].normal[1];
                curr.contacts[i].normal.z = eeContacts[i].normal[2];
            }
        }
        res.points.push_back(curr);
      }

      // Publish the path
      if (fallVizPub.getNumSubscribers() > 0) {
        ROS_INFO("Publishing estimated path");
        vector<geometry_msgs::Pose> points;
        for (vector<FallPoint>::const_iterator i = res.points.begin(); i != res.points.end(); ++i) {
            points.push_back(i->pose);
        }
        publishPathViz(points, res.header.frame_id);
      }

      // TODO: Publish contacts

      // Clean up
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
