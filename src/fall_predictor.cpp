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
static const double RADIUS_DEFAULT = 0.025; // Width of pole

static const double STEP_SIZE = 0.001;
static const double DURATION = 2.0;
static const unsigned int MAX_CONTACTS = 6;

// Computed from models. Note that the gazebo convention for the PR2 model has the end
// effector aligned along the z axis in the zero orientation
static const double END_EFFECTOR_WIDTH = 0.100908;
static const double END_EFFECTOR_HEIGHT = 0.055100;
static const double END_EFFECTOR_LENGTH = 0.244724;

// Inflate the end effector to detect near contact
static const double INFLATION_FACTOR = 0.0;

struct Model {
    dBodyID body;  // the dynamics body
    dGeomID geom;  // geometries representing this body
};

class FallPredictor {
private:
    //! Publisher for the fall visualization
    ros::Publisher fallVizPub;

    //! Publisher for the contact visualization
    ros::Publisher contactVizPub;

    //! Node handle
    ros::NodeHandle nh;

    //! Private nh
    ros::NodeHandle pnh;

    //! Fall prediction Service
    ros::ServiceServer fallPredictionService;

    //! Height of object
    double humanoidHeight;

    //! Radius of object
    double humanoidRadius;

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

    //! Current time
    double t;
public:
   FallPredictor() :
     pnh("~"), t(0.0) {
       pnh.param("humanoid_height", humanoidHeight, HEIGHT_DEFAULT);
       pnh.param("humanoid_radius", humanoidRadius, RADIUS_DEFAULT);
       pnh.param("humanoid_mass", humanoidMass, MASS_DEFAULT);

       fallVizPub = nh.advertise<visualization_msgs::Marker>(
         "/fall_predictor/projected_path", 1);

       contactVizPub = nh.advertise<visualization_msgs::Marker>(
         "/fall_predictor/contacts", 1);

       fallPredictionService = nh.advertiseService("/fall_predictor/predict_fall",
            &FallPredictor::predict, this);
	}

private:
    void collisionCallback(dGeomID o1, dGeomID o2) {
        dBodyID b1 = dGeomGetBody(o1);
        dBodyID b2 = dGeomGetBody(o2);
        dContact contact[MAX_CONTACTS];
        for (int i = 0; i < MAX_CONTACTS; i++){
            contact[i].surface.mode = dContactSoftCFM;
            contact[i].surface.mu = dInfinity;
            contact[i].surface.mu2 = 0;
            contact[i].surface.soft_cfm = 0.01;
        }

        // Always compute contact with the humanoid as object 1, such that the contact normal
        // is towards the humanoid
        dGeomID firstObj = o1 == humanoid.geom ? o1 : o2;
        dGeomID secondObj = o1 == humanoid.geom ? o2 : o1;

        if (int numc = dCollide(firstObj, secondObj, MAX_CONTACTS, &contact[0].geom, sizeof(dContact))) {
            for (int i = 0; i < numc; i++) {
                dJointID c = dJointCreateContact(world, contactgroup, contact + i);
                dJointAttach(c, b1, b2);

                // Determine if this contact should be saved
                int which = whichEndEffector(b1, b2);
                if ((b1 == humanoid.body || b2 == humanoid.body) && which != -1) {
                    assert(firstObj == humanoid.geom && secondObj == endEffectors[which].geom
                           && firstObj == contact[i].geom.g1 && secondObj == contact[i].geom.g2);

                    // Only save one contact per end effector
                    eeContacts[which] = contact[i].geom;
                    hasEeContacts[which] = true;
                }
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
        dWorldSetContactSurfaceLayer(world, 0.001);
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
        humanoid.geom = dCreateCylinder(space, humanoidRadius, humanoidHeight);
        dGeomSetBody(humanoid.geom, humanoid.body);
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
        endEffectors.clear();
        eeContacts.clear();
        hasEeContacts.clear();
        dCloseODE();
    }

    void simLoop(double stepSize) {
        dSpaceCollide(space, this, &FallPredictor::staticNearCallback);
        dWorldStep(world, stepSize);
        dJointGroupEmpty(contactgroup);
    }

    void publishContacts(const vector<humanoid_catching::Contact>& contacts, const string& frame) const {

        for (unsigned int i = 0; i < contacts.size(); ++i) {
            if (!contacts[i].is_in_contact) {
                visualization_msgs::Marker deleteAll;
                deleteAll.ns = "contacts";
                deleteAll.action = visualization_msgs::Marker::DELETE;
                contactVizPub.publish(deleteAll);
                continue;
            }

            visualization_msgs::Marker arrow;
            arrow.header.frame_id = frame;
            arrow.header.stamp = ros::Time::now();
            arrow.ns = "contacts";
            arrow.id = i;
            arrow.type = visualization_msgs::Marker::ARROW;
            arrow.pose.orientation = quaternionFromVector(contacts[i].normal);
            arrow.pose.position = contacts[i].position;
            arrow.scale.x = 0.5;
            arrow.scale.y = 0.05;
            arrow.scale.z = 0.05;

            // Contacts are blue
            arrow.color.b = 1.0f;
            if (i == 1) {
                arrow.color.r = 1.0f;
            }
            arrow.color.a = 1.0;
            contactVizPub.publish(arrow);
        }
    }

    void publishEeViz(const vector<geometry_msgs::Pose>& ees, const string& frame) const {
        for (unsigned int i = 0; i < ees.size(); ++i) {
            visualization_msgs::Marker box;
            box.header.frame_id = frame;
            box.header.stamp = ros::Time::now();
            box.ns = "end_effectors";
            box.id = i;
            box.type = visualization_msgs::Marker::CUBE;
            box.pose = ees[i];
            box.scale.x = END_EFFECTOR_LENGTH;
            box.scale.y = END_EFFECTOR_WIDTH;
            box.scale.z = END_EFFECTOR_HEIGHT;

            // Box is red
            box.color.r = 1.0f;
            if (i == 1) {
                box.color.b = 1.0f;
            }
            box.color.a = 1.0;
            fallVizPub.publish(box);
        }
    }

    void publishPathViz(const vector<geometry_msgs::Pose>& path, const string& frame) const {
        // Clear first
        visualization_msgs::Marker deleteAll;
        deleteAll.ns = "pole";
        deleteAll.action = 3;
        fallVizPub.publish(deleteAll);

        for (unsigned int i = 0; i < path.size(); ++i) {
            visualization_msgs::Marker cyl;
            cyl.header.frame_id = frame;
            cyl.header.stamp = ros::Time::now();
            cyl.ns = "pole";
            cyl.id = 0; // TODO: Enable i;
            cyl.type = visualization_msgs::Marker::CYLINDER;
            cyl.pose = path[i];
            cyl.scale.x = humanoidRadius;
            cyl.scale.y = humanoidRadius;
            cyl.scale.z = humanoidHeight;

            // Cylinder is green
            cyl.color.g = 1.0f;
            cyl.color.a = 1.0;
            fallVizPub.publish(cyl);
        }

        // Clear any other values
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

        ROS_INFO("Adding end effector @ %f %f %f (%f %f %f %f)",
                 endEffector.position.x, endEffector.position.y, endEffector.position.z,
                 endEffector.orientation.x, endEffector.orientation.y, endEffector.orientation.z, endEffector.orientation.w);

        object.geom = dCreateBox(space,
                                    END_EFFECTOR_LENGTH * (1 + INFLATION_FACTOR),
                                    END_EFFECTOR_WIDTH  * (1 + INFLATION_FACTOR),
                                    END_EFFECTOR_HEIGHT * (1 + INFLATION_FACTOR));
        dGeomSetBody(object.geom, object.body);
        dBodySetPosition(object.body, endEffector.position.x, endEffector.position.y, endEffector.position.z);
        dBodySetLinearVel(object.body, 0, 0, 0);
        dBodySetAngularVel(object.body, 0, 0, 0);

        const dReal q[] = {endEffector.orientation.x, endEffector.orientation.y, endEffector.orientation.z, endEffector.orientation.w};
        dBodySetQuaternion(object.body, q);

        // Set it as unresponsive to forces
        dBodySetKinematic(object.body);

        return object;
    }

    static geometry_msgs::Quaternion quaternionFromVector(const geometry_msgs::Vector3& input) {
        tf::Vector3 axisVector(input.x, input.y, input.z);
        tf::Vector3 upVector(0.0, 0.0, 1.0);
        tf::Vector3 rightVector = axisVector.cross(upVector);
        rightVector.normalized();
        tf::Quaternion q(rightVector, -1.0 * acos(axisVector.dot(upVector)));
        q.normalize();
        geometry_msgs::Quaternion orientation;
        tf::quaternionTFToMsg(q, orientation);
        return orientation;
    }

    static geometry_msgs::Vector3 arrayToVector(const dReal* aArray) {
        geometry_msgs::Vector3 result;
        result.x = aArray[0];
        result.y = aArray[1];
        result.z = aArray[2];
        return result;
    }

    static geometry_msgs::Point arrayToPoint(const dReal* aArray) {
        geometry_msgs::Point result;
        result.x = aArray[0];
        result.y = aArray[1];
        result.z = aArray[2];
        return result;
    }

    static geometry_msgs::Quaternion arrayToQuat(const dReal* aArray) {
        geometry_msgs::Quaternion result;
        result.x = aArray[0];
        result.y = aArray[1];
        result.z = aArray[2];
        result.w = aArray[3];
        return result;
    }

    static geometry_msgs::Pose getBodyPose(dBodyID body) {
        const dReal* position = dBodyGetPosition(body);
        const dReal* orientation = dBodyGetQuaternion(body);
        geometry_msgs::Pose pose;
        pose.position = arrayToPoint(position);
        pose.orientation = arrayToQuat(orientation);
        return pose;
    }

    static geometry_msgs::Twist getBodyTwist(dBodyID body) {
        const dReal* linearVelocity = dBodyGetLinearVel(body);
        const dReal* angularVelocity = dBodyGetAngularVel(body);
        geometry_msgs::Twist twist;
        twist.linear = arrayToVector(linearVelocity);
        twist.angular = arrayToVector(angularVelocity);
        return twist;
    }

    vector<double> getPoleInertiaMatrix() const {
        vector<double> I(9);
        I[0] = I[4] = 1 / 12.0 * humanoidMass * pow(humanoidHeight, 2) + 0.25 * humanoidMass * pow(humanoidRadius, 2);
        I[8] = 0.5 * humanoidMass * pow(humanoidRadius, 2);
        return I;
    }

    bool predict(humanoid_catching::PredictFall::Request& req,
               humanoid_catching::PredictFall::Response& res) {
      ROS_INFO("Predicting fall in frame %s", req.header.frame_id.c_str());

      res.header = req.header;

      // Initialize ODE
      initODE();

      initWorld();

      ROS_DEBUG("Initializing humanoid with pose: %f %f %f (%f %f %f %f)", req.pose.position.x, req.pose.position.y, req.pose.position.z,
               req.pose.orientation.x, req.pose.orientation.y, req.pose.orientation.z, req.pose.orientation.w);

      initHumanoid(req.pose, req.velocity);

      initGroundJoint(req.pose);

      for (unsigned int i = 0; i < req.end_effectors.size(); ++i) {
        Model ee = initEndEffector(req.end_effectors[i]);
        endEffectors.push_back(ee);
      }

      // Execute the simulation loop for 2 seconds
      bool contact = false;
      for (t = 0; t <= DURATION && !contact; t += STEP_SIZE) {
        // Clear end effector contacts
        eeContacts.clear();
        hasEeContacts.clear();
        eeContacts.resize(req.end_effectors.size());
        hasEeContacts.resize(req.end_effectors.size());

        // Step forward
        simLoop(STEP_SIZE);

        FallPoint curr;

        // Get the location of the body for the current iteration
        curr.pose = getBodyPose(humanoid.body);
        curr.ground_contact = arrayToPoint(dBodyGetPosition(groundLink));
        curr.velocity = getBodyTwist(humanoid.body);
        curr.time = ros::Duration(t);

        curr.contacts.resize(eeContacts.size());
        for (unsigned int i = 0; i < eeContacts.size(); ++i) {
            curr.contacts[i].is_in_contact = hasEeContacts[i];
            if (hasEeContacts[i]) {
                curr.contacts[i].position = arrayToPoint(eeContacts[i].pos);
                curr.contacts[i].normal = arrayToVector(eeContacts[i].normal);
                contact = true;
            }
        }
        res.points.push_back(curr);
      }

      // Set the mass and inertia matrix
      res.body_mass = humanoidMass;
      res.inertia_matrix = getPoleInertiaMatrix();

      // Publish the path
      if (fallVizPub.getNumSubscribers() > 0) {
        vector<geometry_msgs::Pose> points;
        for (vector<FallPoint>::const_iterator i = res.points.begin(); i != res.points.end(); ++i) {
            points.push_back(i->pose);
        }
        publishPathViz(points, res.header.frame_id);
      }

      if (contactVizPub.getNumSubscribers() > 0) {
        for (vector<FallPoint>::const_iterator i = res.points.begin(); i != res.points.end(); ++i) {
            publishContacts(i->contacts, res.header.frame_id);
        }

        vector<geometry_msgs::Pose> eePoses;
        for (vector<Model>::const_iterator i = endEffectors.begin(); i != endEffectors.end(); ++i) {
            eePoses.push_back(getBodyPose(i->body));
        }
        publishEeViz(eePoses, res.header.frame_id);
      }

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
