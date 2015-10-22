#include <ros/ros.h>
#include <human_catching/PredictFall.h>
#include <visualization_msgs/Marker.h>
#include <ode/ode.h>
#include <iostream>

namespace {
using namespace std;

//! Default weight in kg
static const double WEIGHT_DEFAULT = 50;
static const double HEIGHT_DEFAULT = 1;
static const double DENSITY_DEFAULT = 0.5;

static const double STEP_SIZE = 0.05;
static const double DURATION = 2.0;
static const unsigned int MAX_CONTACTS = 10;

    struct Humanoid {
        dBodyID body;  // the dynamics body
        dGeomID geom[1];  // geometries representing this body
    };

static dWorldID world;
static dSpaceID space;
static dGeomID  ground;
static dJointGroupID contactgroup;
static Humanoid object;

// TODO: Use boost bind and eliminate static variables
static void nearCallback (void *data, dGeomID o1, dGeomID o2) {
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

    //! Weight of the humanoid
    double humanoidWeight;

    //! Height of the center of mass
    double humanoidHeight;

    //! Density of the humanoid
    double humanoidDensity;
public:
   FallPredictor() :
     pnh("~") {
       pnh.param("humanoid_weight", humanoidWeight, WEIGHT_DEFAULT);
       pnh.param("humanoid_height", humanoidHeight, HEIGHT_DEFAULT);
       pnh.param("humanoid_density", humanoidDensity, DENSITY_DEFAULT);
        cout << "HumanoidDensity" << humanoidDensity << endl;
       fallVizPub = nh.advertise<visualization_msgs::Marker>(
         "/fall_predictor/projected_path", 1);

       fallPredictionService = nh.advertiseService("/fall_predictor/predict_fall",
            &FallPredictor::predict, this);
	}

private:
    void initODE() {
        dInitODE();
        world = dWorldCreate();
        space = dSimpleSpaceCreate(0);
        contactgroup = dJointGroupCreate(0);
        dCreatePlane(space, 0, 1, 0, 0);
        dWorldSetGravity(world, 0, -1.0, 0);
        dWorldSetERP(world, 0.2);
        dWorldSetCFM(world, 1e-5);
        dWorldSetContactMaxCorrectingVel(world, 0.9);
        dWorldSetContactSurfaceLayer(world, 0.001);
        dWorldSetAutoDisableFlag(world, 1);
        object.body = dBodyCreate(world);
        dBodySetPosition(object.body, 0, 10, -5);
        dBodySetLinearVel(object.body, 0.0 /* x */, 0.0 /* y */, 0.0 /* z */);
        dMatrix3 R;
        dRFromAxisAndAngle(R, dRandReal() * 2.0 - 1.0,
                          dRandReal() * 2.0 - 1.0,
                          dRandReal() * 2.0 - 1.0,
                          dRandReal() * 10.0 - 5.0);
        dBodySetRotation(object.body, R);
        dMass m;
        dReal sides[3];
        sides[0] = 2.0;
        sides[1] = 2.0;
        sides[2] = 2.0;
        dMassSetBox(&m, humanoidDensity, sides[0], sides[1], sides[2]);
        cout << "dMass: " << m.mass << endl;

        dBodySetMass(object.body, &m);
        object.geom[0] = dCreateBox(space, sides[0], sides[1], sides[2]);
        dGeomSetBody(object.geom[0], object.body);
    }

    void destroyODE() {
        dJointGroupDestroy(contactgroup);
        dSpaceDestroy(space);
        dWorldDestroy(world);
        dCloseODE();
    }

    void simLoop() {
        dSpaceCollide(space, 0, &nearCallback);
        dWorldQuickStep(world, 0.05);
        dJointGroupEmpty(contactgroup);
    }

    bool predict(human_catching::PredictFall::Request& req,
               human_catching::PredictFall::Response& res) {
      // Initialize ODE
      initODE();
      world = dWorldCreate();
      space = dHashSpaceCreate(0);
      contactgroup = dJointGroupCreate(0);

      dWorldSetGravity(world,0,0,-9.8);

      // Create a ground
      ground = dCreatePlane(space, 0, 0, 1, 0);

      // TODO: Create the humanoid

      // Execute the simulation loop for 2 seconds
      for (double t = 0; t <= DURATION; t += STEP_SIZE) {
        simLoop();
      }

      // Clean up
      dWorldDestroy (world);
      destroyODE();

      return true;
    }
  };
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "fall_predictor");
  FallPredictor fp;
  ros::spin();
}
