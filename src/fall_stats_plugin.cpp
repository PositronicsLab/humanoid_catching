#include <gazebo/gazebo.hh>
#include <gazebo/math/Vector3.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Link.hh>

#include <iostream>
#include <fstream>
#include <vector>
#include <boost/filesystem.hpp>

#define PRINT_DEBUG 1

using namespace std;

namespace gazebo
{
class FallStatsPlugin : public ModelPlugin
{

private:
    gazebo::transport::NodePtr node;
    event::ConnectionPtr connection;
    physics::WorldPtr world;
    physics::ModelPtr model;
    double contactTime;
    bool printed;
public:
    FallStatsPlugin() : ModelPlugin()
    {
#if(PRINT_DEBUG)
        cout << "Constructing the fall stats plugin" << std::endl;
#endif
        contactTime = 0;
        printed = false;
    }

public:
    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
        world = _model->GetWorld();
        model = _model;
#if(PRINT_DEBUG)
        cout << "Loading the fall stats plugin" << endl;
#endif

        connection = event::Events::ConnectWorldUpdateBegin(boost::bind(&FallStatsPlugin::worldUpdate, this));
    }

private:
    void writeHeader(ofstream& outputCSV)
    {
        outputCSV << "Ground Contact Time(s)" << endl;
    }

    void printResults()
    {
        // Get the name of the folder to store the result in
        const char* resultsFolder = std::getenv("RESULTS_FOLDER");
        if(resultsFolder == NULL)
        {
            cout << "Results folder not set. Using current directory." << endl;
            resultsFolder = "./";
        }

        const string resultsFileName = string(resultsFolder) + "/" + "results.csv";
        bool exists = boost::filesystem::exists(resultsFileName);
        ofstream outputCSV;
        outputCSV.open(resultsFileName.c_str(), ios::out | ios::app);
        assert(outputCSV.is_open());

        if(!exists)
        {
            writeHeader(outputCSV);
        }

        outputCSV << contactTime << ", " << endl;
        outputCSV.close();
    }

    void worldUpdate()
    {
        physics::LinkPtr link = model->GetLink("link");
#if(PRINT_DEBUG)
        cout << "Updating ground contact time " << world->GetSimTime() << endl;
#endif
        if (contactTime == 0 && link->GetWorldCoGPose().pos.z <= 0.025 + 0.01) {
#if(PRINT_DEBUG)
            cout << "Setting contact time" << endl;
            contactTime = world->GetSimTime().Double();
#endif
        }

        if(!printed && world->GetSimTime().Float() >= 30.0)
        {
#if(PRINT_DEBUG)
            cout << "Scenario completed. Updating results" << endl;
#endif
            printed = true;
            printResults();
        }
    }
};
GZ_REGISTER_MODEL_PLUGIN(FallStatsPlugin);
}

