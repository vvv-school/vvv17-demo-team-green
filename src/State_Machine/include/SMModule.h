#include <string>
#include <vector>
#include <yarp/os/RFModule.h>
#include <yarp/os/RpcServer.h>
#include <yarp/os/RpcClient.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>



using namespace std;
using namespace yarp::os;

#define START_STATE                 0
#define IDLE_STATE                  1
#define TRACKING_FACE_STATE         2
#define TRACKING_OBJECT_STATE       3
#define RECOGNISE_OBJECT_STATE      4
#define POINT_OBJECT_STATE          5
#define REACT_STATE                 6
#define PUSH_OBJECT_STATE           7

class SMModule : public yarp::os::RFModule
{
public:

    //SMModule;
    //virtual ~SMModule;

    /*
    * Configure function. Receive a previously initialized
    * resource finder object. Use it to configure your module.
    * Open port and attach it to message handler and etc.
    */
    virtual bool configure(yarp::os::ResourceFinder &rf);

    /**
     * set the period with which updateModule() should be called
     */
    virtual double getPeriod();

    /*
    * This is our main function. Will be called periodically every getPeriod() seconds.
    */
    virtual bool updateModule();

    /*
    * Message handler. Just echo all received messages.
    */
    virtual bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply);

    /*
    * Interrupt function.
    */
    virtual bool interruptModule();

    /*
    * Close function, to perform cleanup.
    */
    virtual bool close();

private:
    void pushObject();
    bool objectInBin();
    void pointAtObject();
    bool getBinCoords();
    bool track(const string trackedType);
    string queryDetector();
    bool openPorts();


private:
    std::string moduleName;
    bool shouldWait;
    int state;

    vector<double> objPos;
    vector<double> facePos;
    vector<double> binPos;
    yarp::os::RpcServer commandPort;                    // command port
    yarp::os::RpcClient KinematicsPort;                 // Kinematics
    yarp::os::RpcClient DetectorPort;                   // Detector
    yarp::os::RpcClient TrackingPort;                // Tracker
    yarp::os::RpcClient RecogniserPort;                 // Recogniser
};