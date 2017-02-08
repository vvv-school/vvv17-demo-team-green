#include <string>
#include <vector>
#include <yarp/os/RFModule.h>
#include <yarp/os/RpcServer.h>
#include <yarp/os/RpcClient.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

// include for speech
#include <cstdlib>
#include <deque>
#include <yarp/os/all.h>


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
#define INITIALIZE_TABLE            8

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
    //bool objectInBin();
    void pointAtObject();
    bool getTargetBin();
    bool track(const string trackedType);
    string queryDetector();
    bool openPorts();
    bool initBins();
    bool getBinImage();

    void speak(const string &phrase);

    string targetBin;
    string objectName;


private:

    yarp::sig::ImageOf<yarp::sig::PixelRgb> *inImage;

    std::string moduleName;
    bool shouldWait;
    int state;
    int checkObjBin(const Bottle cmd);
    vector<vector<double> > bins;
    vector<double> objPos;
    vector<double> facePos;
    vector<double> binPos;
    yarp::os::RpcServer commandPort;                    // command port
    yarp::os::RpcClient KinematicsPort;                 // Kinematics
    BufferedPort<Bottle> DetectorPortOut;                   // Detector
    BufferedPort<Bottle> DetectorPortIn;                   // Detector
    BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > DetectorPortImage;                   // Detector
    yarp::os::RpcClient TrackingPort;                // Tracker
    yarp::os::RpcClient RecogniserPort;                 // Recogniser
    yarp::os::RpcClient BinPort;
    BufferedPort<Bottle> SpeechPort;
};
