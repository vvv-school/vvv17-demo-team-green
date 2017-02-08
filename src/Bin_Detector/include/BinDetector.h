#include <yarp/os/BufferedPort.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Semaphore.h>
#include <yarp/sig/Image.h>

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

#include <vector>
#include <iostream>

/********************************************************/
class BinDetector : public yarp::os::RFModule
{
    yarp::os::RpcServer commandPort;                    // command port
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >   inPort;   // input port
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >   imageOut;  
     yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> >   imageGray;  
    yarp::os::BufferedPort<yarp::os::Bottle> outPort;   // output port

    std::vector<std::vector<double> > bins;
    std::string moduleName;

    std::vector<int32_t> lowBound;
    std::vector<int32_t> highBound;

    //yarp::os::Mutex mutex;


    void binDetection();

public:

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

    bool getBins(yarp::os::Bottle&);
};