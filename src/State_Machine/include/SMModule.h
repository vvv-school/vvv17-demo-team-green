#include <string>
#include <yarp/os/RFModule.h>
#include <yarp/os/RpcServer.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>

class SMModule : public yarp::os::RFModule
{
public:

    SMModule;
    virtual ~SMModule;

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
    std::string encode(const std::string &msg);
    std::string decode(const std::string &msg);

private:
    std::string modeParam;
    yarp::os::RpcServer commandPort;                    // command port
    yarp::os::BufferedPort<yarp::os::Bottle> inPort;    // input port
    yarp::os::BufferedPort<yarp::os::Bottle> outPort;   // output port
};
