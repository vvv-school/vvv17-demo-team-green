#include <string>

#include <yarp/os/Network.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RpcClient.h>
#include <yarp/os/RpcServer.h>


#include <yarp/os/Time.h>
#include <yarp/os/Mutex.h>
#include <yarp/sig/Vector.h>

#include <yarp/dev/Drivers.h>
#include <yarp/dev/GazeControl.h>
#include <yarp/dev/PolyDriver.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;


class ControllT: public RateThread
{
protected:
    PolyDriver clientGaze;
    IGazeControl  *igaze;

    int state;
    int startup_context_id;

    BufferedPort<Bottle> targetsPort;
    RpcClient neckPort;

    std::string moduleName;

    bool trackingFace;
    Mutex mutex;


public:
    ControllT(std::string moduleName) : RateThread(1000), moduleName(moduleName) { }

    virtual bool threadInit()
    {
        //string robot=rf.check("robot",Value("icubSim")).asString();

        Property optGaze;
        optGaze.put("device","gazecontrollerclient");
        optGaze.put("remote","/iKinGazeCtrl");
        optGaze.put("local","/tracker/gaze");

        if (!clientGaze.open(optGaze))
        {
            yError()<<"Unable to open the Gaze Controller";
            return false;
        }
        clientGaze.view(igaze);

        igaze->storeContext(&startup_context_id);

        // set trajectory time
        igaze->setNeckTrajTime(1);
        igaze->setEyesTrajTime(1);

        targetsPort.open("/"+moduleName+"/target:i");
        neckPort.open("/"+moduleName+"/state_machine_rpc:o");


        return true;
    }

    virtual void run()
    {

        Bottle *pTarget = targetsPort.read(false);
        Vector px(2);

        mutex.lock();
        if(pTarget != NULL){
          if(trackingFace){
            yInfo()<<"track face";
            px[0]=pTarget->get(1).asDouble();
            px[1]=pTarget->get(2).asDouble();
            if(px[0]!=-1 && px[1]!=-1)
              igaze->lookAtMonoPixel(0,px);   // 0: left image plane is used, 1: for right
            //yInfo()<<"gazing at pixel: "<<px.toString(3,3);

          }else{
            yInfo()<<"track object";
            px[0]=pTarget->get(4).asDouble();
            px[1]=pTarget->get(5).asDouble();
            if(px[0]!=-1 && px[1]!=-1)
              igaze->lookAtMonoPixel(0,px);   // 0: left image plane is used, 1: for right
            //yInfo()<<"gazing at pixel: "<<px.toString(3,3);
          }
        }
        mutex.unlock();

        Vector x;
        Vector o;
        igaze->getHeadPose(x,o,NULL);
        double angle = o[1]*o[3];
        yInfo()<<"angle " << angle;
        if(angle < -30){
          Bottle neckDown, response;
          neckDown.addString("objOnTable");
          neckPort.write(neckDown,response);
        }
    }

    virtual void setTrackingFace(bool isTrackingFace){
      mutex.lock();
      trackingFace = isTrackingFace;
      mutex.unlock();
    }

    virtual void threadRelease()
    {
        // stop
        igaze->stopControl();

        // restore the controller context
        igaze->restoreContext(startup_context_id);
        targetsPort.close();
        neckPort.close();
        clientGaze.close();
    }
};

class CtrlModule: public RFModule
{
protected:
    ControllT *thr;

    RpcServer commandPort;

    bool trackingface;


public:
    virtual bool configure(ResourceFinder &rf)
    {
        // retrieve command line options


        double period=rf.check("period",Value(0.02)).asDouble();
        std::string moduleName = rf.check("name", Value("GC_gazecontrol")).asString();
        thr = new ControllT(moduleName);

        commandPort.open("/"+moduleName+"/command");
        if(!attach(commandPort)) {
            yError()<<"Cannot attach to the commandPort";
            return false;
        }

        // set the thread rate that is an integer accounting for [ms]
        thr->setRate(int(period*1000.0));

        return thr->start();
    }

    virtual bool close()
    {
        thr->stop();
        return true;
    }

    virtual double getPeriod()
    {
        return 1.0;
    }

    virtual bool respond(const Bottle& command, Bottle& reply)
    {
      if(command.size() == 0){
        return false;
      }
        if(command.size()==2){
          if (command.get(0).asString()=="track"){
            if(command.get(1).asString()=="face"){
              thr->setTrackingFace(true);
              thr->resume();
              reply.addString("ack");
              return true;
            }
            else if(command.get(1).asString()=="object"){
              thr->setTrackingFace(false);
              thr->resume();
              reply.addString("ack");
              return true;
            }
            else{
              yInfo()<<"I don't know what to track";
              reply.addString("nack");
              thr->suspend();
              return true;
            }
          }
          else if(command.get(0).asString()=="stoptrack"){
            thr->suspend();
            reply.addString("ack");
            yInfo()<<"stop tracking";
            return true;
          }else{
            yInfo()<<"command unknown";
            reply.addString("nack");
            return true;
          }

        }else{
          yInfo()<<"command bottle of wrong dimension";
          reply.addString("nack");
          return true;
        }

    }

    virtual bool updateModule()
    {
        Bottle command;
        commandPort.read(command,true);

        return true;
    }
};

int main(int argc, char *argv[])
{
    Network yarp;
    if (!yarp.checkNetwork())
    {
        yError()<<"YARP doesn't seem to be available";
        return 1;
    }

    ResourceFinder rf;
    rf.configure(argc,argv);

    CtrlModule mod;
    return mod.runModule(rf);
}
