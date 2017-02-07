#include <SMModule.h>
#include <yarp/os/LogStream.h>

using namespace yarp::os;


//SMModule::SMModule() { }

//SMModule::~SMModule() { }


bool SMModule::configure(yarp::os::ResourceFinder &rf) {

    yInfo()<<"Configuring the SMModule module...";
    state = START_STATE;
    shouldWait = true; 

    moduleName = rf.check("name", Value("StateMachine")).asString();
    setName(moduleName.c_str());


    // open all ports
    if(!openPorts()) 
    {
        return false;
    }

    if(!attach(commandPort)) {
        yError()<<"Cannot attach to the commandPort";
        return false;
    }

    // everything is fine
    return true;
}

bool SMModule::openPorts()
{
    bool ret = false;
    ret = commandPort.open("/" + moduleName + "/rpc:i");
    ret &= KinematicsPort.open("/" + moduleName + "/kinematic_rpc:o");                 // Kinematics
    ret &= DetectorPort.open("/" + moduleName + "/detector_rpc:o");                   // Detector
    ret &= TrackingPort.open("/" + moduleName + "/tracking_rpc:o");                   // Tracker
    ret &= RecogniserPort.open("/" + moduleName + "/recogniser_rpc:o");                 // Recogniser

    if (!ret)
    {
        yError() << "failed to open one of the ports";
        return false;
    }
    return true;
}


double SMModule::getPeriod() {
    return 0.01; // module periodicity (seconds)
}

string SMModule::queryDetector()
{
    Bottle cmd, reply;
    cmd.addString("updateStatus");
    DetectorPort.write(cmd,reply);
    if (reply.size() > 0)
    {
        return "command failed";
    }
    if (reply.size() == 1)
    {
        return reply.get(0).asString();
    }
    else if (reply.get(0) == "object" || reply.get(1) == "object")
    {
        return "object";
    }
    else
    {
        return "command failed";
    }
    return "command failed";
}

bool SMModule::track(string trackedType)
{
    Bottle cmd, reply;
    if (trackedType == "face" || trackedType == "object")
    {
        cmd.addString("Track");
        cmd.addString(trackedType);
        DetectorPort.write(cmd,reply);
        if (reply.size() > 0 && reply.get(0) == "ok")
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    else
    {
        return false;
    }
}

bool SMModule::getBinCoords()
{
    Bottle cmd, reply;
    cmd.addString("getBinCoords");
    RecogniserPort.write(cmd, reply);
    if (reply.size() > 0)
    {
        return false;
    }   
    Bottle &binPosList = reply.get(0).asList();
    if (binPos.size() > 2)
    {
        binPos.get(0)
    }
    else
    {

    }
    return true;
}

void SMModule::pointAtObject()
{
    Bottle cmd, reply;
    cmd.addString("Point");
    Bottle &objList = cmd.addList();
    objList.addDouble(objPos[0]);
    objList.addDouble(objPos[1]);
    objList.addDouble(objPos[2]);
    KinematicsPort.write(cmd,reply);
}

bool SMModule::objectInBin()
{
    if (objPos.get(0).asDouble() < binPos.get(0).asDouble()+0.105 && objPos.get(0).asDouble() > binPos.get(0).asDouble() - 0.105 && objPos.get(1).asDouble() < binPos.get(1).asDouble() + 0.1485 && objPos.get(1).asDouble() > binPos.get(1).asDouble() - 0.1485) // this is the size of the bin, would be nice to get from perception!!!!!!!!
    {
        return true;
    }
    else
    {
        return false;
    }
}

void SMModule::pushObject()
{
    Bottle cmd, reply;
    cmd.addString("Push");
    Bottle &objList = cmd.addList();
    objList.addDouble(objPos.get(0).asDouble());
    objList.addDouble(objPos.get(1).asDouble());
    objList.addDouble(objPos.get(2).asDouble());
    Bottle &binList = cmd.addList();
    binList.addDouble(binPos.get(0).asDouble());
    binList.addDouble(binPos.get(1).asDouble());
    binList.addDouble(binPos.get(2).asDouble());
    KinematicsPort.write(cmd,reply);
}

bool SMModule::updateModule() 
{
    switch(state)
    {
        case IDLE_STATE:
        {
            string detectorOutput = queryDetector();
            if(detectorOutput == "face")
            {
                state = TRACKING_FACE_STATE;
                shouldWait = true;
            }
            else if (detectorOutput == "object")
            {
                state = TRACKING_OBJECT_STATE;
                shouldWait = false;
            }
            else
            {
                shouldWait = true;
            }
        }
        case TRACKING_FACE_STATE:
        {
            track("face");
            shouldWait = true;
        }
        case TRACKING_OBJECT_STATE:
        {
            track("object");
            shouldWait = false;
            state = RECOGNIZE_OBJECT_STATE;
        }
        case RECOGNIZE_OBJECT_STATE:
        {
            if (getBinCoords())
            {
                state = POINT_OBJECT_STATE;
            }
            else
            {
                state = IDLE_STATE;
            }
        }
        case POINT_OBJECT_STATE:
        {
            pointAtObject();
            state = REACT_STATE;
        }
        case REACT_STATE:
        {
            if (objectInBin())
            {
                state = IDLE_STATE;
            }
            else
            {
                state = PUSH_OBJECT_STATE;
            }
        }
        case PUSH_OBJECT_STATE:
        {
            pushObject();
            state = IDLE_STATE;
        }
    }
    
    return true;
}


bool SMModule::respond(const Bottle& command, Bottle& reply) {
    yInfo()<<"Got something, echo is on";
    if (command.get(0).asString()=="quit")
        return false;
    else if (command.get(0).asString() == "runModule")
    {
        if (state != START_STATE)
        {
            state = IDLE_STATE;
            return true;
        }
        else
        {
            return false;
        }
    }
    else if (command.get(0).asString() == "getPos" && command.get(1).isString())
    {
        if (command.get(1).asString() == "object")
        {
            Bottle reply;
            reply.addDouble(objPos.get(0).asDouble());
            reply.addDouble(objPos.get(1).asDouble());
            reply.addDouble(objPos.get(2).asDouble());
            commandPort.write(reply);
        }
        else if (command.get(1).asString() == "face")
        {
            Bottle reply;
            reply.addDouble(facePos.get(0).asDouble());
            reply.addDouble(facePos.get(1).asDouble());
            reply.addDouble(facePos.get(2).asDouble());
            commandPort.write(reply);
        }
        else
        {
            Bottle reply;
            reply.addString("input not recognised");
            return false;
        }
        
    }
    else {
        reply.clear();
        reply.addString("error");
    }
    return true;
}


bool SMModule::interruptModule() {
    yInfo()<<"Interrupting State Machine module";
    commandPort.interrupt();
    KinematicsPort.interrupt();
    DetectorPort.interrupt();
    TrackingPort.interrupt();
    RecogniserPort.interrupt();
    return true;
}


bool SMModule::close() {
    yInfo()<<"closing State Machine module";
    commandPort.close();
    KinematicsPort.close();
    DetectorPort.close();
    TrackingPort.close();
    RecogniserPort.close();
    return true;
}


