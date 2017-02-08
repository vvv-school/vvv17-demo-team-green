#include <SMModule.h>
#include <yarp/os/LogStream.h>

using namespace yarp::os;


//SMModule::SMModule() { }

//SMModule::~SMModule() { }


bool SMModule::configure(yarp::os::ResourceFinder &rf) {

    objPos.resize(3);    
    binPos.resize(3);
    facePos.resize(3);
    bins.resize(3);
    bins[0].resize(3);
    bins[1].resize(3);
    bins[2].resize(3);


    targetBin = "";
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

    state = INITIALIZE_TABLE;

    if(!attach(commandPort)) {
        yError()<<"Cannot attach to the commandPort";
        return false;
    }

    // everything is fine
    return true;

}

int SMModule::checkObjBin(Bottle cmd)
{

    Bottle rpccmd, reply;
    rpccmd.addString("stopTracking");
    rpccmd.addString("object");
    TrackingPort.write(rpccmd,reply);
    if(reply.size()> 0)
    {
        if(reply.get(0).asString() == "ack")
        {
            initBins();
            for(int i = 0; i<bins.size(); ++i)
            {
                if(cmd.get(0).asInt()<bins[i][0] +10  &&    cmd.get(0).asInt()>bins[i][0] -10   && cmd.get(1).asInt()<bins[i][1] +10  && cmd.get(1).asInt()>bins[i][1] -10 )        
                {
                    if(i==0 && targetBin == "aluminum")       
                        return 1;
                    if(i==1 && targetBin == "plastic")       
                        return 1; 
                    if(i==2 && targetBin == "paper")       
                        return 1;         
                }
                if(cmd.get(0).asInt() < 190  &&    cmd.get(0).asInt() > 150   && cmd.get(1).asInt() < 150  && cmd.get(1).asInt() > 90 )        
                {
                    return 2;
                }            
            }
        }    
    }
    return 0;
}

bool SMModule::openPorts()
{
    bool ret = false;
    ret = commandPort.open("/" + moduleName + "/rpc:i");
    ret &= KinematicsPort.open("/" + moduleName + "/kinematic_rpc:o");                 // Kinematics
    ret &= DetectorPortOut.open("/" + moduleName + "/detector:o");                   // Detector
    ret &= DetectorPortIn.open("/" + moduleName + "/detector:i");                   // Detector
    ret &= DetectorPortImage.open("/" + moduleName + "/detector_image:i");                   // Detector Image
    ret &= TrackingPort.open("/" + moduleName + "/tracking_rpc:o");                   // Tracker
    ret &= RecogniserPort.open("/" + moduleName + "/recogniser_rpc:o");                 // Recogniser
    ret &= BinPort.open("/" + moduleName + "/bin_detector_rpc:o");
    ret &= SpeechPort.open("/" + moduleName + "/speech_rpc:o");
   // ret &= speechPort.open("/" + moduleName + "/Speech:o"); ??what is this?

    if (!ret)
    {
        yError() << "failed to open one of the ports";
        return false;
    }
    return true;
}

bool SMModule::initBins()
{
    
    if(TrackingPort.getOutputCount() == 0)
        return true;
    Bottle cmd, reply;
    cmd.addString("lookTable");
    TrackingPort.write(cmd, reply); 
    if (reply.size() < 1 )
    {
        yInfo() << "failed to communicate with gaze control";
        return false;
    }
    if (reply.get(0).asString() == "nack")
    {
        return false;
    }
    cmd.clear();
    reply.clear();
    cmd.addString("getBins");
    BinPort.write(cmd, reply);
    if (reply.size() < 1 )
    {
        yInfo() << "failed to communicate with bin detector";
        return false;
    }
    if (reply.get(0).asString() == "nack")
    {
        return false;
    }
    bins[0][0] = reply.get(0).asList()->get(0).asDouble();
    bins[0][1] = reply.get(0).asList()->get(1).asDouble();
    bins[0][2] = reply.get(0).asList()->get(2).asDouble();
    bins[1][0] = reply.get(1).asList()->get(0).asDouble();
    bins[1][1] = reply.get(1).asList()->get(1).asDouble();
    bins[1][2] = reply.get(1).asList()->get(2).asDouble();
    bins[2][0] = reply.get(2).asList()->get(0).asDouble();
    bins[2][1] = reply.get(2).asList()->get(1).asDouble();
    bins[2][2] = reply.get(2).asList()->get(2).asDouble();

    cmd.clear();
    reply.clear();
    cmd.addString("lookUp");
    TrackingPort.write(cmd, reply); 
    if (reply.size() < 1 )
    {
        yInfo() << "failed to communicate with gaze control";
        return false;
    }
    if (reply.get(0).asString() == "nack")
    {
        return false;
    }
    return true;
}

double SMModule::getPeriod() {
    return 0.5; // module periodicity (seconds)  switch back to 0.01 later?
}

string SMModule::queryDetector()
{   
    Bottle *reply;
    Bottle &cmd = DetectorPortOut.prepare();
    cmd.clear();
    cmd.addString("getObject");
    DetectorPortOut.write();
    while (true)
    {        
        reply = DetectorPortIn.read(false);
        yInfo()<<__LINE__;         
        if (reply->size() > 1)
        {
            if (reply->get(1).asDouble() > 0)
            {
                yInfo()<<__LINE__;                 
                return "object";
            }
            else 
            {
                break;
            }
        }
    }
    yInfo()<<__LINE__;
    reply->clear();
    DetectorPortOut.prepare();
    cmd.clear();
    cmd.addString("getFace");
    DetectorPortOut.write();
    while (true)
    {
        reply = DetectorPortIn.read(false);
        if (reply->size() >1)
        {
            if (reply->get(1).asDouble() > 0)
            {
                return "face";
            }
            else 
            {
                break;
            }
        }
    }
    yInfo()<<__LINE__;
    return "no object or face found";
    /*if (reply.size() < 1) //>0
    {
        return "command failed";
    }
    else
    {
        if (reply.get(1).asDouble() != -1)
        {
            return "object";
        }
    }
    cmd.clear();
    reply.clear();
    cmd.addString("getFace");
    DetectorPortOut.write(cmd,reply);
    if (reply.size() < 1) //>0
    {
        return "command failed";
    }
    else
    {
        if (reply.get(1).asDouble() != -1)
        {
            return "face";
        }
    }
    //yInfo()<<__LINE__;*/
}

bool SMModule::track(string trackedType)
{
    Bottle cmd, reply;
    if (trackedType == "face" || trackedType == "object")
    {
        cmd.addString("Track");
        cmd.addString(trackedType);
        TrackingPort.write(cmd,reply);
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
    


bool SMModule::getTargetBin()
{
    Bottle reply;
    RecogniserPort.write(*inImage, reply);
    if (reply.size() > 0)
    {
        if (reply.get(0).asList()->size() > 0)
        {
            targetBin = reply.get(0).asString();
            if (targetBin == "no prediction")
            {
                objectName = "";

                binPos[0] = -0.50;
                binPos[1] = 0.00;
                binPos[2] = -0.10;
                return false;
            }
            objectName = reply.get(1).asString();
 /*           
            binPos[0] = reply.get(0).asList()->get(0).asDouble();
            binPos[1] = reply.get(0).asList()->get(1).asDouble();
            binPos[2] = reply.get(0).asList()->get(2).asDouble();
*/
            if (targetBin == "aluminum")
            {
                binPos = bins[0];
            }
            if (targetBin == "plastic")
            {
                binPos = bins[1];
            }
            if (targetBin == "paper")
            {
                binPos = bins[2];
            }
        }
        else
        {
            binPos[0] = -0.50;
            binPos[1] = 0.00;
            binPos[2] = -0.10;
        }
    }   
    return false;
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


void SMModule::speak(const string &phrase)
{    
    yInfo()<<__LINE__; 
    yInfo()<<phrase;  
    // uncomment this when connecting to robot 
    /*    
    Bottle cmd, reply;
    cmd.clear();
    cmd.addString("say");
    cmd.addString(phrase);    
    if (SpeechPort.asPort().isOpen())
    {
        SpeechPort.write(cmd,reply);
    }
    */
}



bool SMModule::getBinImage()
{
    Bottle &cmd = DetectorPortOut.prepare();
    cmd.clear();
    cmd.addString("getCropImage");
    DetectorPortOut.write();
    while (true)
    {
        inImage= DetectorPortImage.read();
        if (inImage->width() > 0)
        {
            return true;
        }
        else 
        {
            return false;
        }
    }
    return true;
}

/*bool SMModule::objectInBin()
{
    if (objPos[0] < binPos[0]+0.105 && objPos[0] > binPos[0] - 0.105 && objPos[1] < binPos[1] + 0.1485 && objPos[1] > binPos[1] - 0.1485) // this is the size of the bin, would be nice to get from perception!!!!!!!!
    {
        return true;
    }
    else
    {
        return false;
    }
}*/

/*void SMModule::speak(string)
{

}*/

void SMModule::pushObject()
{
    Bottle cmd, reply;
    cmd.addString("Push");
    Bottle &objList = cmd.addList();
    objList.addDouble(objPos[0]);
    objList.addDouble(objPos[1]);
    objList.addDouble(objPos[2]);
    Bottle &binList = cmd.addList();
    binList.addDouble(binPos[0]);
    binList.addDouble(binPos[1]);
    binList.addDouble(binPos[2]);
    KinematicsPort.write(cmd,reply);
}

bool SMModule::updateModule() 
{
    switch(state)
    {
        case INITIALIZE_TABLE:
        {
            /*if (!initBins())
            {
                close();
                return false;
            }*/
            state = IDLE_STATE;
            break;
        }
        case IDLE_STATE:
        {
            string detectorOutput = "object"; // revert to queryDetector(); instead of "object";
            if(detectorOutput == "face")
            {
                state = TRACKING_FACE_STATE;
                shouldWait = true;
                speak("oh, I see a face");
                yInfo()<<"switch to TRACKING_FACE_STATE";
                
            }
            else if (detectorOutput == "object")
            {
                state = TRACKING_OBJECT_STATE;
                shouldWait = false;
                speak("oh, I see an object");
                yInfo()<<"switch to TRACKING_OBJECT_STATE";
            }
            else
            {
                yInfo()<<__LINE__;                
                shouldWait = true;
            }
            break;
        }
        case TRACKING_FACE_STATE:
        {
            track("face");
            shouldWait = true;

            break;        
        }
        case TRACKING_OBJECT_STATE:
        {
            track("object");
            shouldWait = false;
            state = RECOGNISE_OBJECT_STATE;
            yInfo()<<"switch to RECOGNISE_OBJECT_STATE";
            
            break;
        }
        case RECOGNISE_OBJECT_STATE:
        {
            if (!getBinImage())
            {
                state = IDLE_STATE;
                yInfo()<<"switch to IDLE_STATE";                
                break;
            }
            if (getTargetBin())
            {
                state = POINT_OBJECT_STATE;
                speak("please put it in this bin");
                yInfo()<<"switch to POINT_OBJECT_STATE";
            }
            else
            {
                state = IDLE_STATE;
                yInfo()<<"switch to IDLE_STATE";
            }

            break;
        }
        case POINT_OBJECT_STATE:
        {
            pointAtObject();
            state = REACT_STATE;
            yInfo()<<"switch to REACT_STATE";

            break;
        }
        case REACT_STATE:
        {
            state = PUSH_OBJECT_STATE;
            speak("okay, I will do it for you");
            yInfo()<<"switch to PUSH_OBJECT_STATE";

            break;
        }
        case PUSH_OBJECT_STATE:
        {
            pushObject();
            state = IDLE_STATE;
            speak("there you go");
            yInfo()<<"switch to IDLE_STATE";

            break;
        }
    }
    
    return true;
}


bool SMModule::respond(const Bottle& command, Bottle& reply) {
    yInfo()<<"Got something, echo is on";
    yInfo()<<command.toString();
    yInfo()<<"Current state is "<<state;
    if (command.get(0).asString()=="quit")
        return false;
    else if (command.get(0).asString() == "runModule")
    {
        yInfo()<<__LINE__;
        if (state == START_STATE) 
        {
            yInfo()<<__LINE__;            
            state = INITIALIZE_TABLE;  // this is the first state it should go to after START_STATE, not IDLE_STATE
            reply.addString("ack");
            return true;
        }
        else
        {
            yInfo()<<__LINE__; 
            reply.addString("nack");           
            return true;
        }
    }
    else if (command.get(0).asString() == "objOnTable")
    {
        int check = checkObjBin(command);
        if (check ==1)
        {
            state = IDLE_STATE;
            reply.addString("ack");
            return true;
        }
        if(check == 2)  
        {
            state = REACT_STATE;
            reply.addString("ack");
            return true;
        }
        if (check == 0)
        {
            state=IDLE_STATE;
            reply.addString("nack");
            return true;
        }
        state = IDLE_STATE; 
        return false;
    }


    else if (command.get(0).asString() == "getPos" && command.get(1).isString())
    {
        if (command.get(1).asString() == "object")
        {
            Bottle reply;
            reply.addDouble(objPos[0]);
            reply.addDouble(objPos[1]);
            reply.addDouble(objPos[2]);
            commandPort.write(reply);
        }
        else if (command.get(1).asString() == "face")
        {
            Bottle reply;
            reply.addDouble(facePos[0]);
            reply.addDouble(facePos[1]);
            reply.addDouble(facePos[2]);
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
    DetectorPortIn.interrupt();
    DetectorPortOut.interrupt();
    DetectorPortImage.interrupt();
    TrackingPort.interrupt();
    RecogniserPort.interrupt();
    SpeechPort.interrupt();
    return true;
}


bool SMModule::close() {
    yInfo()<<"closing State Machine module";
    commandPort.close();
    KinematicsPort.close();
    DetectorPortIn.interrupt();
    DetectorPortOut.interrupt();
    DetectorPortImage.interrupt();
    TrackingPort.close();
    RecogniserPort.close();
    SpeechPort.close();
    return true;
}


