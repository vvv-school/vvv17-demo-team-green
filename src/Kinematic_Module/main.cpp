/* 
 * Copyright (C) 2012 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Authors: Ugo Pattacini, Phuong Nguyen
 * emails:  <ugo.pattacini@iit.it>, <phuong.nguyen@iit.it>
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

/** 
\defgroup karmaMotor Motor Part of the KARMA Experiment
 
Motor Control Module that allows the robot to push/draw the 
object and explore a tool. 

\section intro_sec Description 
This module aims to control the robot hands in order to properly
execute the push and the draw actions of an object within the 
KARMA experiment to then learn the corresponding affordance. \n 
It also enable the tool exploration. 
 
\section lib_sec Libraries 
- YARP libraries. 

\section parameters_sec Parameters 
--robot \e robot
- Select the robot to connect to.

--name \e name
- Select the stem-name of the module used to open up ports. 
  By default \e name is <i>karmaMotor</i>.
 
--elbow_set <i>(<height> <weight>)</i>
- To specify how to weigh the task to keep the elbow high.
 
\section portsa_sec Ports Accessed
Assume that iCubInterface (with ICartesianControl interface
implemented) is running. 
 
\section portsc_sec Ports Created 
- \e /karmaMotor/rpc receives the information to execute the 
  motor action as a Bottle. It manages the following commands:
  -# <b>Push</b>: <i>[push] cx cy cz theta radius</i>. \n
  The coordinates <i>(cx,cy,cz)</i> represent in meters the
  position of the object's centroid to be pushed; <i>theta</i>,
  given in degrees, and <i>radius</i>, specified in meters,
  account for the point from which push the object, that is
  located onto the circle centered in <i>(cx,cy,cz)</i> and
  contained in the x-y plane. \n
  The reply <i>[ack]</i> is returned as soon as the push is
  accomplished.
  -# <b>Draw</b>: <i>[draw] cx cy cz theta radius dist</i>. \n
  The coordinates <i>(cx,cy,cz)</i> represent in meters the
  position of the object's centroid to be drawn closer;
  <i>theta</i>, given in degrees, and <i>radius</i>, specified
  in meters, account for the point from which draw the object,
  that is located onto the circle centered in <i>(cx,cy,cz)</i>
  and contained in the x-y plane. The parameter <i>dist</i>
  specifies the length in meters of the draw action. \n
  The reply <i>[ack]</i> is returned as soon as the draw is
  accomplished.
  -# <b>Virtual draw</b>: <i>[vdraw] cx cy cz theta radius
   dist</i>. \n Simulate the draw without performing any
   movement in order to test the quality of the action. \n
   The reply <i>[ack] val</i> is returned at the end of the
   simulation, where <i>val</i> accounts for the quality of the
   action: the lower it is the better the action is.
  -# <b>Tool-attach</b>: <i>[tool] [attach] arm x y z</i>. \n
  Attach a tool to the given arm whose dimensions are specified
  in the frame attached to the hand. The subsequent action will
  make use of this tool.
  -# <b>Tool-get</b>: <i>[tool] [get]</i>. \n
  Retrieve tool information as <i>[ack] arm x y z</i>.
  -# <b>Tool-remove</b>: <i>[tool] [remove]</i>. \n
  Remove the attached tool.
  -# <b>Find</b>: <i>[find] arm eye</i>. \n
  An exploration is performed which aims at finding the tool
  dimension. It is possible to select the arm for executing the
  movement as well as the eye from which the motion is observed.
  The reply <i>[ack] x y z</i> returns the tool's dimensions
  with respect to reference frame attached to the robot hand.
 
- \e /karmaMotor/stop:i receives request for immediate stop of
  any ongoing processing.
 
- \e /karmaMotor/vision:i receives the information about the 
  pixel corresponding to the tool tip during the tool
  exploration phase.
 
- \e /karmaMotor/finder:rpc communicates with the module in 
  charge of solving for the tool's dimensions.

- \e /KarmaMotor/reaching-supervisor/rpc:o sending planning and controlling
  request to *reaching-supervisor*
 
\section tested_os_sec Tested OS
Windows, Linux

\author Ugo Pattacini
\author Phuong Nguyen (constributor)
*/ 

#include <cstdio>
#include <string>
#include <cmath>
#include <algorithm>

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>

#include <iCub/ctrl/math.h>
#include <iCub/ctrl/minJerkCtrl.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::math;
using namespace iCub::ctrl;


/************************************************************************/
class KarmaMotor: public RFModule, public PortReader
{
protected:
    PolyDriver driverG;
    PolyDriver driverL;
    PolyDriver driverR;
    PolyDriver driverHL;
    PolyDriver driverHR;

    IGazeControl        *iGaze;
    ICartesianControl   *iCartCtrlL;
    ICartesianControl   *iCartCtrlR;
    ICartesianControl   *iCartCtrl;
    ICartesianControl   *iCartCtrlOther;

    IPositionControl2   *posCtrlR;
    IPositionControl2   *posCtrlL;
    IPositionControl2   *posCtrl;
    IEncoders           *encsR;
    IEncoders           *encsL;
    IEncoders           *encs;
    IControlMode2       *ctrlModeR;
    IControlMode2       *ctrlModeL;


    Vector encodersR, encodersL;
    Vector commandR, commandL, command;
    double timeActions;
    double handAngle;
    double safeMargin;
    double segL;    // segment length of push/pull action to avoid curved movement

    string pushHand;
    Matrix toolFrame;

    string handUsed;
    bool interrupting;
    double flip_hand;
    int shake_joint;

    bool elbow_set;
    double elbow_height,elbow_weight;

    BufferedPort<Bottle> visionPort;
    RpcClient            finderPort;
    RpcServer            rpcPort;
    Port                 stopPort;
    RpcClient            reachingPort;  // rpc port to connect to /reaching-supervisor/rpc:i

    /************************************************************************/
    double dist(const Matrix &M)
    {
        double ret=0.0;
        for (int r=0; r<M.rows(); r++)
            for (int c=0; c<M.cols(); c++)
                ret+=M(r,c)*M(r,c);

        return sqrt(ret);
    }

    /************************************************************************/
    bool read(ConnectionReader &connection)
    {
        Bottle cmd; cmd.read(connection);
        interruptModule();
        return true;
    }

    /************************************************************************/
    bool respond(const Bottle &command, Bottle &reply)
    {
        int ack=Vocab::encode("ack");
        int nack=Vocab::encode("nack");

        int cmd=command.get(0).asVocab();
        switch (cmd)
        {
            //-----------------
            case VOCAB4('p','u','s','h'):
            {
                Bottle payload=command.tail();
                if (payload.size()>=5)
                {
                    Vector c(3);
                    double theta;
                    double radius;

                    c[0]=payload.get(0).asDouble();
                    c[1]=payload.get(1).asDouble();
                    c[2]=payload.get(2).asDouble();
                    theta=payload.get(3).asDouble();
                    radius=payload.get(4).asDouble();

                    if (push(c,theta,radius,pushHand,toolFrame))
                        reply.addVocab(ack);
                    else
                        reply.addVocab(nack);
                }

                break;
            }

            //-----------------
            case VOCAB4('d','r','a','w'):
            case VOCAB4('v','d','r','a'):
            {
                Bottle payload=command.tail();
                if (payload.size()>=6)
                {
                    Vector c(3);
                    double theta;
                    double radius;
                    double dist;

                    c[0]=payload.get(0).asDouble();
                    c[1]=payload.get(1).asDouble();
                    c[2]=payload.get(2).asDouble();
                    theta=payload.get(3).asDouble();
                    radius=payload.get(4).asDouble();
                    dist=payload.get(5).asDouble();

                    double res=draw(cmd==VOCAB4('v','d','r','a'),c,theta,
                                    radius,dist,pushHand,toolFrame);

                    reply.addVocab(ack);
                    if (cmd==VOCAB4('v','d','r','a'))
                        reply.addDouble(res);
                }

                break;
            }

            //-----------------
            case VOCAB4('f','i','n','d'):
            {
                Bottle payload=command.tail();
                if (payload.size()>=2)
                {
                    string arm=payload.get(0).asString().c_str();
                    string eye=payload.get(1).asString().c_str();
                    Bottle solution;

                    if (findToolTip(arm,eye,solution))
                    {
                        reply.addVocab(ack);
                        reply.append(solution.tail());
                    }
                    else
                        reply.addVocab(nack);
                }

                break;
            }

            //-----------------
            case VOCAB4('t','o','o','l'):
            {
                if (command.size()>1)
                {
                    Bottle subcommand=command.tail();
                    int tag=subcommand.get(0).asVocab();
                    if (tag==Vocab::encode("attach"))
                    {
                        Bottle payload=subcommand.tail();
                        if (payload.size()>=4)
                        {
                            pushHand=payload.get(0).asString().c_str();

                            Vector point(4);
                            point[0]=payload.get(1).asDouble();
                            point[1]=payload.get(2).asDouble();
                            point[2]=payload.get(3).asDouble();
                            point[3]=1.0;

                            Vector r(4,0.0);
                            r[2]=-1.0;
                            r[3]=atan2(-point[1],point[0]);
                            toolFrame=axis2dcm(r);
                            toolFrame.setCol(3,point);

                            reply.addVocab(ack);
                        }
                    }
                    else if (tag==Vocab::encode("get"))
                    {
                        reply.addVocab(ack);
                        reply.addString(pushHand.c_str());
                        reply.addDouble(toolFrame(0,3));
                        reply.addDouble(toolFrame(1,3));
                        reply.addDouble(toolFrame(2,3));
                    }
                    else if (tag==Vocab::encode("remove"))
                    {
                        pushHand="selectable";
                        toolFrame=eye(4,4);

                        reply.addVocab(ack);
                    }
                }

                break;
            }

            //-----------------
            case VOCAB4('t','i','m','e'):
            {
                if (command.size()>1)
                {
                    Bottle subcommand=command.tail();
                    int tag=subcommand.get(0).asVocab();
                    if (tag==Vocab::encode("set"))
                    {
                        Bottle payload=subcommand.tail();
                        if (payload.size()>=1)
                        {
                            if (payload.get(0).asDouble()>=0.0)
                            {
                                timeActions = payload.get(0).asDouble();
                                reply.addVocab(ack);
                            }
                            else
                            {
                                reply.addVocab(nack);
                                reply.addString("motion time has to be positive!!!");
                            }
                        }
                    }
                    else if (tag==Vocab::encode("get"))
                    {
                        reply.addVocab(ack);
                        reply.addDouble(timeActions);
                    }
                }

                break;
            }

            //-----------------
            case VOCAB4('a','n','g','l'):
            {
                if (command.size()>1)
                {
                    Bottle subcommand=command.tail();
                    int tag=subcommand.get(0).asVocab();
                    if (tag==Vocab::encode("set"))
                    {
                        Bottle payload=subcommand.tail();
                        if (payload.size()>=1)
                        {
                            handAngle = payload.get(0).asDouble();
                            reply.addVocab(ack);
                        }
                    }
                    else if (tag==Vocab::encode("get"))
                    {
                        reply.addVocab(ack);
                        reply.addDouble(handAngle);
                    }
                }

                break;
            }

            //-----------------
            case VOCAB4('s','a','f','e'):
            {
                if (command.size()>1)
                {
                    Bottle subcommand=command.tail();
                    int tag=subcommand.get(0).asVocab();
                    if (tag==Vocab::encode("set"))
                    {
                        Bottle payload=subcommand.tail();
                        if (payload.size()>=1)
                        {
                            safeMargin = payload.get(0).asDouble();
                            reply.addVocab(ack);
                        }
                    }
                    else if (tag==Vocab::encode("get"))
                    {
                        reply.addVocab(ack);
                        reply.addDouble(safeMargin);
                    }
                }

                break;
            }

            //-----------------
            case VOCAB4('s','e','g','L'):
            {
                if (command.size()>1)
                {
                    Bottle subcommand=command.tail();
                    int tag=subcommand.get(0).asVocab();
                    if (tag==Vocab::encode("set"))
                    {
                        Bottle payload=subcommand.tail();
                        if (payload.size()>=1)
                        {
                            segL = payload.get(0).asDouble();
                            reply.addVocab(ack);
                        }
                    }
                    else if (tag==Vocab::encode("get"))
                    {
                        reply.addVocab(ack);
                        reply.addDouble(segL);
                    }
                }

                break;
            }

            //-----------------
            case VOCAB4('h','e','l','p'):
            {
                reply.addVocab(Vocab::encode("many"));
                reply.addString("Available commands are:");
                reply.addString("push - [push] cx cy cz theta radius - The object is pushed from the point at angle theta (deg) on the circle of radius (in m), centered at objects coordinates (cx,cy,cz) and contained in the x-y plane.");
                reply.addString("draw/pull - [draw]/[pull] cx cy cz theta radius dist - Pulls the object dist (in cm) from the point at angle theta (deg) on the circle of radius (in m), centered at objects coordinates (cx,cy,cz).");
                reply.addString("virtual draw/pull - [vdra]/[vpul] cx cy cz theta radius dist - Simulates the draw without performing any movement in order to test the quality of the action.");

                reply.addString("find - [find] arm eye - An exploration is performed which aims at finding the tool dimension.");
                reply.addString("tool-attach - [tool] [attach] arm x y z ");
                reply.addString("time set - [time] [set] time (s)");
                reply.addString("time get - [time] [get]");
                reply.addString("angle set - [hand] [set] alpha (deg), alpha is the angle to tilt the robot hand for pull action");
                reply.addString("angle get - [hand] [get]");
                reply.addString("safeMargin set - [safe] [set] r (m), r is the radius of the cylinder along z-axis to protect the robot when pulling");
                reply.addString("safeMargin get - [safe] [get]");
                reply.addString("segmentLength set - [segL] [set] L (m), L is the lenght of the a segment to divide the straight motion");
                reply.addString("segmentLength get - [segL] [get]");
                reply.addString("help - produces this help.");
                reply.addVocab(ack);
                break;
            }

            //-----------------
            default:
                interrupting=false;
                return RFModule::respond(command,reply);
        }

        interrupting=false;
        return true;
    }

    /************************************************************************/
    void initHandCtrl()
    {
        int nj;

        // Right hand
        posCtrlR->getAxes(&nj);
        encodersR.resize(nj);
        commandR.resize(nj);

        // Left hand
        posCtrlL->getAxes(&nj);
        encodersL.resize(nj);
        commandL.resize(nj);
    }

    /***************************************************************/
    bool moveHand(const string& armType, const Vector& cmd)
    {
        VectorOf<int> joints,modes;
        if (armType =="left")
        {
            for (size_t i=10; i<encodersL.length(); i++)
            {
                joints.push_back(i);
                modes.push_back(VOCAB_CM_POSITION);
            }

            ctrlModeL->setControlModes(joints.size(),joints.getFirst(),
                                       modes.getFirst());
            posCtrl = posCtrlL;
        }
        else
        {
            for (size_t i=10; i<encodersR.length(); i++)
            {
                joints.push_back(i);
                modes.push_back(VOCAB_CM_POSITION);
            }

            ctrlModeR->setControlModes(joints.size(),joints.getFirst(),
                                       modes.getFirst());
            posCtrl = posCtrlR;
        }

        Vector poss=cmd.subVector(10,cmd.length()-1);
        Vector accs(poss.length(),1e9); // we don't care since it simply limits the velocity
        Vector vels(poss.length(),50.0);

        posCtrl->setRefAccelerations(joints.size(),joints.getFirst(),accs.data());
        posCtrl->setRefSpeeds(joints.size(),joints.getFirst(),vels.data());
        posCtrl->positionMove(joints.size(),joints.getFirst(),poss.data());

        return true;
    }

    /************************************************************************/
    bool prepareHandForPull(const string &armType)
    {
        if (armType == "left")
        {
            yInfo("left hand");
            encsL->getEncoders(encodersL.data());
            command = encodersL;
        }
        else
        {
            yInfo("right hand");
            encsR->getEncoders(encodersR.data());
            command = encodersR;
        }

        if (command.size()>=16)
        {
            command[10] = 170.0;
            command[11] = 20.0;
            command[12] = 60.0;
            command[13] = 30.0;
            command[14] = 60.0;
            command[15] = 90.0;
        }

        yInfo("move fingers");
        return moveHand(armType,command);
    }

    /***************************************************************/
    bool restoreHand(const string& armType)
    {
        return moveHand(armType,(armType=="left"?encodersL:encodersR));
    }

    /***************************************************************/
    bool keepOtherArmSafe()
    {
        yInfo("Move other arm away for safety!!!");
        int contextOther;
        double zSafe = 0.2;
        Vector xCur(3,0.0), oCur(4,0.0);
        iCartCtrlOther->storeContext(&contextOther);
        iCartCtrlOther->setTrajTime(1.0);
        iCartCtrlOther->getPose(xCur,oCur);
        xCur[2] += zSafe;

        iCartCtrlOther->goToPose(xCur,oCur,1.0);
        iCartCtrlOther->restoreContext(contextOther);
        iCartCtrlOther->deleteContext(contextOther);

        return true;
    }

    /***************************************************************/
    void changeElbowHeight()
    {
        if (elbow_set)
        {
            Bottle tweakOptions; 
            Bottle &optTask2=tweakOptions.addList();
            optTask2.addString("task_2");
            Bottle &plTask2=optTask2.addList();
            plTask2.addInt(6);
            Bottle &posPart=plTask2.addList();
            posPart.addDouble(0.0);
            posPart.addDouble(0.0);
            posPart.addDouble(elbow_height);
            Bottle &weightsPart=plTask2.addList();
            weightsPart.addDouble(0.0);
            weightsPart.addDouble(0.0);
            weightsPart.addDouble(elbow_weight);
            iCartCtrl->tweakSet(tweakOptions);
        }
    }

    /************************************************************************/
    void setTaskVelocities(const Vector& xs, const Vector& xf)
    {
        double Ts=0.1;  // controller's sample time [s]
        double T=6.0;   // how long it takes to move to the target [s]
        double v_max=0.1;   // max cartesian velocity [m/s]

        // instantiate the controller
        minJerkVelCtrlForIdealPlant ctrl(Ts,xf.length());

        bool done=false;
        Vector dir=(xf-xs)/norm(xf-xs); // direction to the target
        while (!interrupting && !done)
        {
            Vector x,o;
            iCartCtrl->getPose(x,o);

            Vector e=xf-x;  // compute current distance to the target
            Vector vel_x=dir*ctrl.computeCmd(T,e);  // control upon the feedback

            // enforce velocity bounds
            for (size_t i=0; i<vel_x.length(); i++)
                vel_x[i]=sign(e[i])*std::min(v_max,fabs(vel_x[i]));

            // call the proper method
            iCartCtrl->setTaskVelocities(vel_x,Vector(4,0.0));
            Time::delay(Ts);

            done=(norm(e.subVector(0,1))<0.01);
            if (done)
                yDebug("xf= %s; x= %s",xf.toString(3,3).c_str(),x.toString(3,3).c_str());
        }

        iCartCtrl->stopControl();
    }

    /************************************************************************/
    double askPlannerToMove(const Vector& target, const double& localPlanningTime)
    {
        double timeOfExecution = -1.0;
        double replyWaitingTime = 15.0; // Time to wait for reply [s] this is due to planning time
        Bottle cmd,reply;
        cmd.addString("run_planner_pos");
        Bottle &pos = cmd.addList();
        for (int i=0; i<target.size(); i++)
        {
            pos.addDouble(target[i]);
            yDebug("[askPlannerToMove] target[%d]= %f",i,target[i]);
        }
        cmd.addDouble(localPlanningTime);
        if (reachingPort.write(cmd,reply))
        {
//            if (!reply.isNull())
//            {
//                timeOfExecution = reply.get(0).asDouble();
//            }
            double start = yarp::os::Time::now();
            double checkTime;
            do
            {
                yarp::os::Time::delay(0.1);
                checkTime = yarp::os::Time::now();
                yDebug("[askPlannerToMove] wait");
            }
            while (reply.isNull() && checkTime < 1.1*replyWaitingTime);
            timeOfExecution = reply.get(0).asDouble();
        }
        return timeOfExecution;
    }

    /************************************************************************/
    bool approachWithPlanner(const Vector &x)
    {
        bool done = false;
        double doneThreshold = 0.025;
        double timeCoef = 1.5;
        unsigned int trialTimeMax = 5;
        unsigned int trialCount;
        while (!interrupting && !done && !(trialCount>=trialTimeMax))   // This is to check the distance condition
        {
            yDebug("Testing communicate with supervisor");
            double approachTime = askPlannerToMove(x,0.1);
            yDebug("Approaching Time: %f",approachTime);
            // Use following as "waitMotionDone"
            if (approachTime>0.0)     // approachTime=-1.0 means planner fail, agent needs to ask another time of action
            {
                double start = yarp::os::Time::now();
                double checkTime;
                do
                {
                    yarp::os::Time::delay(0.1);
                    checkTime = yarp::os::Time::now();
                    yDebug("[karmaMotor] Time of %f(s): moving arm with reactCtrl",checkTime-start);
                }
                while (interrupting || checkTime-start<timeCoef*approachTime); // Time to finish motion should be consider longer than expected
            }
            else if (approachTime==-1.0)
            {
                return false;
            }
            Vector xs,os;
            iCartCtrl->getPose(xs,os);
            Vector e=x-xs;
            yDebug("e= %s, norm(e)= %f",e.toString().c_str(), norm(e));
            done = (norm(e)<=doneThreshold);
            if (done)
                yDebug("x= %s; xs= %s",x.toString(3,3).c_str(),xs.toString(3,3).c_str());
            trialCount++;
        }
        return done;
    }

    /************************************************************************/
    bool push(const Vector &c, const double theta, const double radius,
              const string &armType="selectable", const Matrix &frame=eye(4,4))
    {
        // wrt root frame: frame centered at c with x-axis pointing rightward,
        // y-axis pointing forward and z-axis pointing upward
        Matrix H0(4,4); H0.zero();
        H0(1,0)=1.0;
        H0(0,1)=-1.0;
        H0(2,2)=1.0;
        H0(0,3)=c[0]; H0(1,3)=c[1]; H0(2,3)=c[2]; H0(3,3)=1.0;

        double theta_rad=CTRL_DEG2RAD*theta;
        double _c=cos(theta_rad);
        double _s=sin(theta_rad);
        double _theta=CTRL_RAD2DEG*atan2(_s,_c);    // to have theta in [-180.0,180.0]
        double epsilon=0.02;

        // wrt H0 frame: frame centered at R*[_c,_s] with z-axis pointing inward
        // and x-axis tangential
        Matrix H1(4,4); H1.zero();
        H1(0,0)=-_s;       H1(1,0)=_c;
        H1(2,1)=-1.0;
        H1(0,2)=-_c;       H1(1,2)=-_s;
        H1(0,3)=radius*_c; H1(1,3)=radius*_s; H1(3,3)=1.0;

        // wrt H0 frame: frame centered at R*[_c,_s] with z-axis pointing outward
        // and x-axis tangential
        Matrix H2(4,4); H2.zero();
        H2(0,0)=_s;        H2(1,0)=-_c;
        H2(2,1)=-1.0;
        H2(0,2)=_c;        H2(1,2)=_s;
        H2(0,3)=radius*_c; H2(1,3)=radius*_s; H2(3,3)=1.0;

        // matrices that serve to account for pushing with the back of the hand
        Matrix H1eps=H1; Matrix H2eps=H2;
        H1eps(0,3)+=epsilon*_c; H1eps(1,3)+=epsilon*_s;
        H2eps(0,3)+=epsilon*_c; H2eps(1,3)+=epsilon*_s;
        
        // go back into root frame and apply tool (if any)
        Matrix invFrame=SE3inv(frame);
        H1=H0*H1*invFrame;
        H2=H0*H2*invFrame;
        H1eps=H0*H1eps*invFrame;
        H2eps=H0*H2eps*invFrame;
        
        Vector xd1=H1.getCol(3).subVector(0,2);
        Vector od1=dcm2axis(H1);

        Vector xd2=H2.getCol(3).subVector(0,2);
        Vector od2=dcm2axis(H2);

        Vector xd1eps=H1eps.getCol(3).subVector(0,2);
        Vector od1eps=dcm2axis(H1eps);

        Vector xd2eps=H2eps.getCol(3).subVector(0,2);
        Vector od2eps=dcm2axis(H2eps);

        yInfo("identified locations...");
        yInfo("xd1=(%s) od1=(%s)",xd1.toString(3,3).c_str(),od1.toString(3,3).c_str());
        yInfo("xd2=(%s) od2=(%s)",xd2.toString(3,3).c_str(),od2.toString(3,3).c_str());

        // choose the arm
        if (armType=="selectable")
        {
            if (xd1[1]>=0.0)
            {
                iCartCtrl = iCartCtrlR;
                iCartCtrlOther = iCartCtrlL;
            }
            else
            {
                iCartCtrl = iCartCtrlL;
                iCartCtrlOther = iCartCtrlR;
            }
        }
        else if (armType=="left")
        {
            iCartCtrl=iCartCtrlL;
            iCartCtrlOther = iCartCtrlR;
        }
        else
        {
            iCartCtrl=iCartCtrlR;
            iCartCtrlOther = iCartCtrlL;
        }

        // deal with the arm context
        int context;
        iCartCtrl->storeContext(&context);

        Bottle options;
        Bottle &straightOpt=options.addList();
        straightOpt.addString("straightness");
        straightOpt.addDouble(10.0);
        iCartCtrl->tweakSet(options);
        changeElbowHeight();

        Vector dof;
        iCartCtrl->getDOF(dof);
        
        dof=1.0; dof[1]=0.0;
        iCartCtrl->setDOF(dof,dof);

        Vector xdhat1,odhat1,xdhat2,odhat2;
        Vector dummy;

        // try out different poses
        iCartCtrl->askForPose(xd1,od1,xdhat1,odhat1,dummy);
        iCartCtrl->askForPose(xd2,od2,xdhat2,odhat2,dummy);

        Matrix Hhat1=axis2dcm(odhat1); Hhat1(0,3)=xdhat1[0]; Hhat1(1,3)=xdhat1[1]; Hhat1(2,3)=xdhat1[2];
        Matrix Hhat2=axis2dcm(odhat2); Hhat2(0,3)=xdhat2[0]; Hhat2(1,3)=xdhat2[1]; Hhat2(2,3)=xdhat2[2];

        double d1=dist(H1-Hhat1);
        double d2=dist(H2-Hhat2);

        yInfo("solutions...");
        yInfo("#1: xdhat1=(%s) odhat1=(%s); e=%.3f",xdhat1.toString(3,3).c_str(),odhat1.toString(3,3).c_str(),d1);
        yInfo("#2: xdhat2=(%s) odhat2=(%s); e=%.3f",xdhat2.toString(3,3).c_str(),odhat2.toString(3,3).c_str(),d2);
        yInfo("selection: ");

        // compare solutions and choose the best
        Vector *xd,*od;
        if (fabs(_theta-90.0)<45.0)
        {
            yInfo("(detected singularity) ");
            if (iCartCtrl==iCartCtrlR)
            {
                xd=&xd1;
                od=&od1;
            }
            else
            {
                xd=&xd2;
                od=&od2;
            }
        }
        else if (fabs(_theta+90.0)<45.0)
        {
            yInfo("(detected singularity) ");
            if (iCartCtrl==iCartCtrlR)
            {
                xd=&xd2;
                od=&od2;
            }
            else
            {
                xd=&xd1;
                od=&od1;
            }
        }
        else if (d1<d2)
        {
            xd=&xd1;
            od=&od1;
        }
        else
        {
            xd=&xd2;
            od=&od2;
        }

        if (xd==&xd1)
            yInfo("#1 ");
        else
            yInfo("#2 ");

        if ((iCartCtrl==iCartCtrlR) && (_theta<0.0) && (xd==&xd2))
        {
            yInfo("(increased radius)");
            xd=&xd2eps;
            od=&od2eps;
        }
        else if ((iCartCtrl==iCartCtrlL) && (_theta<0.0) && (xd==&xd1))
        {
            yInfo("(increased radius)");
            xd=&xd1eps;
            od=&od1eps;
        }

        yInfo(": xd=(%s); od=(%s)",xd->toString(3,3).c_str(),od->toString(3,3).c_str());

        Vector xTemp = *xd;
        double dist_xd = sqrt(xTemp[0]*xTemp[0] + xTemp[1]*xTemp[1]);
        yInfo("distance from xd to center %f", dist_xd);
        bool resPush;
        if (dist_xd>safeMargin && xTemp[0]<=0.0)
        {
            // execute the movement
            Vector offs(3,0.0); offs[2]=0.1;
            if (!interrupting)
            {
                Vector x=*xd+offs;

                keepOtherArmSafe();
                yInfo("moving to: x=(%s); o=(%s)",x.toString(3,3).c_str(),od->toString(3,3).c_str());

                // Use for left arm with pushing left only
                if (iCartCtrl==iCartCtrlL && theta==180)
                {
                    if (!approachWithPlanner(x))
                        return false;
                }

                // Use for right arm only
                else //if (iCartCtrl==iCartCtrlR)
                {
                    iCartCtrl->goToPoseSync(x,*od,timeActions);
                    iCartCtrl->waitMotionDone(0.1,4.0);
                }
            }
            // Going down to initial position for pushing
            if (!interrupting)
            {
                yInfo("moving to: x=(%s); o=(%s)",xd->toString(3,3).c_str(),od->toString(3,3).c_str());
                iCartCtrl->goToPoseSync(*xd,*od,timeActions);
                iCartCtrl->waitMotionDone(0.1,4.0);
            }

            Matrix H=axis2dcm(*od);
            Vector center=c; center.push_back(1.0);
            H.setCol(3,center);
            Vector x=-1.0*frame.getCol(3); x[3]=1.0;
            x=H*x; x.pop_back();

            setTaskVelocities(*xd,x.subVector(0,2));
            resPush = true;
        }
        else
        {
            yWarning()<<" [karmaMotor] It is not safe to conduct the action. Quit!!!";
            resPush = false;
        }
        
        iCartCtrl->restoreContext(context);
        iCartCtrl->deleteContext(context);
        return resPush;
    }

    /************************************************************************/
    double draw(bool simulation, const Vector &c, const double theta, const double radius,
                const double dist, const string &armType, const Matrix &frame=eye(4,4))
    {
        // c0 is the projection of c on the sagittal plane
        Vector c_sag=c;
        c_sag[1]=0.0;

        // wrt root frame: frame centered at c_sag with x-axis pointing rightward,
        // y-axis pointing forward and z-axis pointing upward
        Matrix H0(4,4); H0.zero();
        H0(1,0)=1.0;
        H0(0,1)=-1.0;
        H0(2,2)=1.0;
        H0(0,3)=c_sag[0]; H0(1,3)=c_sag[1]; H0(2,3)=c_sag[2]; H0(3,3)=1.0;

        double theta_rad=CTRL_DEG2RAD*theta;
        double _c=cos(theta_rad);
        double _s=sin(theta_rad);

        // wrt H0 frame: frame translated in R*[_c,_s]
        Matrix H1=eye(4,4);
        H1(0,3)=radius*_c; H1(1,3)=radius*_s;

        // wrt H1 frame: frame translated in [0,-dist]
        Matrix H2=eye(4,4);
        H2(1,3)=-dist;

        // convert (go back) into root frame
        H2=H0*H1*H2;
        H1=H0*H1;

        // apply final axes
        Matrix R(3,3); R.zero();
        R(0,0)=-1.0;
        R(2,1)=-1.0;
        R(1,2)=-1.0;

        H1.setSubmatrix(R,0,0);
        H2.setSubmatrix(R,0,0);

        Vector xd1=H1.getCol(3).subVector(0,2);
        Vector od1=dcm2axis(H1);

        Vector xd2=H2.getCol(3).subVector(0,2);
        Vector od2=dcm2axis(H2);

        yInfo("identified locations on the sagittal plane...");
        yInfo("xd1=(%s) od1=(%s)",xd1.toString(3,3).c_str(),od1.toString(3,3).c_str());
        yInfo("xd2=(%s) od2=(%s)",xd2.toString(3,3).c_str(),od2.toString(3,3).c_str());

        yInfo("armType: %s", armType.c_str());
        yInfo("pushHand: %s", pushHand.c_str());
        // choose the arm
        string pushHand0 = pushHand;
        if (armType=="selectable")
        {
            double yObj = c[1]+radius*_c;
            yInfo("yObj = %f",yObj);
            if (yObj>=0)
            {
                iCartCtrl = iCartCtrlR;
                iCartCtrlOther = iCartCtrlL;
                pushHand = "right";
            }
            else
            {
                iCartCtrl=iCartCtrlL;
                iCartCtrlOther = iCartCtrlR;
                pushHand = "left";
            }
        }
        else if (armType=="left")
        {
            iCartCtrl = iCartCtrlL;
            iCartCtrlOther = iCartCtrlR;
        }
        else if (armType=="right")
        {
            iCartCtrl = iCartCtrlR;
            iCartCtrlOther = iCartCtrlL;
        }
        yInfo("armType: %s", armType.c_str());
        yInfo("pushHand: %s", pushHand.c_str());

        // recover the original place: do translation and rotation
        if (c[1]!=0.0)
        {
            Vector r(4,0.0);
            r[2]=-1.0;
            r[3]=atan2(c[1],fabs(c[0]));
            Matrix H=axis2dcm(r);

            H(0,3)=H1(0,3);
            H(1,3)=H1(1,3)+c[1];
            H(2,3)=H1(2,3);
            H1(0,3)=H1(1,3)=H1(2,3)=0.0;
            H1=H*H1;

            H(0,3)=H2(0,3);
            H(1,3)=H2(1,3)+c[1];
            H(2,3)=H2(2,3);
            H2(0,3)=H2(1,3)=H2(2,3)=0.0;
            H2=H*H2;

            xd1=H1.getCol(3).subVector(0,2);
            od1=dcm2axis(H1);

            xd2=H2.getCol(3).subVector(0,2);
            od2=dcm2axis(H2);
        }

        // Safe pulling
        double dist_xd2 = sqrt(xd2[0]*xd2[0] + xd2[1]*xd2[1]);
        yInfo("distance from xd2 to center %f", dist_xd2);
        if (dist_xd2<safeMargin)
        {
            xd2[0] = sign(c[0]) * sqrt(safeMargin*safeMargin - xd2[1]*xd2[1]);
        }

        yInfo("in-place locations...");
        yInfo("xd1=(%s) od1=(%s)",xd1.toString(3,3).c_str(),od1.toString(3,3).c_str());
        yInfo("xd2=(%s) od2=(%s)",xd2.toString(3,3).c_str(),od2.toString(3,3).c_str());

        // Rotate the wrist and change the fingers' angles
        yInfo("prepare the hand for pulling with *%s* hand...",pushHand.c_str());
        prepareHandForPull(pushHand);

        // deal with the arm context
        int context;
        iCartCtrl->storeContext(&context);

        Matrix Htip(4,4); Htip.zero();

        double sign_handAngle;
        // The tip frame should be considered wrt the original End-Effector (hand) FoR
        // Make the palm of the hand parallel to to table
        if (pushHand == "left")
        {
            Htip(0,0) =  1.0;
            Htip(2,1) = -1.0;
            Htip(1,2) =  1.0;

            sign_handAngle = -1.0;
        }
        else if (pushHand == "right")
        {
            Htip(0,0) =  1.0;
            Htip(2,1) =  1.0;
            Htip(1,2) = -1.0;

            sign_handAngle = 1.0;
        }

        Htip(3,3) = 1.0;

        Vector handRot0(4,0.0);
        handRot0[1] = 1.0; handRot0[3] = sign_handAngle * handAngle*M_PI/180.0; //Tilt hand 'handAngle' degrees
        Matrix R0 = axis2dcm(handRot0);

        Vector handRot1(4,0.0);
        handRot1[2] = 1.0; handRot1[3] = 30*M_PI/180.0; //Rotate hand 30 degrees of yaw
        Matrix R1 = axis2dcm(handRot1);

        Htip = R0*R1*Htip;

        Htip(0,3) = 0.05; Htip(1,3) = 0.0; Htip(2,3) = 0.025; // New tool tip position wrt the original hand FoR

        Vector tip_x = Htip.getCol(3).subVector(0,2);
        Vector tip_o = dcm2axis(Htip);

        iCartCtrl->attachTipFrame(tip_x,tip_o);                // establish the new controlled frame

        Vector tip_x_out, tip_o_out;
        iCartCtrl->getTipFrame(tip_x_out,tip_o_out);
        yInfo("tip_x=(%s) tip_o=(%s)",tip_x.toString(3,3).c_str(),tip_o.toString(3,3).c_str());
        yInfo("tip_x_out=(%s) tip_o_out=(%s)",tip_x_out.toString(3,3).c_str(),tip_o_out.toString(3,3).c_str());

        Bottle options;
        Bottle &straightOpt=options.addList();
        straightOpt.addString("straightness");
        straightOpt.addDouble(30.0);
        iCartCtrl->tweakSet(options);
        changeElbowHeight();

        Vector dof;
        iCartCtrl->getDOF(dof);

        dof=1.0; dof[1]=0.0;
        iCartCtrl->setDOF(dof,dof);
        
        double res=0.0;

        // simulate the movements
        if (simulation)
        {
            Vector xdhat1,odhat1,xdhat2,odhat2,qdhat;
            iCartCtrl->askForPose(xd1,od1,xdhat1,odhat1,qdhat);
            bool canPull = iCartCtrl->askForPose(qdhat,xd2,od2,xdhat2,odhat2,qdhat);

            double e_x1=norm(xd1-xdhat1);
            double e_o1=norm(od1-odhat1);
            yInfo("testing x=(%s); o=(%s) => xhat=(%s); ohat=(%s) ... |e_x|=%g; |e_o|=%g",
                   xd1.toString(3,3).c_str(),od1.toString(3,3).c_str(),
                   xdhat1.toString(3,3).c_str(),odhat1.toString(3,3).c_str(),
                   e_x1,e_o1);

            double e_x2=norm(xd2-xdhat2);
            double e_o2=norm(od2-odhat2);
            yInfo("testing x=(%s); o=(%s) => xhat=(%s); ohat=(%s) ... |e_x|=%g; |e_o|=%g",
                   xd2.toString(3,3).c_str(),od2.toString(3,3).c_str(),
                   xdhat2.toString(3,3).c_str(),odhat2.toString(3,3).c_str(),
                   e_x2,e_o2);

            double nearness_penalty=((norm(xdhat1)<0.15)||(norm(xdhat2)<0.15)?10.0:0.0);
            yInfo("nearness penalty=%g",nearness_penalty);
            res=e_x1+e_o1+e_x2+e_o2+nearness_penalty;
            yInfo("final quality=%g",res);
            string pullTest = (canPull) ? "true" : "false";
            yInfo("Can pull: %s",pullTest.c_str());
        }
        // execute the movements
        else
        {
            Vector offs(3,0.0); offs[2]=0.1;
            if (!interrupting)
            {
                Vector x=xd1+offs;

                keepOtherArmSafe();
                yInfo("moving to: x=(%s); o=(%s)",x.toString(3,3).c_str(),od1.toString(3,3).c_str());
                iCartCtrl->goToPoseSync(x,od1,2.0);
                iCartCtrl->waitMotionDone(0.1,5.0);
            }

            // Going down to initial position for pulling
            if (!interrupting)
            {
                yInfo("moving to: x=(%s); o=(%s)",xd1.toString(3,3).c_str(),od1.toString(3,3).c_str());
                iCartCtrl->goToPoseSync(xd1,od1,1.5);
                iCartCtrl->waitMotionDone(0.1,5.0);
            }

            setTaskVelocities(xd1,xd2);
        }

        iCartCtrl->restoreContext(context);
        iCartCtrl->deleteContext(context);

        restoreHand(pushHand);
        pushHand = pushHand0;

        return res;
    }

    /************************************************************************/
    void shakeHand()
    {
        IEncoders        *ienc;
        IVelocityControl *ivel;

        if (handUsed=="left")
        {
            driverHL.view(ienc);
            driverHL.view(ivel);
        }
        else
        {
            driverHR.view(ienc);
            driverHR.view(ivel);
        }

        double pos;
        ienc->getEncoder(shake_joint,&pos);

        double e=flip_hand-pos;
        if ((flip_hand>0.0) && (e<0.0) ||
            (flip_hand<0.0) && (e>0.0))
        {
            flip_hand=-flip_hand;
            e=flip_hand-pos;
        }

        ivel->velocityMove(shake_joint,120.0*sign(e));
    }

    /************************************************************************/
    void stopHand(const string &hand)
    {
        IVelocityControl *ivel;
        if (hand=="left")
            driverHL.view(ivel);
        else
            driverHR.view(ivel);

        ivel->stop(4);
    }

    /************************************************************************/
    void moveTool(const string &arm, const string &eye, const Vector &xd, const Vector &od,
                  const Vector &xOffset, const int maxItems)
    {
        iGaze->restoreContext(0);
        
        if (!interrupting)
        {
            iGaze->setTrackingMode(true);
            iGaze->lookAtFixationPoint(xd+xOffset);
            iCartCtrl->goToPoseSync(xd,od,1.0);
            iCartCtrl->waitMotionDone(0.1);
        }

        iGaze->setSaccadesMode(false);
        iGaze->setNeckTrajTime(2.5);
        iGaze->setEyesTrajTime(1.5);

        // put the shaking joint in velocity mode
        IControlMode2 *imode;
        if (arm=="left")
            driverHL.view(imode);
        else
            driverHR.view(imode);
        imode->setControlMode(shake_joint,VOCAB_CM_VELOCITY);
        handUsed=arm;   // this triggers the hand shaking

        // gaze robustly at the tool tip
        Vector pxCum(2,0.0);
        int cnt=0; bool done=false;
        double t0=Time::now();
        while (!interrupting && !done)
        {
            double t1=Time::now();
            if (Bottle *target=visionPort.read(false))
            {
                if (target->size()>=2)
                {
                    Vector px(2);
                    px[0]=target->get(0).asDouble();
                    px[1]=target->get(1).asDouble()+50.0;
                    iGaze->lookAtMonoPixel(eye=="left"?0:1,px);

                    pxCum+=px;
                    cnt++;
                }
            }

            if (t1-t0>=3.0)
            {
                if (cnt>20)
                    done=fabs(pxCum[1]/cnt-120)<30.0;

                pxCum=0.0;
                cnt=0;
                t0=t1;
            }

            Time::delay(0.02);
        }

        // gather sufficient information
        Bottle command,reply;
        command.addVocab(Vocab::encode("enable"));
        finderPort.write(command,reply);

        command.clear();
        command.addVocab(Vocab::encode("num"));
        finderPort.write(command,reply);
        int curItems=reply.get(1).asInt();

        int nItems=0;
        while (!interrupting && (nItems<curItems+maxItems))
        {
            finderPort.write(command,reply);
            nItems=reply.get(1).asInt();

            if (Bottle *target=visionPort.read(false))
            {
                if (target->size()>=2)
                {
                    Vector px(2);
                    px[0]=target->get(0).asDouble();
                    px[1]=target->get(1).asDouble()+50.0;
                    iGaze->lookAtMonoPixel(eye=="left"?0:1,px);
                }
            }

            Time::delay(0.1);
        }

        command.clear();
        command.addVocab(Vocab::encode("disable"));
        finderPort.write(command,reply);

        handUsed="null";
        stopHand(arm);
    }

    /************************************************************************/
    bool findToolTip(const string &arm, const string &eye, Bottle &reply)
    {
        if (arm=="left")
            iCartCtrl=iCartCtrlL;
        else if (arm=="right")
            iCartCtrl=iCartCtrlR;
        else
            return false;

        int context_arm,context_gaze;
        iCartCtrl->storeContext(&context_arm);
        iGaze->storeContext(&context_gaze);

        Vector dof;
        iCartCtrl->getDOF(dof);
        dof=1.0; dof[0]=dof[1]=0.0;
        iCartCtrl->setDOF(dof,dof);

        Bottle command;
        command.addVocab(Vocab::encode("clear"));
        finderPort.write(command,reply);

        // solving
        command.clear();
        command.addVocab(Vocab::encode("select"));
        command.addString(arm.c_str());
        command.addString(eye.c_str());
        finderPort.write(command,reply);

        Matrix R(4,4);
        R(0,0)=-1.0;
        R(2,1)=-1.0;
        R(1,2)=-1.0;
        R(3,3)=+1.0;
        Vector r(4,0.0); r[0]=-1.0;
        Vector xd(3,0.0),od;
        Vector offset(3,0.0); offset[2]=0.1;

        // point 1
        r[3]=0.0;
        od=dcm2axis(axis2dcm(r)*R);
        xd[0]=-0.35;
        shake_joint=4;
        moveTool(arm,eye,xd,od,offset,25);

        // point 2
        r[3]=CTRL_DEG2RAD*(arm=="left"?30.0:-30.0);
        od=dcm2axis(axis2dcm(r)*R);
        xd[1]=(arm=="left")?-0.15:0.15;
        offset[1]=(arm=="left")?0.1:-0.1;
        moveTool(arm,eye,xd,od,offset,25);

        // point 3
        r[3]=CTRL_DEG2RAD*(arm=="left"?20.0:-20.0);
        od=dcm2axis(axis2dcm(r)*R);
        xd[2]=0.15;
        offset[1]=(arm=="left")?0.2:-0.2;
        offset[2]=0.1;
        moveTool(arm,eye,xd,od,offset,25);

        // point 4
        r[3]=CTRL_DEG2RAD*(arm=="left"?10.0:-10.0);
        od=dcm2axis(axis2dcm(r)*R);
        xd[0]=-0.3;
        xd[1]=(arm=="left")?-0.05:0.05;
        xd[2]=-0.05;
        moveTool(arm,eye,xd,od,offset,25);

        // point 5
        r[3]=CTRL_DEG2RAD*(arm=="left"?45.0:-45.0);
        od=dcm2axis(axis2dcm(r)*R);
        xd[0]=-0.35;
        xd[1]=(arm=="left")?-0.05:0.05;
        xd[2]=0.1;
        offset[1]=(arm=="left")?0.1:-0.1;
        moveTool(arm,eye,xd,od,offset,25);

        // point 6
        xd[0]=-0.35;
        xd[1]=(arm=="left")?-0.1:0.1;
        xd[2]=0.0;
        Vector r1(4,0.0); r1[2]=(arm=="left")?-1.0:1.0; r1[3]=CTRL_DEG2RAD*45.0;
        Vector r2(4,0.0); r2[0]=(arm=="left")?1.0:-1.0; r2[3]=CTRL_DEG2RAD*45.0;
        od=dcm2axis(axis2dcm(r2)*axis2dcm(r1)*R);
        offset[0]=0;
        offset[1]=(arm=="left")?-0.05:0.05;
        offset[2]=0.1;
        shake_joint=6;
        moveTool(arm,eye,xd,od,offset,50);

        // solving
        command.clear();
        command.addVocab(Vocab::encode("find"));
        finderPort.write(command,reply);

        iCartCtrl->restoreContext(context_arm);
        iCartCtrl->deleteContext(context_arm);

        iGaze->restoreContext(context_gaze);
        iGaze->deleteContext(context_gaze);

        return true;
    }

public:
    /************************************************************************/
    bool configure(ResourceFinder &rf)
    {
        string name=rf.check("name",Value("karmaMotor")).asString().c_str();
        string robot=rf.check("robot",Value("icub")).asString().c_str();
        elbow_set=rf.check("elbow_set");
        if (elbow_set)
        {
            if (Bottle *pB=rf.find("elbow_set").asList())
            {
                elbow_height=pB->get(0).asDouble();
                elbow_weight=pB->get(1).asDouble();
            }
            else
            {
                elbow_height=0.4;
                elbow_weight=30.0;
            }
        }

        Property optionG("(device gazecontrollerclient)");
        optionG.put("remote","/iKinGazeCtrl");
        optionG.put("local",("/"+name+"/gaze_ctrl").c_str());

        Property optionL("(device cartesiancontrollerclient)");
        optionL.put("remote",("/"+robot+"/cartesianController/left_arm").c_str());
        optionL.put("local",("/"+name+"/cart_ctrl/left_arm").c_str());

        Property optionR("(device cartesiancontrollerclient)");
        optionR.put("remote",("/"+robot+"/cartesianController/right_arm").c_str());
        optionR.put("local",("/"+name+"/cart_ctrl/right_arm").c_str());

        Property optionHL("(device remote_controlboard)");
        optionHL.put("remote",("/"+robot+"/left_arm").c_str());
        optionHL.put("local",("/"+name+"/hand_ctrl/left_arm").c_str());

        Property optionHR("(device remote_controlboard)");
        optionHR.put("remote",("/"+robot+"/right_arm").c_str());
        optionHR.put("local",("/"+name+"/hand_ctrl/right_arm").c_str());

        if (!driverG.open(optionG))
            return false;

        if (!driverL.open(optionL))
        {
            driverG.close();
            return false;
        }

        if (!driverR.open(optionR))
        {
            driverG.close();
            driverL.close();
            return false;
        }

        if (!driverHL.open(optionHL))
        {
            driverG.close();
            driverL.close();
            driverR.close();
            return false;
        }

        if (!driverHR.open(optionHR))
        {
            driverG.close();
            driverL.close();
            driverR.close();
            driverHL.close();
            return false;
        }

        driverG.view(iGaze);
        driverL.view(iCartCtrlL);
        driverR.view(iCartCtrlR);

        // New update
        bool okR;
        okR = driverHR.view(posCtrlR);
        okR = okR && driverHR.view(encsR);
        okR = okR && driverHR.view(ctrlModeR);

        if (!okR) {
            yInfo("Problems acquiring interfaces with right hand");
            return 0;
        }

        bool okL;
        okL = driverHL.view(posCtrlL);
        okL = okL && driverHL.view(encsL);
        okL = okL && driverHL.view(ctrlModeL);

        if (!okL) {
            yInfo("Problems acquiring interfaces with left hand");
            return 0;
        }

        initHandCtrl();

        visionPort.open(("/"+name+"/vision:i").c_str());
        finderPort.open(("/"+name+"/finder:rpc").c_str());
        rpcPort.open(("/"+name+"/rpc").c_str());
        stopPort.open(("/"+name+"/stop:i").c_str());
        attach(rpcPort);
        stopPort.setReader(*this);
        reachingPort.open(("/"+name+"/reaching-supervisor/rpc:o").c_str());

        // TODO: Move to script file later!!!!
//        string portSupervisor = "/reaching-supervisor/rpc:i";

//        if (yarp::os::Network::connect(reachingPort.getName(),portSupervisor.c_str()))
//            yInfo("[karmaMotor] connected to reaching-supervisor");
//        else
//            yWarning("[karmaMotor] didn't connect to reaching-supervisor");

        interrupting=false;
        handUsed="null";
        flip_hand=6.0;

        pushHand="selectable";
        toolFrame=eye(4,4);

        timeActions = 1.5;
        handAngle   = 15.0;
        safeMargin  = 0.25; // 25cm
        segL        = 0.03; //  3cm

        return true;
    }

    /************************************************************************/
    bool interruptModule()
    {
        interrupting=true;

        iGaze->stopControl();
        iCartCtrlL->stopControl();
        iCartCtrlR->stopControl();

        if (handUsed!="null")
        {
            stopHand("left");
            stopHand("right");
        }

        return true;
    }

    /************************************************************************/
    bool close()
    {
        visionPort.close();
        finderPort.close();
        rpcPort.close();
        stopPort.close();   // close prior to shutting down motor-interfaces
        reachingPort.close();

        driverG.close();
        driverL.close();
        driverR.close();
        driverHL.close();
        driverHR.close();
        return true;
    }

    /************************************************************************/
    double getPeriod()
    {
        return 0.02;
    }

    /************************************************************************/
    bool updateModule()
    {
        if (!interrupting && (handUsed!="null"))
            shakeHand();

        return true;
    }
};


/************************************************************************/
int main(int argc, char *argv[])
{
    Network yarp;
    if (!yarp.checkNetwork())
    {
        yInfo("YARP server not available!");
        return 1;
    }

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.configure(argc,argv);

    KarmaMotor karmaMotor;
    return karmaMotor.runModule(rf);
}



