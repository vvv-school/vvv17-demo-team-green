#include <string>

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;


double maxRange = 0.45;
double minRange = 0.35;
double maxLength = 0.45;
double minLength = 0.25;
double height = 0.10;

/*****************************************************/
class CtrlModule: public RFModule
{
protected:
    PolyDriver drvArmR, drvArmL, drvGaze;
    PolyDriver drvHandR, drvHandL;
    ICartesianControl *iarm;
    IGazeControl      *igaze;
    int startup_ctxt_arm_right;
    int startup_ctxt_arm_left;
    int startup_ctxt_gaze;

    Vector target, binLoc;
    RpcServer rpcPort;
    double tableHigh;


    /***************************************************/
    Vector computePosHandPoint(const Vector &x)
    {

        Vector xnew = x;


        if (xnew[1]>-0.01 && xnew[1]< 0.01)
        {
            xnew[0] = max(-maxRange, xnew[0]);
            xnew[0] = min(-minRange, xnew[0]);
            xnew[1] = 0.01;
        }

        double A = norm2(xnew.subVector(0,1));
        double alpha = atan(x[0]/fabs(x[1]));
        yDebug()<<"alpha "<<alpha*180.0/M_PI;


        if (A > maxRange )
        {
            xnew[0]= maxRange*sin(alpha);
            xnew[1]= sign(x[1]) * maxRange*cos(alpha);
        }
        else if(A < minRange)
        {
            xnew[0]= minRange*sin(alpha);
            xnew[1]= sign(x[1]) * minRange*cos(alpha);

        }

        xnew[2]=height;

        return xnew;
    }
		
    /****************************************************/
    Vector computeHandOrientationPoint(const Vector &x, const string &hand)
    {
        Matrix Rot(3,3);
        Vector ori(4);
        if (hand=="right")
        {
            Rot(0,0)=-1.0; Rot(0,1)= 0.0; Rot(0,2)= 0.0;
            Rot(1,0)= 0.0; Rot(1,1)= 0.0; Rot(1,2)=-1.0;
            Rot(2,0)= 0.0; Rot(2,1)=-1.0; Rot(2,2)= 0.0;

//            ori[0]=0.0; ori[1]=0.0; ori[2]=-1.0; ori[3]= M_PI/2.0-atan(x[0]/fabs(x[1]));
        }
        else
        {
            Rot(0,0)=-1.0; Rot(0,1)= 0.0; Rot(0,2)= 0.0;
            Rot(1,0)= 0.0; Rot(1,1)= 0.0; Rot(1,2)= -1.0;
            Rot(2,0)= 0.0; Rot(2,1)= -1.0; Rot(2,2)= 0.0;
//            ori[0]=0.0; ori[1]=0.0; ori[2]= 1.0; ori[3]= M_PI/2.0-atan(x[0]/fabs(x[1]));
        }

//        Matrix Rot2 = axis2dcm(ori);

//        return dcm2axis(Rot2.submatrix(0,2,0,2)*Rot);
        return dcm2axis(Rot);

    }

    /***************************************************/
    Vector computeHandOrientationPush(const string &hand)
{
        Matrix Rot(3,3);
        Vector ori(4);
        if (hand=="left")
        {
            Rot(0,0)=-1.0; Rot(0,1)= 0.0; Rot(0,2)= 0.0;
            Rot(1,0)= 0.0; Rot(1,1)= 0.0; Rot(1,2)=-1.0;
            Rot(2,0)= 0.0; Rot(2,1)=-1.0; Rot(2,2)= 0.0;

//            ori[0]=0.0; ori[1]=0.0; ori[2]=-1.0; ori[3]= M_PI/2.0-atan(x[0]/fabs(x[1]));
        }
        else if (hand == "right")
        {
            Rot(0,0)=-1.0; Rot(0,1)= 0.0; Rot(0,2)= 0.0;
            Rot(1,0)= 0.0; Rot(1,1)= 0.0; Rot(1,2)= -1.0;
            Rot(2,0)= 0.0; Rot(2,1)= -1.0; Rot(2,2)= 0.0;
//            ori[0]=0.0; ori[1]=0.0; ori[2]= 1.0; ori[3]= M_PI/2.0-atan(x[0]/fabs(x[1]));
        }
        else
        {
            Rot(0,0)= 0.0; Rot(0,1)= 0.0; Rot(0,2)=  1.0;
            Rot(1,0)=-1.0; Rot(1,1)= 0.0; Rot(1,2)=  0.0;
            Rot(2,0)= 0.0; Rot(2,1)= -1.0; Rot(2,2)= 0.0;
        }
//        Matrix Rot2 = axis2dcm(ori);

//        return dcm2axis(Rot2.submatrix(0,2,0,2)*Rot);
        return dcm2axis(Rot);

    }

    /***************************************************/
    void pointTargetWithHand(const Vector &x, const Vector &o, const string &hand)
    {

	if (hand=="right")
            drvArmR.view(iarm);
        else
            drvArmL.view(iarm);

        Vector dof(10,1.0),dummy;
        iarm->setDOF(dof,dummy);
    
        Vector approach=x;
       // approach[1]+=0.1; // 10 cm
        iarm->goToPoseSync(approach,o);
        iarm->waitMotionDone();
    }

    /***************************************************/
    void look_down()
    {
        Vector ang(3,0.0);
        ang[1]=-40.0;   // elevation [deg]
        igaze->lookAtAbsAngles(ang);
        igaze->waitMotionDone();
    }

    /***************************************************/
    void approachTargetWithHand(const Vector &x, const Vector &o, const string &hand)
    {

	if (hand=="left" || hand=="front")
            drvArmR.view(iarm);
        else
            drvArmL.view(iarm);
        Vector dof(10,1.0),dummy;
        iarm->setDOF(dof,dummy);
    
        Vector approach=x;
	if (hand =="right")approach[1]-=0.1;
	else if (hand =="left")approach[1]+=0.1;
	else approach[0]+=0.1;
        
        iarm->goToPoseSync(approach,o);
        iarm->waitMotionDone();
    }

    /***************************************************/
    void roll(const Vector &bin, const Vector &o,
              const string &hand)
    {
        iarm->setTrajTime(5.0);

        Vector target=bin;
        yDebug()<<"push target pos "<<target.toString().c_str();
        iarm->goToPoseSync(target,o);
        iarm->waitMotionDone();
    }

    /***************************************************/
    void home(const string &hand)
    {
        Vector home_x(3);
        home_x[0]=-0.2;
        home_x[2]=0.08;

        // select the correct interface
        if (hand=="right")
        {
            drvArmR.view(iarm);
            home_x[1]=0.3;
        }
        else
        {
            drvArmL.view(iarm);
            home_x[1]=-0.3;
        }

        igaze->lookAtAbsAngles(Vector(3,0.0));
        iarm->goToPositionSync(home_x);

        iarm->waitMotionDone(0.1,4.0);
        igaze->waitMotionDone();
        igaze->setTrackingMode(false);
    }


public:
    /***************************************************/

    void moveFingers(const string &hand,
                     const VectorOf<int> &joints,
                     const double fingers_closure)
    {
        // select the correct interface
        IControlLimits2   *ilim;
        IPositionControl2 *ipos;
        IControlMode2     *imod;
        if (hand=="right")
        {
            drvHandR.view(ilim);
            drvHandR.view(ipos);
        }
        else
        {
            drvHandL.view(ilim);
            drvHandL.view(ipos);
        }

        // enforce [0,1] interval
        double fingers_closure_sat=std::min(1.0,std::max(0.0,fingers_closure));

        // move each finger first:
        // if min_j and max_j are the minimum and maximum bounds of joint j,
        // then we should move to min_j+fingers_closure_sat*(max_j-min_j)
       // FILL IN THE CODE
        for (size_t i=0; i<joints.size(); i++)
        {
            int j=joints[i];
            double min_j,max_j,target;
            ilim->getLimits(j,&min_j,&max_j);
            target=min_j+fingers_closure_sat*(max_j-min_j);
            ipos->positionMove(j,target);

        }

        // wait until all fingers have attained their set-points
        bool done=false;
        double t0=Time::now();
        while (!done&&(Time::now()-t0<10.0))
        {
//            yInfo()<<"Waiting...";
            Time::delay(0.1);   // release the quantum to avoid starving resources
            ipos->checkMotionDone(&done);
        }

        if (done)
            yInfo()<<"Movement completed";
        else
            yWarning()<<"Timeout expired";
    
    }


/***************************************************/
    bool point_it(const Vector &x, const double fingers_closure)
    {
        Vector xNew; string hand;


        // we select the hand accordingly
        hand=(x[1]>0.0?"right":"left");
        yInfo()<<"selected hand = \""<<hand<<'\"';



        xNew = computePosHandPoint(x);
        yDebug()<<"point position xNew "<<xNew.toString(3,3).c_str();

        Vector o=computeHandOrientationPoint(xNew,hand);
        yInfo()<<"computed orientation = ("<<o.toString(3,3)<<")";

        // we set up here the lists of joints we need to actuate
        VectorOf<int> fingers;
        for (int i=13; i<16; i++)
            fingers.push_back(i);

        // let's put the hand in the pre-grasp configuration
        moveFingers(hand,fingers,0.0);
        yInfo()<<"prepared hand";


        moveFingers(hand,fingers,fingers_closure);
        yInfo()<<"closed fingers";


        pointTargetWithHand(xNew,o,hand);
        yInfo()<<"approached object";

//        moveFingers(hand,fingers,0.0);
//        yInfo()<<"released";

        home(hand);
        yInfo()<<"gone home";
        return true;

    }



/***************************************************/
    bool push_it(const Vector &x, const Vector &bin)
    {
        string hand;
        Vector newBin = bin;
        newBin[2] = tableHigh+0.05;
        // we select the hand accordingly
        if (bin[0]<-0.40)
        {
            hand = "front";
            newBin[1] = x[1];
//            newBin[2]=x[2];
        }
        else if (bin[1]>=0)
        {
            hand = "right";
            newBin[0] = x[0];
//            newBin[2]=x[2];
        }
        else
        {
            hand = "left";
            newBin[0] = x[0];
//            newBin[2]=x[2];
        }
        yInfo()<<"selected hand = \""<<hand<<'\"';

        Vector o=computeHandOrientationPush(hand);

        approachTargetWithHand(x,o,hand);
        yInfo()<<"approached";


        roll(newBin,o,hand);
        yInfo()<<"roll!";
        //TODO: make the roll return


        home(hand);
        yInfo()<<"gone home";
        return true;
    }
	



/***************************************************/
    bool openCartesian(const string &robot, const string &arm)
    {
        PolyDriver &drvArm=(arm=="right_arm"?drvArmR:drvArmL);

        Property optArm;
        optArm.put("device","cartesiancontrollerclient");
        optArm.put("remote","/"+robot+"/cartesianController/"+arm);
        optArm.put("local","/cartesian_client/"+arm);


        // let's give the controller some time to warm up
        bool ok=false;
        double t0=Time::now();
        while (Time::now()-t0<10.0)
        {
            // this might fail if controller
            // is not connected to solver yet
            if (drvArm.open(optArm))
            {
                ok=true;
                break;
            }

            Time::delay(1.0);
        }

        if (!ok)
        {
            yError()<<"Unable to open the Cartesian Controller for "<<arm;
            return false;
        }
        return true;
    }

public:

/****************************************************************/
  bool configure(ResourceFinder &rf)
    {
        string robot=rf.check("robot",Value("icubSim")).asString();

        if (!openCartesian(robot,"right_arm"))
            return false;

        if (!openCartesian(robot,"left_arm"))
        {
            drvArmR.close();
            return false;
        }

        target.resize(3,0.0);
        binLoc.resize(3,0.0);
        tableHigh = -0.05;

        Property optGaze;
        optGaze.put("device","gazecontrollerclient");
        optGaze.put("remote","/iKinGazeCtrl");
        optGaze.put("local","/gaze_client");
       
        if (!drvGaze.open(optGaze))
        {
            yError()<<"Unable to open the Gaze Controller";
            drvArmR.close();
            drvArmL.close();
            return false;
        }

        Property optHandL;
        optHandL.put("device","remote_controlboard");
        optHandL.put("remote","/"+robot+"/left_arm");
        optHandL.put("local","/position/left_arm");


        if (!drvHandL.open(optHandL))
        {
            yError()<<"Unable to open the HANDL Controller";
            drvArmR.close();
            drvArmL.close();
            drvGaze.close();
            return false;
        }

        Property optHandR;
        optHandR.put("device","remote_controlboard");
        optHandR.put("remote","/"+robot+"/right_arm");
        optHandR.put("local","/position/right_arm");

        if (!drvHandR.open(optHandR))
        {
            yError()<<"Unable to open the HandR Controller";
            drvArmR.close();
            drvArmL.close();
            drvHandL.close();
            drvGaze.close();
            return false;
        }

        // save startup contexts
        drvArmR.view(iarm);
        iarm->storeContext(&startup_ctxt_arm_right);

        drvArmL.view(iarm);
        iarm->storeContext(&startup_ctxt_arm_left);

        drvGaze.view(igaze);
        igaze->storeContext(&startup_ctxt_gaze);

        rpcPort.open("/Kinematic/rpc:i");
        attach(rpcPort);
        return true;
    }

    /***************************************************/
    bool interruptModule()
    {
        return true;
    }

    /***************************************************/
    bool close()
    {
        drvArmR.view(iarm);
        iarm->restoreContext(startup_ctxt_arm_right);

        drvArmL.view(iarm);
        iarm->restoreContext(startup_ctxt_arm_left);

        igaze->restoreContext(startup_ctxt_gaze);

        drvArmR.close();
        drvArmL.close();
        drvGaze.close();
        drvHandR.close();
        drvHandL.close();
        rpcPort.close();
        return true;
    }

    /***************************************************/
    bool respond(const Bottle &command, Bottle &reply)
    {
        string cmd=command.get(0).asString();
        if (cmd=="help")
        {
            reply.addVocab(Vocab::encode("many"));
            reply.addString("Available commands:");
            reply.addString("- look_down");
            reply.addString("- point_it");
            reply.addString("- quit");
        }
        else if (cmd=="look_down")
        {
            look_down();
            // we assume the robot is not moving now
            reply.addString("ack");
            reply.addString("Yep! I'm looking down now!");
        }
        else if (cmd=="point_it")
        {
            // the "closure" accounts for how much we should
            // close the fingers around the object:
            // if closure == 0.0, the finger joints have to reach their minimum
            // if closure == 1.0, the finger joints have to reach their maximum
            double fingers_closure=0.5; // default value

            // we can pass a new value via rpc
            if (command.size()>1)
            {
                yInfo()<<"command size: "<<command.size();
                if (command.size() ==5)
                {
                    yDebug()<<"check";

                    for (int i=1;i<=3; i++)
                        target[i-1] = command.get(i).asDouble();
                    fingers_closure=command.get(4).asDouble();
                }
                else if (command.size() ==4)
                {
                    yDebug()<<"check";

                    for (int i=1;i<=3; i++)
                        target[i-1] = command.get(i).asDouble();
                }
            }

            bool ok=point_it(target,fingers_closure);
            // we assume the robot is not moving now
            if (ok)
            {
                reply.addString("ack");
                reply.addString("Yeah! I did it! Maybe...");
            }
            else
            {
                reply.addString("nack");
                reply.addString("I don't see any object!");
            }
        }

        else if (cmd=="push_it")
        {
            // the "closure" accounts for how much we should
            // close the fingers around the object:
            // if closure == 0.0, the finger joints have to reach their minimum
            // if closure == 1.0, the finger joints have to reach their maximum


            // we can pass a new value via rpc
            if (command.size()>1)
            {
                yInfo()<<"command size: "<<command.size();
                if (command.size() ==7)
                {
                    yDebug()<<"check push_it";

                    for (int i=1;i<=3; i++)
                        target[i-1] = command.get(i).asDouble();
                    for (int i=4;i<=6; i++)
                        binLoc[i-4] = command.get(i).asDouble();
                }
            }

            bool ok=push_it(target,binLoc);
            // we assume the robot is not moving now
            if (ok)
            {
                reply.addString("ack");
                reply.addString("Yeah! I did it! Maybe...");
            }
            else
            {
                reply.addString("nack");
                reply.addString("I don't see any object!");
            }
        }

        else if (cmd=="set_table_high")
        {
            if (command.size()>1)
            {
                tableHigh = command.get(1).asDouble();
                yInfo()<<"Table high "<<tableHigh;
                reply.addString("ack");
                reply.addString("table high ");
                reply.addDouble(tableHigh);
            }
            else
                reply.addString("nack");
        }

        else
            // the father class already handles the "quit" command
            return RFModule::respond(command,reply);
        return true;
    }

    /***************************************************/
    double getPeriod()
    {
        return 1.0;
    }

    /***************************************************/
    bool updateModule()
    {
        return true;
    }
};


/***************************************************/
int main(int argc, char *argv[])
{
    Network yarp;
    if (!yarp.checkNetwork())
    {
        yError()<<"YARP doesn't seem to be available";
        return 1;
    }

    CtrlModule mod;
    ResourceFinder rf;
    rf.configure(argc,argv);
    return mod.runModule(rf);
}
    

