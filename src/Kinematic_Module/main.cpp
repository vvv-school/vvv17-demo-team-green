#include <string>

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>

#include "Kinematic_Module_IDL.h"

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
class CtrlModule: public RFModule, public Kinematic_Module_IDL
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
    double trajTime;


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
    bool pointTargetWithHand(const Vector &x, const Vector &o, const string &hand)
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
        return iarm->waitMotionDone();

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
        iarm->waitMotionDone(0.1,4.0);
    }

    /***************************************************/
    bool move_object(const Vector &bin, const Vector &o,
              const string &hand)
    {
        iarm->setTrajTime(trajTime);

        Vector target=bin;
        yDebug()<<"push target pos "<<target.toString().c_str();
        iarm->goToPoseSync(target,o,trajTime);
        return iarm->waitMotionDone();

     }

    /***************************************************/
    void home(const string &hand)
    {
        Vector home_x(3);
        home_x[0]=-0.2;
        home_x[2]=0.08;

        // select the correct interface
        if (hand=="left"&&hand=="front")
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


    // Thrift methods
    double get_table_high()
    {
        return tableHigh;
    }

    bool set_table_high(const double _high)
    {
        tableHigh = _high;
    }

    double get_traj_time()
    {
        return trajTime;
    }

    bool set_traj_time(const double _trajTime)
    {
        trajTime = _trajTime;
    }

    bool point(const Vector &_targetPos)
    {
        target = _targetPos;
        double fingers_closure = 0.5;
        return point_it(target,fingers_closure);
    }

    bool push(const Vector &_targetPos, const Vector &_binLoc)
    {
        target = _targetPos;
        binLoc = _binLoc;
        return push_it(target,binLoc);
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
        bool flag = false;

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


        flag=pointTargetWithHand(xNew,o,hand);
        yInfo()<<"point object";

//        moveFingers(hand,fingers,0.0);
//        yInfo()<<"released";




        home(hand);
        yInfo()<<"gone home";
        return flag;

    }



/***************************************************/
    bool push_it(const Vector &x, const Vector &bin)
    {
        string hand;
	bool flag = false;
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


        flag=move_object(newBin,o,hand);
        yInfo()<<"move object!";
        //TODO: make the move_object return
		
        home(hand);
        yInfo()<<"gone home";
        return flag;
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
        string moduleName = rf.check("name",Value("GC_kinematics")).asString();

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
        trajTime = 5.0;

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

        rpcPort.open("/" +moduleName+"/rpc:i");
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

    /***************************************************/

    bool attach(RpcServer &source)
    {
        return this->yarp().attachAsServer(source);
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
    

