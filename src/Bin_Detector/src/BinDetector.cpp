#include <BinDetector.h>
#include <yarp/os/LogStream.h>

using namespace yarp::os;


// BinDetector::BinDetector() { }

// BinDetector::~BinDetector() { }

#define RED 


bool BinDetector::configure(yarp::os::ResourceFinder &rf) {

    bins.resize(3);
    bins[0].resize(3);
    bins[1].resize(3);
    bins[2].resize(3);

    thresholdH.resize(3);
    thresholdL.resize(3);

    lowBound.push_back(0);
    lowBound.push_back(0);
    lowBound.push_back(0);
    
    highBound.push_back(255);
    highBound.push_back(255);
    highBound.push_back(255);

    thresholdH[RED_BIN] = cv::Scalar(20, 255, 255);
    thresholdH[BLUE_BIN] = cv::Scalar(80, 100, 255);
    thresholdH[GREEN_BIN] = cv::Scalar(40, 255, 255);

    thresholdL[RED_BIN] = cv::Scalar(0, 190, 200);
    thresholdL[BLUE_BIN] = cv::Scalar(40, 0, 50);
    thresholdL[GREEN_BIN] = cv::Scalar(30, 100, 100);

    moduleName = rf.check("name", Value("GC_bindetector")).asString();
    setName(moduleName.c_str());


    // open all ports
    bool ret = commandPort.open("/"+moduleName+"/rpc");
    ret &= inPort.open("/"+moduleName+"/image:i");
    ret &= outPort.open("/"+moduleName+":o");
    ret &= imageOut.open("/"+moduleName+"-image:o");
    ret &= imageGray.open("/"+moduleName+"-gray:o");
    if(!ret) {
        yError()<<"Cannot open some of the ports";
        return false;
    }

    if(!attach(commandPort)) {
        yError()<<"Cannot attach to the commandPort";
        return false;
    }

    // everything is fine
    return true;
}


double BinDetector::getPeriod() {
    return 0.5; // module periodicity (seconds)
}


bool BinDetector::updateModule() {
    yarp::sig::ImageOf<yarp::sig::PixelRgb> &outImage  = imageOut.prepare();
    yarp::sig::ImageOf<yarp::sig::PixelMono> &imgGr  = imageGray.prepare();

    yarp::sig::ImageOf<yarp::sig::PixelRgb> *inImage = inPort.read();
    cv::Mat inColour_cv = cv::cvarrToMat((IplImage *)inImage->getIplImage());

    cv::Mat img = inColour_cv.clone();
    cv::Mat gray;

    cvtColor(img, img, CV_RGB2HSV);

    // mutex.lock();



    // mutex.unlock();


   
    for (int bin_id = 0; bin_id < 3; bin_id++)
    {
        cv::inRange(img, thresholdL[bin_id], thresholdH[bin_id], gray);

        std::vector<std::vector<cv::Point> > contours;

        cv::findContours(gray, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
        

        cv::drawContours(img, contours, 0, cv::Scalar(255,255,0));        

        std::vector<cv::Moments> mu(contours.size() );
        for( int i = 0; i < contours.size(); i++ )
        { mu[i] = moments( contours[i], false ); }


        // //Mass center
        std::vector<cv::Point2f> mc( contours.size() );
        for( int i = 0; i < contours.size(); i++ )
        { 
            if(contours[i].size()>20)
            {
                mc[i] = cv::Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 ); 
                cv::circle(img, mc[i], 3, cv::Scalar(0,255,0), -1, 8, 0);
                bins[bin_id][0] = mc[0].x;
                bins[bin_id][1] = mc[0].y;
                bins[bin_id][2] = 0;                
            }

        }
     }




    


    // // Blue position - alumunium
    // bins[0][0] = 30;
    // bins[0][1] = 200;
    // bins[0][2] = 0;

    // // Green position - plastic
    // bins[1][0] = 160;
    // bins[1][1] = 150;
    // bins[1][2] = 0;

    // // Red position - paper
    // bins[2][0] = 290;
    // bins[2][1] = 200;
    // bins[2][2] = 0;



    cvtColor(img, img, CV_HSV2RGB);

    IplImage out = img;
    outImage.resize(out.width, out.height);
    cvCopy( &out, (IplImage *) outImage.getIplImage());
    imageOut.write();

    IplImage outGray = gray;
    imgGr.resize(outGray.width, outGray.height);
    cvCopy( &outGray, (IplImage *) imgGr.getIplImage());
    imageGray.write();


    Bottle& output = outPort.prepare();
    output.clear();
    getBins(output);
    outPort.write();
    

    return true;
}


bool BinDetector::respond(const Bottle& command, Bottle& reply) {
    if (command.get(0).asString()=="getBins")
    {
        getBins(reply);
        return true;
    }    
    else if (command.get(0).asString()=="setUpperBound")
        setUpperBound(command.get(1).asInt(),command.get(2).asInt(),command.get(3).asInt());
    else if (command.get(0).asString()=="setLowerBound")
        setLowerBound(command.get(1).asInt(),command.get(2).asInt(),command.get(3).asInt());
    else if (command.get(0).asString()=="getLowerBound")
    {
        std::vector<int32_t> bound = getLowerBound();
        reply.clear();
        reply.addInt(bound[0]);
        reply.addInt(bound[1]);
        reply.addInt(bound[2]);
        return true;
    }    
    else if (command.get(0).asString()=="getUpperBound")
        {
        std::vector<int32_t> bound = getUpperBound();
        reply.clear();
        reply.addInt(bound[0]);
        reply.addInt(bound[1]);
        reply.addInt(bound[2]);
        return true;
    }  
    else {
        reply.clear();
    reply.addString("Error");
        return false;
    }
    reply.clear();
    reply.addString("Ok");
    return true;
}


bool BinDetector::interruptModule() {
    return true;
}


bool BinDetector::close() {
    commandPort.close();
    outPort.close();
    imageOut.close();
    inPort.close();
    imageGray.close();
    return true;
}

bool BinDetector::getBins(Bottle& reply)
{
    reply.clear();   
    for (int i = 0; i < 3; i++)
    {
        yarp::os::Bottle &t = reply.addList();
        t.addDouble(bins[i][0]);
        t.addDouble(bins[i][1]);
        t.addDouble(bins[i][2]);

    }


    return true;
}

void BinDetector::binDetection()
{

}

bool BinDetector::setLowerBound(const int32_t r, const int32_t g, const int32_t b)
{
    mutex.lock();
    lowBound.clear();
    lowBound.push_back(r);
    lowBound.push_back(g);
    lowBound.push_back(b);
    mutex.unlock();
    return true;
}
/********************************************************/
bool BinDetector::setUpperBound(const int32_t r, const int32_t g, const int32_t b)
{
    mutex.lock();
    highBound.clear();
    highBound.push_back(r);
    highBound.push_back(g);
    highBound.push_back(b);
    mutex.unlock();
    return true;
}


std::vector<int32_t> BinDetector::getLowerBound()
{
    std::vector<int32_t> v;
    mutex.lock();
    v = lowBound;
    mutex.unlock();
    return v;
}

/********************************************************/
std::vector<int32_t> BinDetector::getUpperBound()
{
    std::vector<int32_t> v;
    mutex.lock();
    v = highBound;
    mutex.unlock();
    return v;
}
