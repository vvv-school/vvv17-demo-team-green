#include <BinDetector.h>
#include <yarp/os/LogStream.h>

using namespace yarp::os;


// BinDetector::BinDetector() { }

// BinDetector::~BinDetector() { }


bool BinDetector::configure(yarp::os::ResourceFinder &rf) {

    bins.resize(3);
    bins[0].resize(3);
    bins[1].resize(3);
    bins[2].resize(3);

    lowBound.push_back(0);
    lowBound.push_back(0);
    lowBound.push_back(0);
    
    highBound.push_back(255);
    highBound.push_back(255);
    highBound.push_back(255);



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
    return 0.01; // module periodicity (seconds)
}


bool BinDetector::updateModule() {
    yarp::sig::ImageOf<yarp::sig::PixelRgb> &outImage  = imageOut.prepare();
    yarp::sig::ImageOf<yarp::sig::PixelMono> &imgGr  = imageGray.prepare();

    yarp::sig::ImageOf<yarp::sig::PixelRgb> *inImage = inPort.read();
    cv::Mat inColour_cv = cv::cvarrToMat((IplImage *)inImage->getIplImage());

    cv::Mat img = inColour_cv.clone();
    cv::Mat gray;

    cvtColor(img, img, CV_RGB2HSV);

    mutex.lock();

    cv::inRange(img, cv::Scalar(lowBound[0],lowBound[1],lowBound[2]), 
        cv::Scalar(highBound[0],highBound[1],highBound[2]), gray);

    mutex.unlock();

   
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
    outPort.write();
    

    return true;
}


bool BinDetector::respond(const Bottle& command, Bottle& reply) {
    if (command.get(0).asString()=="getBins")
        getBins(reply);
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
