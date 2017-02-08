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

    lowBound.push_back(100);
    lowBound.push_back(100);
    lowBound.push_back(100);
    
    highBound.push_back(160);
    highBound.push_back(255);
    highBound.push_back(255);



    moduleName = rf.check("name", Value("BinDetector")).asString();
    setName(moduleName.c_str());


    // open all ports
    bool ret = commandPort.open("/"+moduleName+"/rpc");
    ret &= inPort.open("/"+moduleName+":i");
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


    cv::inRange(inColour_cv, cv::Scalar(lowBound[0],lowBound[1],lowBound[2]), 
        cv::Scalar(highBound[0],highBound[1],highBound[2]), gray);

   
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
    else {
        return false;
    }
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