#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;

int main(int argc, char** argv)
{
  Network yarp;
  if (!yarp.checkNetwork())
  {
    yError() << "yarp network error";
    return -1;
  }

  RpcClient port;
  port.open("/client");
  yarp.connect("/client", "/Object_Recognition/image");
  
  std::string filename = "/home/icub/00001990.ppm";
  cv::Mat img_cv = cv::imread(filename, CV_LOAD_IMAGE_COLOR);
  std::cout << filename << " opened. Size: "
            << img_cv.rows << " " << img_cv.cols
            << std::endl;

  IplImage img = img_cv;
  ImageOf<PixelRgb> inImage;
  inImage.resize(img.width, img.height); 
  cvCopy( &img, (IplImage *) inImage.getIplImage());
  
  Bottle reply;
  port.write(inImage, reply);
  yInfo() << "Received: " << reply.toString();
}
