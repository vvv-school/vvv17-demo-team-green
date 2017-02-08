#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include "Classifier.h"
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;

typedef std::pair<string, float> Prediction;


class ImageProcessor : public PortReader
{
protected:
  Classifier *classifier_;

  bool read(ConnectionReader &connection)
  {
    yInfo() << "About to recognize an image";
    ImageOf<PixelRgb> imageIn;
    if (!imageIn.read(connection))
        return false;

    cv::Mat in_cv = cv::cvarrToMat( (IplImage *)imageIn.getIplImage() );
    std::vector<Prediction> predictions = classifier_->Classify(in_cv, 7);

    for (size_t i=0; i < predictions.size(); i++) {
      yInfo() << "Class " << predictions[i].first
              << " predicted with confidence " << predictions[i].second;
    }

    Bottle reply; 
    if (predictions.size() > 0)
    {
      // TODO What kind of output ? just one predictions or multiple, with or without confidence ?
      reply.addString(predictions[0].first);
    } else
    {
      reply.addString("NO_PREDICTION");
    }

    if (ConnectionWriter *writer=connection.getWriter())
      reply.write(*writer);

    return true;
  }

public:
  ImageProcessor(const  std::string &model_path, const std::string &weights_path, const std::string &mean_file, const std::string &label_file)
  {
    classifier_ = new Classifier(model_path, weights_path, mean_file, label_file);
  }

  ~ImageProcessor()
  {
    delete classifier_;
  }
};

class RecoModule: public RFModule
{
protected:
  ResourceFinder *rf;
  RpcServer rpcPort;
  RpcServer testRpcPort;
  ImageProcessor *imgProc;
  bool closing;

public:
  bool configure(ResourceFinder &rf)
  {
    closing = false;
    this->rf = &rf;
    std::string moduleName = rf.check("name", Value("Object_Recognition")).asString();
    std::string model_path = rf.check("model_path", Value("data/deploy.prototxt")).asString();
    std::string weights_path = rf.check("weights_path", Value("data/garbage_fine_tuning_iter_15000.caffemodel")).asString();
    std::string mean_file = rf.check("mean_file", Value("data/mean.binaryproto")).asString();
    std::string label_file = rf.check("label_file", Value("data/labels.txt")).asString();

    imgProc = new ImageProcessor(model_path, weights_path, mean_file, label_file);
    rpcPort.setReader(*imgProc);

    rpcPort.open(("/" + moduleName + "/image").c_str());
    testRpcPort.open(("/" + moduleName + "/rpc").c_str());

    attach(testRpcPort);

    return true;
  }

  bool interruptModule()
  {
    rpcPort.interrupt();
    return true;
  }

  bool close()
  {
    rpcPort.close();
    delete imgProc;
    return true;
  }

  bool updateModule()
  {
    return !closing;
  }

  bool quit()
  {
    closing = true;
    return true;
  }

  double getPeriod()
  {
    return 0.0; //sync upon incoming images
  }
};

int main(int argc, char** argv)
{
  Network yarp;
  if (!yarp.checkNetwork())
  {
    yError() << "yarp network error";
    return -1;
  }
  RecoModule mod;
  ResourceFinder rf;
  rf.setVerbose();
  rf.configure(argc,argv);

  return mod.runModule(rf);
}
