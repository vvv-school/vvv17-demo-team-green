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

class Processing : public BufferedPort<ImageOf<PixelRgb>>
{
protected:
  bool init_;
  std::string moduleName;
  BufferedPort<Bottle> predictionsOut;
  Classifier classifier_;

public:

  Processing(const std::string &moduleName, std::string model_path, std::string weights_path, std::string mean_file, std::string label_file)
  {
    this->moduleName = moduleName;
    classifier_ = new Classifier(model_path, weights_path, mean_file, label_file);  
  }

  ~Processing()
  {
    delete classifier_;
  }

  bool open()
  {
    this->useCallback();

    BufferedPort<ImageOf<PixelRgb>>::open("/" + moduleName + "/image:i");
    predictionsOut.open("/" + moduleName + "/predictions:o");

    return true;
  }

  void close()
  {
    imgPortIn.close();
    imgPortOut.close();
    BufferedPort<ImageOf<PixelRgb>>::close();
  }

  void interrupt()
  {
    BufferedPort<ImageOf<PixelRgb>>::interrupt();
  }

  void onRead(ImageOf<PixelRgb> &img) {
    cv::Mat in_cv = cv::arrToMat( (IplImage *)img.getIpalImage() );
    std::vector<Prediction> predictions = classifier_->Classify(img);

    Bottle &outPreds = predictionsOut.prepare();

    if (predictions.size() > 0)
    {
      // TODO What kind of output ? just one predictions or multiple, with or without confidence ?
      outPreds.addString(predictions[0].first);
      out.Preds.write();
    }

  }
}


class RecoModule: public RFModule
{

protected:
  ResourceFinder *rf;
  RpcServer rpcPort;

  Processing *processing;
  friend class processing;

  bool attach(RpcServer &source)
  {
    return this->yarp().attachAsServer(source);
  } 

public:
  bool configure(ResourceFinder &rf)
  {
    this->rf = &rf;
    std::string moduleName = rf.check("name", Value("recognition_service")).asString();
    std::string model_path = rf.check("model_path", "data/deploy.prototxt").asString();
    std::string weights_path = rf.check("weights_path", "data/bvlc_reference_caffenet.caffemodel").asString();
    std::string mean_file = rf.check("mean_file", "data/imagenet_mean.binaryproto").asString();
    std::string label_file = rf.check("label_file", "data/synset_words.txt").asString();

    rpcPort.open(("/" + moduleName + "/rpc").c_str());
    processing = new Processing(moduleName, model_path, weights_path, mean_file, label_file);
    processing->open();

    attach(rpcPort);

    return true;
  }

  bool interruptModule()
  {
    return true;
  }

  bool close()
  {
    processing->interrupt();
    processing->close();
    delete processing;
    return true;
  }

  bool respond(const Bottle &command, Bottle &reply)
  {
    string cmd = command.get(0).asString();

    if (cmd == "help")
    {
      reply.addVocab(Vocab::encode("many"));
      reply.addString("load");
      reply.addString("recognize");
      reply.addString("quit");
    }
    else if (cmd == "recognize")
    {
      
    }
    else if (cmd == "load")
    {
      std::string img_path = command.get(1).asString();
      cv::Mat img = cv::imread(img_path, CV_LOAD_IMAGE_COLOR);
      std::vector<Prediction> predictions = classifier_->Classify(img);
    }
    else
    {
      return RFModule::respond(command, reply);
    }

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
