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

class Processing : public BufferedPort<ImageOf<PixelRgb> >
{
protected:
  bool init_;
  std::string moduleName;
  BufferedPort<Bottle> predictionsOut;
  Classifier *classifier_;

public:

  Processing(const std::string &moduleName,const  std::string &model_path, const std::string &weights_path, const std::string &mean_file, const std::string &label_file)
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

    BufferedPort<ImageOf<PixelRgb> >::open("/" + moduleName + "/img_stream");
    predictionsOut.open("/" + moduleName + "/output_class");

    return true;
  }

  void close()
  {
    predictionsOut.close();
    BufferedPort<ImageOf<PixelRgb> >::close();
  }

  void interrupt()
  {
    BufferedPort<ImageOf<PixelRgb> >::interrupt();
  }

  void onRead(ImageOf<PixelRgb> &img) {
    cv::Mat in_cv = cv::cvarrToMat( (IplImage *)img.getIplImage() );
    std::vector<Prediction> predictions = classifier_->Classify(in_cv, 3);

    for (int i=0; i < predictions.size(); i++) {
      std::cout << "Class " << predictions[i].first
                << " predicted with confidence " << predictions[i].first
                << std::endl;
    }

    Bottle &outPreds = predictionsOut.prepare();

    if (predictions.size() > 0)
    {
      // TODO What kind of output ? just one predictions or multiple, with or without confidence ?
      outPreds.addString(predictions[0].first);
    } else
    {
      outPreds.addString("NO_PREDICTION");
    }
    predictionsOut.write();
  }
};


class RecoModule: public RFModule
{

protected:
  ResourceFinder *rf;
  RpcServer rpcPort;
  bool closing;

  Processing *processing;
  friend class processing;

  /*bool attach(RpcServer &source)
  {
    return this->yarp().attachAsServer(source);
  }*/

public:
  bool configure(ResourceFinder &rf)
  {
    closing = false;
    this->rf = &rf;
    std::string moduleName = rf.check("name", Value("Objet_Recognition")).asString();
    std::string model_path = rf.check("model_path", Value("data/deploy.prototxt")).asString();
    std::string weights_path = rf.check("weights_path", Value("data/bvlc_reference_caffenet.caffemodel")).asString();
    std::string mean_file = rf.check("mean_file", Value("data/imagenet_mean.binaryproto")).asString();
    std::string label_file = rf.check("label_file", Value("data/synset_words.txt")).asString();

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

  bool updateModule()
  {
    return !closing;
  }

  bool quit()
  {
    closing = true;
    return true;
  }

  bool respond(const Bottle &command, Bottle &reply)
  {
    string cmd = command.get(0).asString();

    if (cmd == "help")
    {
      reply.addVocab(Vocab::encode("many"));
      reply.addString("recognize_file");
      reply.addString("quit");
    }
    else if (cmd == "recognize_file")
    {
      string filename = command.get(1).asString();
      cv::Mat img_cv = cv::imread(filename, CV_LOAD_IMAGE_COLOR);

      yarp::sig::ImageOf<yarp::sig::PixelRgb> &inImage  = processing->prepare();

      IplImage img = img_cv;
      inImage.resize(img.width, img.height);
      cvCopy( &img, (IplImage *) inImage.getIplImage());
      processing->write();
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
