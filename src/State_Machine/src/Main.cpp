#include <yarp/os/Network.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/ResourceFinder.h>

#include <SMModule.h>

using namespace yarp::os;


int main(int argc, char * argv[])
{
    Network yarp;

    SMModule module;
    ResourceFinder rf;
    rf.configure(argc, argv);
    rf.setDefaultContext("StateMachineModule");
    //rf.setDefaultConfigFile("StateMachineModule.ini");
    // rf.setVerbose(true);

    // parameters for speech-dev
    rf.setDefault("name","iSpeak");
    rf.setDefault("package","speech-dev");

    module.runModule(rf);

    yInfo()<<"Main returning...";
    return 0;
}
