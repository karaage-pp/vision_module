
#ifndef _VISION_PROCESS
#define _VISION_PROCESS

#include <ros/ros.h>

template <class SUB, class PUB>
class CVisionProcess{

public:
    CVisionProcess(const char *nodeName, 
            const char *subTopic, const char *pubTopic);
    ~CVisionProcess();

protected:
    ros::NodeHandle _nh;
    ros::Subscriber _sub;
    ros::Publisher _pub;
    char _nodeName[64];
    char _subTopic[128];
    char _pubTopic[128];

    void connectCB(const ros::SingleSubscriberPublisher &pub);
    void disconnectCB(const ros::SingleSubscriberPublisher &pub);

public:
    int getNumSubscribers();
    void subscribe();
    void shutdown();
    void enableAutomode(bool condition);
    virtual void callback(const SUB &msg);
};

#include "VisionProcess_private.h"

#endif 

