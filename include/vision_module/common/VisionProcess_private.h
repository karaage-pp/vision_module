
#ifndef _VISION_PROCESS_PRIVATE
#define _VISION_PROCESS_PRIVATE

template <class SUB, class PUB>
CVisionProcess<SUB, PUB>::CVisionProcess(const char *nodeName, 
        const char *subTopic, const char *pubTopic){

    sprintf(_nodeName, "%s", nodeName);
    sprintf(_subTopic, "%s", subTopic);
    sprintf(_pubTopic, "%s", pubTopic);
    printf("%s, %s, %s", _nodeName, _subTopic, _pubTopic);
    _pub = _nh.advertise<PUB>(_pubTopic, 1, 
            boost::bind(&CVisionProcess::connectCB, this, _1), 
            boost::bind(&CVisionProcess::disconnectCB, this, _1));
    if(_pub)ROS_INFO("%s: SETUPED", _nodeName);
}

template <class SUB, class PUB>
CVisionProcess<SUB, PUB>::~CVisionProcess(){
}

template <class SUB, class PUB>
void CVisionProcess<SUB, PUB>::connectCB(
        const ros::SingleSubscriberPublisher &pub){

    subscribe();
}

template <class SUB, class PUB>
void CVisionProcess<SUB, PUB>::disconnectCB(
        const ros::SingleSubscriberPublisher &pub){

    if(_pub.getNumSubscribers()){
    }else{
        shutdown();
    }
}

template <class SUB, class PUB>
int CVisionProcess<SUB, PUB>::getNumSubscribers(){
    return _pub.getNumSubscribers();
}

template <class SUB, class PUB>
void CVisionProcess<SUB, PUB>::subscribe(){
    if(!_sub){
        _sub = _nh.subscribe(_subTopic, 1, &CVisionProcess::callback, this);
        ROS_INFO("%s: subscribe %s", _nodeName, _subTopic);
    }
}

template <class SUB, class PUB>
void CVisionProcess<SUB, PUB>::shutdown(){
    if(_sub){
        _sub.shutdown();
        ROS_INFO("%s: unsubscribe %s", _nodeName, _subTopic);
    }
}

template <class SUB, class PUB>
void CVisionProcess<SUB, PUB>::enableAutomode(bool condition){
    if(condition){
        if(_pub.getNumSubscribers()){
            subscribe();
        }else{
            shutdown();
        }
    }else{
        shutdown();
    }
}

template <class SUB, class PUB>
void CVisionProcess<SUB, PUB>::callback(const SUB &msg){
    ROS_INFO("%s: ERROR: callback is unset", _nodeName);
}

#endif

