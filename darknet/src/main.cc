// Begin Main.cpp

#include <ros/ros.h>
#include <std_srvs/Trigger.h>//追加トゥアン

#include <dynamic_reconfigure/server.h>
#include <vision_module/darknetConfig.h>
#include <config.h>
#include <vision_module/VisionOrder.h>
#include <vision_module/ImageInfo.h>
#include <vision_module/ObjectInfo.h>
#include <vision_module/common/VisionProcess.h>
#include <vision_module/common/ROSImageConverter.h>
#include <darknet/detector_class.h>
#include <vector>
#include <string>
#include <algorithm>

#define CV_OBJECT_WINDOW "ObjectImage"
#define NODE_NAME "darknet"
#define OBJECT_DETECTION vision_module::VisionOrder::OBJECT_DETECTION
#define SUB_TOPIC vision_module::VisionOrder::IMAGE_CAPTURE_INFO.c_str()
#define PUB_TOPIC vision_module::VisionOrder::OBJECT_DETECTION_INFO.c_str()

bool closer(const vision_module::ObjectData &left,
            const vision_module::ObjectData &right){

    return left.camera.z < right.camera.z;
}

template <class SUB, class PUB>
class CProcess : public CVisionProcess<SUB, PUB> {
public:
    CProcess(const char *nodeName,
            const char *subTopic, const char *pubTopic);
    ~CProcess();

protected:
    CDetector net;
    float m_maxWidth;
    float m_maxHeight;
    float m_minHeight;
    float m_maxDepth;
    int m_numPoint;
    float m_maxDist;

    bool m_display;
    bool m_server_command; //追加トゥアン

    dynamic_reconfigure::Server<
        vision_module::darknetConfig> m_server;
    dynamic_reconfigure::Server<
        vision_module::darknetConfig>::CallbackType m_f;

    float dotProduct(vision_module::Vector vec1, vision_module::Vector vec2);

public:
    friend vision_module::Vector operator-(const vision_module::Vector &vec1,
                                         const vision_module::Vector &vec2);
    //追加トゥアン
    bool start_call(std_srvs::Trigger::Request &req,
        std_srvs::Trigger::Response &res);
    bool stop_call(std_srvs::Trigger::Request &req,
        std_srvs::Trigger::Response &res);
    //追加トゥアン
    void callback(const SUB &msg);
    void dcfg(vision_module::darknetConfig &config,
            uint32_t level);
};

int main(int argc, char **argv){
    ros::init(argc, argv, NODE_NAME);
    CProcess<vision_module::ImageInfoConstPtr,
             vision_module::ObjectInfo> proc(NODE_NAME, SUB_TOPIC, PUB_TOPIC);
     //追加トゥアン
    ros::NodeHandle n;
    ros::ServiceServer service_start = n.advertiseService("darknet/start", &CProcess<vision_module::ImageInfoConstPtr,
             vision_module::ObjectInfo>::start_call, &proc);
    ros::ServiceServer service_stop = n.advertiseService("darknet/stop", &CProcess<vision_module::ImageInfoConstPtr,
             vision_module::ObjectInfo>::stop_call, &proc);
    //追加トゥアン
    ros::spin();
    return 0;
}

template <class SUB, class PUB>
CProcess<SUB, PUB>::CProcess(const char *nodeName,
        const char *subTopic, const char *pubTopic)
                    : CVisionProcess<SUB, PUB>(nodeName,
                            subTopic, pubTopic),
                    net(COCO_DATA, YOLO_CFG, WEIGHT, THRESH) {
    m_server_command = true;//追加トゥアン

    m_f = boost::bind(&CProcess::dcfg, this, _1, _2);
    m_server.setCallback(m_f);
}

template <class SUB, class PUB>
CProcess<SUB, PUB>::~CProcess(){
}
//追加トゥアン
template <class SUB, class PUB>
bool CProcess<SUB, PUB>::start_call(std_srvs::Trigger::Request &req,
        std_srvs::Trigger::Response &res)
{
    m_server_command = true;
    return true;
}
template <class SUB, class PUB>
bool CProcess<SUB, PUB>::stop_call(std_srvs::Trigger::Request &req,
        std_srvs::Trigger::Response &res)
{
    m_server_command = false;
    return true;
}
//追加トゥアン
template <class SUB, class PUB>
float CProcess<SUB, PUB>::dotProduct(vision_module::Vector vec1,
                            vision_module::Vector vec2){

    return (vec1.x * vec2.x) + (vec1.y * vec2.y) + (vec1.z * vec2.z);
}

vision_module::Vector operator-(const vision_module::Vector &vec1,
                                const vision_module::Vector &vec2){

    vision_module::Vector result;
    result.x = vec1.x - vec2.x;
    result.y = vec1.y - vec2.y;
    result.z = vec1.z - vec2.z;

    return result;
}

template <class SUB, class PUB>
void CProcess<SUB, PUB>::callback(const SUB &msg){
    if (m_server_command){//追加トゥアン

    CvPoint3D32f points[RGBD_IMAGE_SIZE];
    vision_module::ObjectInfo res;
    cv::Mat colorMat = cv::Mat::zeros(msg->height, msg->width, CV_8UC3);
    cv::Mat objectMat = cv::Mat::zeros(msg->height, msg->width, CV_8UC3);

    int objCount = 0;

    try{

    //メッセージをOpenCV形式に変換
    if(convertToCvPC(msg, points, RGBD_IMAGE_SIZE));
    else throw "convertTo";
    if(convertToMat(msg, colorMat));
    else throw "convertToMat";
    objectMat = colorMat.clone();

    //タイムスタンプを受け継ぐ
    res.header = msg->header;
    //darknetによる物体検出
    net.detect(colorMat);
    //処理結果を取得
    std::vector<cv::Mat> rois = net.getRois();
    std::vector<float> probs = net.getProbs();
    std::vector<std::string> names = net.getNames();
    std::vector<struct bbox> bboxes = net.getBBoxes();
    objCount = rois.size();

    res.objects.clear();
    //検出された物体ごとに処理
    for(int i = 0; i < objCount; i++){
        vision_module::ObjectData data;

        data.szwht.x = 0.0;
        data.szwht.y = 0.0;
        data.szwht.z = 0.0;

        data.normal.x = 0.0;
        data.normal.y = 0.0;
        data.normal.z = 0.0;
        data.planeNum = -1;

        data.width = rois[i].cols;
        data.height = rois[i].rows;
        setToVector(rois[i], data.bgr);

        struct bbox bbox = bboxes[i];
        int centerX = (bbox.left + bbox.right) / 2;
        int centerY = (bbox.top + bbox.bottom) / 2;
        CvPoint3D32f p = points[(centerY * msg->width) + centerX];
        data.camera.x = p.x;
        data.camera.y = p.y;
        data.camera.z = p.z;

        char t[128];
        sprintf(t, "%s: %.3f", names[i].c_str(), probs[i]);
        int top = bbox.top;
        if(top + 20 < msg->height)top += 20;
        else top = msg->height - 1;

          vision_module::NBest generic;
          generic.id = -1;
          generic.name = names[i];
          generic.score = probs[i];
          data.generic.push_back(generic);

          // point cloudが取得できなかった物体を白で囲む publishはしない
          if(data.camera.x == 0 && data.camera.y == 0 && data.camera.x==0){
              cv::rectangle(objectMat,
                      cv::Point(bbox.left, bbox.top),
                      cv::Point(bbox.right, bbox.bottom),
                      cv::Scalar(200, 200, 200), 1, 4);
              cv::putText(objectMat, t,
                      cv::Point(bbox.left, top),
                      cv::FONT_HERSHEY_SIMPLEX, 0.6,
                      cv::Scalar(200, 200, 200), 1, CV_AA);
          // personのみ緑枠
          }else if(strcmp(names[i].c_str(), "person") == 0){
              cv::rectangle(objectMat,
                      cv::Point(bbox.left, bbox.top),
                      cv::Point(bbox.right, bbox.bottom),
                      cv::Scalar(0, 200, 0), 1, 4);
              cv::putText(objectMat, t,
                      cv::Point(bbox.left, top),
                      cv::FONT_HERSHEY_SIMPLEX, 0.6,
                      cv::Scalar(0, 200, 0), 1, CV_AA);
              res.objects.push_back(data);
          }else{
              cv::rectangle(objectMat,
                      cv::Point(bbox.left, bbox.top),
                      cv::Point(bbox.right, bbox.bottom),
                      cv::Scalar(0, 0, 200), 1, 4);
              cv::putText(objectMat, t,
                      cv::Point(bbox.left, top),
                      cv::FONT_HERSHEY_SIMPLEX, 0.6,
                      cv::Scalar(0, 0, 200), 1, CV_AA);
              res.objects.push_back(data);
          }

          // res.objects.push_back(data);
    }

    //近い順にソート
    std::sort(res.objects.begin(), res.objects.end(), closer);
    //publish
    this->_pub.publish(res);

    //処理結果をプリント
//     printf("-------%s:  NUM OF OBJECTS: %d-------\n",
//                 this->_nodeName, objCount);
//
//     for(int i = 0; i < objCount; i++){
//         vision_module::ObjectData *data = &res.objects[i];
//         ROS_INFO("%s : NUM_OBJ %d:\n\
// POS %.2f, %.2f, %.2f: NORMAL %.2f, %.2f, %.2f: PlaneNum %2d\n\
// SIZE %.2f, %.2f, %.2f",
//                     this->_nodeName, i,
//                     data->camera.x, data->camera.y, data->camera.z,
//                     data->normal.x, data->normal.y, data->normal.z,
//                     data->planeNum,
//                     data->szwht.x, data->szwht.y, data->szwht.z);
//     }
//     printf("\n");

    //処理結果を保存
    // cv::imwrite(OBJECT_IMAGE, objectMat);
    //処理結果を表示
    if(m_display){
        cv::imshow(CV_OBJECT_WINDOW, objectMat);
        cv::waitKey(10);
    }
    }catch(char *e){
    ROS_ERROR("ERROR: %s : %s", this->_nodeName, e);
    }
    }//追加トゥアン
}

template <class SUB, class PUB>
void CProcess<SUB, PUB>::dcfg(vision_module::darknetConfig &config,
            uint32_t level){
    ROS_INFO("\
Reconfigure Request: %s\n\
MAX_WIDTH : %f\n\
MAX_HEIGHT: %f\n\
MIN_HEIGHT: %f\n\
MAX_DEPTH : %f\n\
MIN_POINT : %d\n\
MAX_DIST  : %f\n\
DISPLAY : %d",
        NODE_NAME,
        config.MAX_WIDTH,
        config.MAX_HEIGHT,
        config.MIN_HEIGHT,
        config.MAX_DEPTH,
        config.MIN_POINT,
        config.MAX_DIST,
        config.DISPLAY
    );
    m_maxWidth =  config.MAX_WIDTH;
    m_maxHeight = config.MAX_HEIGHT;
    m_minHeight = config.MIN_HEIGHT;
    m_maxDepth =  config.MAX_DEPTH;
    m_numPoint =  config.MIN_POINT;
    m_maxDist =   config.MAX_DIST;
    m_display =   config.DISPLAY;
    if(m_display);
    else cv::destroyAllWindows();
}
