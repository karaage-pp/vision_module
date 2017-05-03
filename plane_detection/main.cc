// Begin Main.cpp

#include <ros/ros.h>
#include <std_srvs/Trigger.h>//追加トゥアン

#include <dynamic_reconfigure/server.h>
#include <vision_module/plane_detectionConfig.h>
#include <vision_module/plane_detection/config.h>
#include <vision_module/plane_detection/PlaneDetection.h>
#include <vision_module/common/ROSImageConverter.h>
#include <vision_module/common/VisionProcess.h>
#include <vision_module/ImageInfo.h>
#include <vision_module/VisionOrder.h>

#define CV_PLANE_WINDOW "PlaneImage"
#define CV_FILTERED_WINDOW "FilteredImage"
#define NODE_NAME "plane_detection"
#define SUB_TOPIC vision_module::VisionOrder::IMAGE_CAPTURE_INFO.c_str()
#define PUB_TOPIC vision_module::VisionOrder::PLANE_DETECTION_INFO.c_str()

template <class SUB, class PUB>
class CProcess : public CVisionProcess<SUB, PUB> {
public:
    CProcess(const char *nodeName,
            const char *subTopic, const char *pubTopic);
    ~CProcess();

protected:
    float m_maxDist;
    float m_minDist;
    float m_errorRange;
    int m_minSegmentSize;
    int m_searchIteration;
    float m_maxGap;

    bool m_display;
    bool m_saveImage;
    bool m_server_command; //追加トゥアン

    dynamic_reconfigure::Server<
        vision_module::plane_detectionConfig> m_server;
    dynamic_reconfigure::Server<
        vision_module::plane_detectionConfig>::CallbackType m_f;

public:
    //追加トゥアン
    bool start_call(std_srvs::Trigger::Request &req,
        std_srvs::Trigger::Response &res);
    bool stop_call(std_srvs::Trigger::Request &req,
        std_srvs::Trigger::Response &res);
    //追加トゥアン
    void callback(const SUB &msg);
    void dcfg(vision_module::plane_detectionConfig &config,
            uint32_t level);
};

int main(int argc, char **argv){
    ros::init(argc, argv, NODE_NAME);
    CProcess<vision_module::ImageInfoConstPtr,
             vision_module::ObjectInfo> proc(NODE_NAME, SUB_TOPIC, PUB_TOPIC);
     //追加トゥアン
    ros::NodeHandle n;
    ros::ServiceServer service_start = n.advertiseService("plane_detection/start", &CProcess<vision_module::ImageInfoConstPtr,
             vision_module::ObjectInfo>::start_call, &proc);
    ros::ServiceServer service_stop = n.advertiseService("plane_detection/stop", &CProcess<vision_module::ImageInfoConstPtr, 
             vision_module::ObjectInfo>::stop_call, &proc);
    //追加トゥアン
    ros::spin();
    return 0;
}

template <class SUB, class PUB>
CProcess<SUB, PUB>::CProcess(const char *nodeName,
        const char *subTopic, const char *pubTopic)
                    : CVisionProcess<SUB, PUB>(nodeName,
                            subTopic, pubTopic) {
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
void CProcess<SUB, PUB>::callback(const SUB &msg){
    if (m_server_command){//追加トゥアン
    cv::Mat color = cv::Mat::zeros(
                    RGBD_IMAGE_WIDTH, RGBD_IMAGE_HEIGHT, CV_8UC3);
    cv::Mat plane = cv::Mat::zeros(
                    RGBD_IMAGE_WIDTH, RGBD_IMAGE_HEIGHT, CV_8UC3);
    CvPoint3D32f points[RGBD_IMAGE_SIZE];
    CPlaneDetection2 pd;
    std::vector<CPlaneDetection2::planeInfo> pi;

    //平面検出パラメータの初期化
    pd.init(RGBD_IMAGE_WIDTH, RGBD_IMAGE_HEIGHT, m_maxDist);
    pd.paramConfig(m_errorRange, m_minSegmentSize, m_searchIteration,
                    m_maxDist, m_minDist, m_maxGap);

    try{
        //データをOpenCV形式に変更
        if(convertToMat(msg, color));
        else throw "convertToMat";
        if(convertToCvPC(msg, points, RGBD_IMAGE_SIZE));
        else throw "convertToCvPC";

        //平面検出
        pd.detectPlane(points, m_maxDist);

        vision_module::ObjectInfo res;
        //タイムスタンプを受け継ぐ
        res.header = msg->header;
        copyTo(msg, res);
        //検出された平面のラベルを取得
        setLabel(pd.getSegmentIndx(), res, RGBD_IMAGE_SIZE);
        res.planes.clear();
        pi.clear();
        pi = pd.getPlaneInfo();
        //検出された平面毎に処理
        for(std::vector<CPlaneDetection2::planeInfo>::iterator
                it = pi.begin();
                it != pi.end(); it++){

            vision_module::PlaneData buf;
            buf.center.x = it->center.x;
            buf.center.y = it->center.y;
            buf.center.z = it->center.z;
            buf.normal.x = it->normal.x;
            buf.normal.y = it->normal.y;
            buf.normal.z = it->normal.z;
            buf.upperLeft.x = it->lowerMin.x;
            buf.upperLeft.y = it->lowerMin.y;
            buf.upperLeft.z = it->lowerMin.z;
            buf.lowerLeft.x = it->upperMin.x;
            buf.lowerLeft.y = it->upperMin.y;
            buf.lowerLeft.z = it->upperMin.z;
            buf.upperRight.x = it->lowerMax.x;
            buf.upperRight.y = it->lowerMax.y;
            buf.upperRight.z = it->lowerMax.z;
            buf.lowerRight.x = it->upperMax.x;
            buf.lowerRight.y = it->upperMax.y;
            buf.lowerRight.z = it->upperMax.z;
            res.planes.push_back(buf);
/*
            ROS_INFO("\
[%04d]\n\
CENTER %.2f, %.2f, %.2f, NORMAL %.2f, %.2f %.2f\n\
UL %.2f, %.2f, %.2f, UR %.2f, %.2f, %.2f\n\
LL %.2f, %.2f, %.2f, LR %.2f, %.2f, %.2f [%04d]",
                m_procNum,
                buf.center.x, buf.center.y, buf.center.z,
                buf.normal.x, buf.normal.y, buf.normal.z,
                buf.upperLeft.x, buf.upperLeft.y, buf.upperLeft.z,
                buf.upperRight.x, buf.upperRight.y, buf.upperRight.z,
                buf.lowerLeft.x, buf.lowerLeft.y, buf.lowerLeft.z,
                buf.lowerRight.x, buf.lowerRight.y, buf.lowerRight.z,
                m_procNum);
*/
        }

        //publish
        this->_pub.publish(res);
        //処理結果を保存
        if(m_saveImage){
            cv::imwrite(PLANE_IMAGE_PATH, pd.getPlaneMat());
            cv::imwrite(FILTERED_IMAGE, pd.getFilterMat());
        }
        //処理結果を表示
        if(m_display){
            cv::imshow(CV_FILTERED_WINDOW, pd.getFilterMat());
            cv::imshow(CV_PLANE_WINDOW, pd.getPlaneMat());
            cv::waitKey(10);
        }
    }catch(char *e){
        ROS_INFO("ERROR: %s :%s", this->_nodeName, e);
    }
    }//追加トゥアン
}

template <class SUB, class PUB>
void CProcess<SUB, PUB>::dcfg(vision_module::plane_detectionConfig &config,
            uint32_t level){
    ROS_INFO("\
Reconfigure Request: %s\n\
MAX_DIST: %f\n\
MIN_DIST: %f\n\
ERROR_RANGE: %f\n\
MIN_SEGMENT_SIZE: %d\n\
SEARCH_ITERATION: %d\n\
MAX_GAP: %f\n\
DISPLAY : %s",
        NODE_NAME,
        config.MAX_DIST,
        config.MIN_DIST,
        config.ERROR_RANGE,
        config.MIN_SEGMENT_SIZE,
        config.SEARCH_ITERATION,
        config.MAX_GAP,
        config.DISPLAY?"True":"False"
    );
    m_maxDist           = config.MAX_DIST;
    m_minDist           = config.MIN_DIST;
    m_errorRange        = config.ERROR_RANGE;
    m_minSegmentSize    = config.MIN_SEGMENT_SIZE;
    m_searchIteration   = config.SEARCH_ITERATION;
    m_maxGap            = config.MAX_GAP;

    m_display = config.DISPLAY;
    m_saveImage = config.SAVE_IMAGE;
    if(m_display);
    else cv::destroyAllWindows();
}
