// Begin Main.cpp

#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>//追加トゥアン

#include <vision_module/VisionOrder.h>
#include <vision_module/ImageInfo.h>
#include <vision_module/FaceInfo.h>
#include <dynamic_reconfigure/server.h>
#include <vision_module/common/VisionProcess.h>
#include <vision_module/common/ROSImageConverter.h>
#include <vision_module/face_detectionConfig.h>
#include <vision_module/face_detection/config.h>
#include <vision_module/face_detection/FaceDetection.h>

#define CV_FACE_WINDOW "FaceImage"
#define NODE_NAME "face_detection"
#define SUB_TOPIC vision_module::VisionOrder::IMAGE_CAPTURE_INFO.c_str()
#define PUB_TOPIC vision_module::VisionOrder::FACE_DETECTION_INFO.c_str()

template <class SUB, class PUB>
class CProcess : public CVisionProcess<SUB, PUB> {
public:
    CProcess(const char *nodeName,
            const char *subTopic, const char *pubTopic);
    ~CProcess();

protected:
    CFaceDetection m_fd;

    bool m_display;
    bool m_saveImage;
    bool m_server_command; //追加トゥアン
    dynamic_reconfigure::Server<
        vision_module::face_detectionConfig> m_server;
    dynamic_reconfigure::Server<
        vision_module::face_detectionConfig>::CallbackType m_f;

    int getFaceData(CvPoint3D32f *points,
                    const uint32_t &width,
                    const uint32_t &height,
                    const float &maxDist,
                    const cv::Rect &rect,
                    CvPoint3D32f &pos,
                    CvPoint3D32f &size);

    static bool less(const vision_module::FaceData &left,
                    const vision_module::FaceData &right);

public:
    //追加トゥアン
    bool start_call(std_srvs::Trigger::Request &req,
        std_srvs::Trigger::Response &res);
    bool stop_call(std_srvs::Trigger::Request &req,
        std_srvs::Trigger::Response &res);
    //追加トゥアン
    void callback(const SUB &msg);
    void dcfg(vision_module::face_detectionConfig &config,
            uint32_t level);
};

int main(int argc, char **argv){
    ros::init(argc, argv, NODE_NAME);
    CProcess<vision_module::ImageInfoConstPtr,
             vision_module::FaceInfo> proc(NODE_NAME, SUB_TOPIC, PUB_TOPIC);
     //追加トゥアン
    ros::NodeHandle n;
    ros::ServiceServer service_start = n.advertiseService("face_detection/start", &CProcess<vision_module::ImageInfoConstPtr,
             vision_module::FaceInfo>::start_call, &proc);
    ros::ServiceServer service_stop = n.advertiseService("face_detection/stop", &CProcess<vision_module::ImageInfoConstPtr, 
             vision_module::FaceInfo>::stop_call, &proc);
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

    if(m_fd.setModel(MODEL));
    else ROS_INFO("ERROR : %s : LOAD MODEL FAILED", this->_nodeName);
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
int CProcess<SUB, PUB>::getFaceData(CvPoint3D32f *points,
                            const uint32_t &width,
                            const uint32_t &height,
                            const float &maxDist,
                            const cv::Rect &rect,
                            CvPoint3D32f &pos,
                            CvPoint3D32f &size){

    int xend = rect.x + (rect.width * 3) / 4,
        yend = rect.y + (rect.height * 3) / 4,
        x = rect.x + rect.width / 4, y = rect.y + rect.height / 4;
    float minx = 20.0, maxx = -20.0,
            miny = 20.0, maxy = -20.0,
            minz = 20.0, maxz = -20.0;
    int numPoint = 0;

    if(xend > width || yend > height)return -1;

    pos.x = 0.0;
    pos.y = 0.0;
    pos.z = 0.0;
    for(int i = y; i < yend; i++){
        int ixwidth = i * width;
        for(int j = x; j < xend; j++){
            CvPoint3D32f *point = &points[ixwidth + j];
            if(point->z < maxDist || point->z > 0.4);
            else continue;

            if(minx > point->x)minx = point->x;
            else if(maxx < point->x)maxx = point->x;
            if(miny > point->y)miny = point->y;
            else if(maxy < point->y)maxy = point->y;
            if(minz > point->z)minz = point->z;
            else if(maxz < point->z)maxz = point->z;

            pos.x += point->x;
            pos.y += point->y;
            pos.z += point->z;
            numPoint++;
        }
    }
    pos.x = pos.x / (float)numPoint;
    pos.y = pos.y / (float)numPoint;
    pos.z = pos.z / (float)numPoint;
    size.x = (maxx - minx) * 2;
    size.y = (maxy - miny) * 2;
    size.z = maxz - minz;
    return 0;
}

template <class SUB, class PUB>
bool CProcess<SUB, PUB>::less(const vision_module::FaceData &left,
                        const vision_module::FaceData &right){
    return left.camera.z < right.camera.z;
}

template <class SUB, class PUB>
void CProcess<SUB, PUB>::callback(const SUB &msg){
    if (m_server_command){//追加トゥアン
    cv::Mat color = cv::Mat::zeros(
                    msg->width, msg->height, CV_8UC3);
    CvPoint3D32f *points = new CvPoint3D32f[msg->width * msg->height];
    vision_module::FaceInfo res;

    try{
        //タイムスタンプを引き継ぐ
        res.header = msg->header;

        //メッセージをOpenCV形式にコンバート
        if(convertToMat(msg, color));
        else throw "convertToMat";
        if(convertToCvPC(msg, points, msg->width * msg->height));
        else throw "convertToCvPC";

        //顔検出する
        m_fd.detectFace(color);
        std::vector<cv::Rect> faces = m_fd.getFaces();

        res.faces.clear();
        //検出された顔それぞれについてカメラ座標系での位置を取得する
        for(std::vector<cv::Rect>::iterator it = faces.begin();
            it != faces.end(); it++){

            vision_module::FaceData buf;
            CvPoint3D32f center;
            cv::Mat roi = cv::Mat::zeros(it->height, it->width, CV_8UC3);

            roi = cv::Mat(color, *it).clone();
            buf.width = roi.cols;
            buf.height = roi.rows;
            if(msg->points.size()){
                CvPoint3D32f camera, szwht;
                getFaceData(points, msg->width, msg->height, 8.0, *it,
                            camera, szwht);

                if(szwht.x < 0.25 && szwht.x > 0.08 && szwht.z < 0.1);
                else continue;
                buf.szwht.x = szwht.x;
                buf.szwht.y = szwht.y;
                buf.szwht.z = szwht.z;
                buf.camera.x = camera.x;
                buf.camera.y = camera.y;
                buf.camera.z = camera.z;
            }
            setToVector(roi, buf.bgr);
            m_fd.drawFaces(color, *it);
            res.faces.push_back(buf);
        }

        //顔を近い順に並び替える
        std::sort(res.faces.begin(), res.faces.end(), less);
        //処理結果をpublichする
        this->_pub.publish(res);

        //処理結果を保存する
        if(m_saveImage)cv::imwrite(IMAGE_PATH, color);
        //処理結果を表示する
        if(m_display){
            cv::imshow(CV_FACE_WINDOW, color);
            cv::waitKey(10);
        }
    }catch(char *e){
        ROS_INFO("ERROR : %s : %s", this->_nodeName, e);
        delete[] points;
    }catch(...){
        delete[] points;
    }
    delete[] points;
    }//追加トゥアン
}

template <class SUB, class PUB>
void CProcess<SUB, PUB>::dcfg(vision_module::face_detectionConfig &config,
            uint32_t level){
    ROS_INFO("\
Reconfigure Request: %s\n\
DISPLAY : %s",
        NODE_NAME,
        config.DISPLAY?"True":"False"
    );
    m_display =   config.DISPLAY;
    m_saveImage = config.SAVE_IMAGE;
    if(m_display);
    else cv::destroyAllWindows();
}
