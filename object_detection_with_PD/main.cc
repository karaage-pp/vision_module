// Begin Main.cpp

#include <ros/ros.h>
#include <std_srvs/Trigger.h>//追加トゥアン

#include <vision_module/ImageInfo.h>
#include <vision_module/ObjectInfo.h>
#include <vision_module/VisionOrder.h>
#include <dynamic_reconfigure/server.h>
#include <vision_module/object_detection_with_PDConfig.h>
#include <vision_module/object_detection_with_PD/config.h>
#include <vision_module/object_detection_with_PD/ObjectDetection.h>
#include <vision_module/common/VisionProcess.h>
#include <vision_module/common/ROSImageConverter.h>
#include <vector>
#include <algorithm>

#define CV_OBJECT_WINDOW "ObjectImage"
#define CV_RAW_WINDOW "RawObjectImage"
#define NODE_NAME "object_detection_with_PD"
#define SUB_TOPIC vision_module::VisionOrder::PLANE_DETECTION_INFO.c_str()
#define PUB_TOPIC vision_module::VisionOrder::OBJECT_DETECTION_INFO.c_str()

vision_module::Vector operator-(const vision_module::Vector &vec1,
                                const vision_module::Vector &vec2){

    vision_module::Vector result;
    result.x = vec1.x - vec2.x;
    result.y = vec1.y - vec2.y;
    result.z = vec1.z - vec2.z;

    return result;
}

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
    float m_maxWidth;
    float m_maxHeight;
    float m_minHeight;
    float m_maxDepth;
    int m_numPoint;
    float m_maxDist;

    bool m_display;
    bool m_saveImage;
    bool m_server_command; //追加トゥアン

    dynamic_reconfigure::Server<
        vision_module::object_detection_with_PDConfig> m_server;
    dynamic_reconfigure::Server<
        vision_module::object_detection_with_PDConfig>::CallbackType m_f;

    float dotProduct(vision_module::Vector vec1, vision_module::Vector vec2);

public:
    //追加トゥアン
    bool start_call(std_srvs::Trigger::Request &req,
        std_srvs::Trigger::Response &res);
    bool stop_call(std_srvs::Trigger::Request &req,
        std_srvs::Trigger::Response &res);
    //追加トゥアン
    void callback(const SUB &msg);
    void dcfg(vision_module::object_detection_with_PDConfig &config,
            uint32_t level);
};

int main(int argc, char **argv){
    ros::init(argc, argv, NODE_NAME);
    CProcess<vision_module::ObjectInfoConstPtr,
             vision_module::ObjectInfo> proc(NODE_NAME, SUB_TOPIC, PUB_TOPIC);
     //追加トゥアン
    ros::NodeHandle n;
    ros::ServiceServer service_start = n.advertiseService("object_detection_with_PD/start", &CProcess<vision_module::ObjectInfoConstPtr,
             vision_module::ObjectInfo>::start_call, &proc);
    ros::ServiceServer service_stop = n.advertiseService("object_detection_with_PD/stop", &CProcess<vision_module::ObjectInfoConstPtr,
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
    m_f = boost::bind(&CProcess<SUB, PUB>::dcfg, this, _1, _2);
    m_server.setCallback(m_f);

    ROS_INFO("CONFIG: WIDTH %.2f, HEIGHT %.2f, DEPTH %.2f",
                m_maxWidth, m_maxHeight, m_maxDepth);
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

template <class SUB, class PUB>
void CProcess<SUB, PUB>::callback(const SUB &msg){
    if (m_server_command){//追加トゥアン
    int image_size = msg->width * msg->height;
    CObjectDetection od(msg->width, msg->height);
    CvPoint3D32f points[image_size];
    int label[image_size];
    std::vector<OD::ObjectInfo> objectInfo;
    vision_module::ObjectInfo res;
    cv::Mat colorMat = cv::Mat::zeros(msg->height, msg->width, CV_8UC3);
    cv::Mat objectMat;
    int objCount = 0;

    //タイムスタンプを受け継ぐ
    res.header = msg->header;
    try{

    //メッセージをOpenCVの形に変換
    if(convertToCvPC(msg, points, image_size));
    else throw "convertTo";
    if(convertToMat(msg, colorMat));
    else throw "convertToMat";
    if(setLabel(msg, label, image_size));
    else throw "setLabel";

    //遠い点を取り除く
    od.filter(label, points, m_maxDist);
    //物体検出する
    od.detectObject(label, points, 100);
    objectInfo.clear();
    //処理結果を代入する
    objectInfo = od.getObjectInfo();

    objectMat = colorMat.clone();
    res.objects.clear();
    int numPlanes = msg->planes.size();
    //検出された物体毎に処理を行なう
    for(int i = 0; i < objectInfo.size(); i++){
        OD::ObjectInfo obj = objectInfo[i];
        vision_module::ObjectData data;

        //物体の点群サイズが小さい場合は破棄する
        if(obj.numPoint >
                (int)(m_numPoint / (obj.center.z * obj.center.z)));
//            printf("OBJECT NUM_POINT = %d\n", obj.numPoint);
        else{
            printf("NUM_POINT = %d\n", obj.numPoint);
            continue;
        }

        //物体の幅，高さ等を計算
        data.szwht.x = obj.right.p3D.x - obj.left.p3D.x;
        data.szwht.y = obj.bottom.p3D.y - obj.top.p3D.y;
        data.szwht.z = obj.rear.p3D.z - obj.front.p3D.z;
        //物体サイズが大きい場合は破棄
        if(data.szwht.x > m_maxWidth){
            printf("LINE %d\n NUM_POINT %d SIZE %f\n", __LINE__,
                                                        obj.numPoint,
                                                        data.szwht.x);
            continue;
        }
        if(data.szwht.y > m_maxHeight){
            printf("LINE %d\n NUM_POINT %d SIZE %f\n", __LINE__,
                                                        obj.numPoint,
                                                        data.szwht.y);
            continue;
        }
        if(data.szwht.z > m_maxDepth){
            printf("LINE %d\n NUM_POINT %d SIZE %f\n", __LINE__,
                                                        obj.numPoint,
                                                        data.szwht.z);
            continue;
        }

        data.normal.x = 0.0;
        data.normal.y = 0.0;
        data.normal.z = 0.0;
        data.planeNum = -1;
        //どの平面上に乗っているか調べる
        for(int planeNum = 0; planeNum < numPlanes; planeNum++){
            CvPoint3D32f center, normal;
            center.x = msg->planes[planeNum].center.x;
            center.y = msg->planes[planeNum].center.y;
            center.z = msg->planes[planeNum].center.z;
            normal.x = msg->planes[planeNum].normal.x;
            normal.y = msg->planes[planeNum].normal.y;
            normal.z = msg->planes[planeNum].normal.z;
            if(od.isOnThisPlane(center, normal, i, points)){
                data.normal.x = normal.x;
                data.normal.y = normal.y;
                data.normal.z = normal.z;
                data.planeNum = planeNum;
                break;
            }
        }

        //どの平面にも乗っていない場合は破棄
        if(data.planeNum == -1){
            continue;
        }
        float dot = 0.0;
        data.camera.x = obj.center.x;
        data.camera.y = obj.center.y;
        data.camera.z = obj.center.z;

        //平面から物体までの距離を計算
        //平面にめり込んでいる場合は破棄
        dot = dotProduct(msg->planes[data.planeNum].upperLeft
                                - msg->planes[data.planeNum].lowerLeft,
                            data.camera
                            - msg->planes[data.planeNum].lowerLeft);
        if(dot < 0.0){
            printf("LINE %d\n", __LINE__);
            continue;
        }
        dot = dotProduct(msg->planes[data.planeNum].upperRight
                                - msg->planes[data.planeNum].upperLeft,
                            data.camera
                            - msg->planes[data.planeNum].upperLeft);
        if(dot < 0.0){
            printf("LINE %d\n", __LINE__);
            continue;
        }
        dot = dotProduct(msg->planes[data.planeNum].lowerRight
                                - msg->planes[data.planeNum].upperRight,
                            data.camera
                            - msg->planes[data.planeNum].upperRight);
        if(dot < 0.0){
            printf("LINE %d\n", __LINE__);
            continue;
        }
        dot = dotProduct(msg->planes[data.planeNum].lowerLeft
                                - msg->planes[data.planeNum].lowerRight,
                            data.camera
                            - msg->planes[data.planeNum].lowerRight);
        if(dot < 0.0){
            printf("LINE %d\n", __LINE__);
            continue;
        }

        //変更宮澤 切り出し画像をの幅を広げる jsai学習用に緊急で変更 常に使うことがないように
        /*if (obj.bottom.y < 410)
          obj.bottom.y += 5;
        if (obj.top.y > 10 )
          obj.top.y -= 5;
        if (obj.right.x < 500)
        obj.right.x += 5;
        if (obj.left.x > 10)
          obj.left.x -= 5;*/

        //物体画像を切り出す
        cv::Mat roi = cv::Mat::zeros(obj.bottom.y - obj.top.y,
                                    obj.right.x - obj.left.x, CV_8UC3);
        roi = cv::Mat(colorMat,
                        cv::Rect(obj.left.x, obj.top.y,
                            obj.right.x - obj.left.x,
                            obj.bottom.y - obj.top.y)).clone();
        cv::rectangle(objectMat, cv::Point(obj.left.x, obj.top.y),
                                cv::Point(obj.right.x, obj.bottom.y),
                                cv::Scalar(0, 0, 200), 3, 4);

        //処理結果を代入
        data.width = roi.cols;
        data.height = roi.rows;
        setToVector(roi, data.bgr);
        res.objects.push_back(data);

        objCount++;
    }

    //近い順に並び替える
    std::sort(res.objects.begin(), res.objects.end(), closer);
    //publishする
    this->_pub.publish(res);

    //処理結果をプリント
    printf("------- NUM OF OBJECTS: %d-------\n", objCount);

    for(int i = 0; i < objCount; i++){
        vision_module::ObjectData *data = &res.objects[i];
        ROS_INFO("NUM_OBJ %d:\n\
POS %.2f, %.2f, %.2f: NORMAL %.2f, %.2f, %.2f: PlaneNum %2d\n\
SIZE %.2f, %.2f, %.2f",
                    i,
                    data->camera.x, data->camera.y, data->camera.z,
                    data->normal.x, data->normal.y, data->normal.z,
                    data->planeNum,
                    data->szwht.x, data->szwht.y, data->szwht.z);
    }
    printf("\n");

    //処理結果を保存
    if(m_saveImage){
        cv::imwrite(OBJECT_IMAGE, objectMat);
        cv::imwrite(RAW_OBJECT_IMAGE, od.getObjectMat());
    }
    //処理結果を表示
    if(m_display){
        cv::imshow(CV_RAW_WINDOW, od.getObjectMat());
        cv::imshow(CV_OBJECT_WINDOW, objectMat);
        cv::waitKey(1);
    }
    }catch(char *e){
    }
    }//追加トゥアン
}

template <class SUB, class PUB>
void CProcess<SUB, PUB>::dcfg(vision_module::object_detection_with_PDConfig &config,
            uint32_t level){
    ROS_INFO("\
Reconfigure Request: %s\n\
MAX_WIDTH : %f\n\
MAX_HEIGHT: %f\n\
MIN_HEIGHT: %f\n\
MAX_DEPTH : %f\n\
MIN_POINT : %d\n\
MAX_DIST  : %f\n\
DISPLAY : %s",
        NODE_NAME,
        config.MAX_WIDTH,
        config.MAX_HEIGHT,
        config.MIN_HEIGHT,
        config.MAX_DEPTH,
        config.MIN_POINT,
        config.MAX_DIST,
        config.DISPLAY?"True":"False"
    );
    m_maxWidth =  config.MAX_WIDTH;
    m_maxHeight = config.MAX_HEIGHT;
    m_minHeight = config.MIN_HEIGHT;
    m_maxDepth =  config.MAX_DEPTH;
    m_numPoint =  config.MIN_POINT;
    m_maxDist =   config.MAX_DIST;
    m_display =   config.DISPLAY;
    m_saveImage = config.SAVE_IMAGE;
    if(m_display);
    else cv::destroyAllWindows();
}
