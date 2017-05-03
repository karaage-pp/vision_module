// Begin Main.cpp

#include <stdio.h>
#include <std_srvs/Trigger.h>//追加トゥアン

#include <ros/ros.h>
#include <vision_module/VisionOrder.h>
#include <vision_module/ObjectInfo.h>
#include <dynamic_reconfigure/server.h>
#include <vision_module/object_recognitionConfig.h>
#include <vision_module/object_recognition/config.h>
#include <vision_module/common/VisionProcess.h>
#include <vision_module/common/ROSImageConverter.h>
#include <vision_module/common/TrainSVM.h>
#include <vision_module/object_recognition/Alexnet.h>
#include <vision_module/object_recognition/ImageProc.h>

#define CV_RECOGNIZED_WINDOW "RecognizedObjects"
#define NODE_NAME "object_recognition"
#define SUB_TOPIC vision_module::VisionOrder::OBJECT_DETECTION_INFO.c_str()
#define PUB_TOPIC vision_module::VisionOrder::OBJECT_RECOGNITION_INFO.c_str()

template <class SUB, class PUB>
class CProcess : public CVisionProcess<SUB, PUB>, CTrainSVM, CImageProc {
public:
    CProcess(const char *nodeName,
            const char *subTopic, const char *pubTopic);
    ~CProcess();

protected:
    CAlexnet m_net;

    int m_numCand;
    bool m_pubFeatures;
    bool m_display;
    bool m_sendImage;
    bool m_saveImage;
    bool m_server_command; //追加トゥアン

    dynamic_reconfigure::Server<
        vision_module::object_recognitionConfig> m_server;
    dynamic_reconfigure::Server<
        vision_module::object_recognitionConfig>::CallbackType m_f;

public:
  //追加トゥアン
  bool start_call(std_srvs::Trigger::Request &req,
      std_srvs::Trigger::Response &res);
  bool stop_call(std_srvs::Trigger::Request &req,
      std_srvs::Trigger::Response &res);
  //追加トゥアン
    void callback(const SUB &msg);
    void dcfg(vision_module::object_recognitionConfig &config,
            uint32_t level);
};

int main(int argc, char **argv){
    ros::init(argc, argv, NODE_NAME);
    CProcess<vision_module::ObjectInfoConstPtr,
             vision_module::ObjectInfo> proc(NODE_NAME, SUB_TOPIC, PUB_TOPIC);
     //追加トゥアン
    ros::NodeHandle n;
    ros::ServiceServer service_start = n.advertiseService("object_recognition/start", &CProcess<vision_module::ObjectInfoConstPtr,
             vision_module::ObjectInfo>::start_call, &proc);
    ros::ServiceServer service_stop = n.advertiseService("object_recognition/stop", &CProcess<vision_module::ObjectInfoConstPtr,
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
                            m_net(MODEL, "alexnet_int", PYWORKSPACE, LABEL) {

    setSVM(SVM_MODEL, 4096);
    m_server_command = true;//追加トゥアン
    m_f = boost::bind(&CProcess::dcfg, this, _1, _2);
    m_server.setCallback(m_f);
}

template <class SUB, class PUB>
CProcess<SUB, PUB>::~CProcess(){
    delSVM();
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
    vision_module::ObjectInfo res;
    const int size = msg->objects.size();
    std::vector<scoreIndex> result;
    std::vector<std::string> names;
    std::vector<float> features;
    std::vector<float> probs;
    std::vector<float> genScores;
    std::string getLabelName;
    int total_width = 0;
    int max_height = 0;

    //タイムスタンプを受け継ぐ
    res.header = msg->header;
    copyTo(msg, res);

    //受け取った画像の幅の和を算出
    for(int i = 0; i < size; i++){
        if(res.objects[i].height > max_height)
            max_height = res.objects[i].height;
        total_width += res.objects[i].width;
    }

    cv::Mat objRecogMat = cv::Mat::zeros(max_height, total_width, CV_8UC3);
    cv::Rect roi_rect;

    //物体毎に処理
    for(int i = 0; i < size; i++){
        vision_module::ObjectData *obj = &(res.objects[i]);
        cv::Mat roi = cv::Mat::zeros(obj->height, obj->width, CV_8UC3);

        vision_module::NBest specific;
        vision_module::NBest generic;
        result.clear();
        names.clear();
        genScores.clear();
        features.clear();

        //画像データをOpenCV形式に変更
        setToUINT8(obj->bgr, roi.data, roi.cols * roi.rows * 3);

        roi_rect.width = obj->width;
        roi_rect.height = obj->height;
        cv::Mat temp(objRecogMat, roi_rect);
        roi.copyTo(temp);
        roi_rect.x += roi.cols;

        //画像特徴量を抽出
        features = m_net.extractFeatures(roi);
        probs = m_net.classify(roi);
        //特徴量をSVMにて分類
        classify(features, result);
        for(int j = 0; j < m_numCand; j++){
            specific.id = result[j].index;
            specific.score = result[j].score;

            obj->specific.push_back(specific);
        }
        if(!obj->generic.size()){
            for(int j = 1; j <= m_numCand; j++){
                generic.id = m_net.getId(j);
                generic.name = m_net.getLabelName(j);
                generic.score = m_net.getProb(j);

                obj->generic.push_back(generic);
            }
        }

        int width = roi.cols / 2, height = roi.rows / 2;
        int x = width / 2, y = height / 2;
        //物体画像の中心部を切り取り
        cv::Mat center(roi, cv::Rect(x, y, width, height));
        //画像のカラーヒストグラムを計算
        calcHistAsHSV(center);
        //処理結果を代入
        obj->color = getNBestName(0);

        if(!m_pubFeatures);
        else obj->features = features;

        //処理結果をプリント
        for(int j = 0; j < obj->generic.size(); j++){
            ROS_INFO("%s : OBJ %2d: GENERIC NAME '%s', SCORE %.2f",
                        this->_nodeName, i,
                        res.objects[i].generic[j].name.c_str(),
                        res.objects[i].generic[j].score);
        }
        for(int j = 0; j < obj->specific.size(); j++){
            ROS_INFO("%s : OBJ %2d: SPECIFIC ID %2d, SCORE %.2f",
                        this->_nodeName, i,
                        res.objects[i].specific[j].id,
                        res.objects[i].specific[j].score);
        }
        ROS_INFO("\
%s : POS: %.2f, %.2f, %.2f SIZE: %.2f, %.2f, %.2f COLOR %s",
                    this->_nodeName,
                    res.objects[i].camera.x,
                    res.objects[i].camera.y,
                    res.objects[i].camera.z,
                    res.objects[i].szwht.x,
                    res.objects[i].szwht.y,
                    res.objects[i].szwht.z,
                    res.objects[i].color.c_str());
        if(!m_saveImage)obj->bgr.clear();
    }

    //publish
    this->_pub.publish(res);

    if(size){
        //処理結果を保存
        if(m_saveImage)cv::imwrite(IMAGE_PATH, objRecogMat);
        //処理結果を表示
        if(m_display){
            cv::imshow(CV_RECOGNIZED_WINDOW, objRecogMat);
            cv::waitKey(10);
        }
    }
    }//追加トゥアン

}

template <class SUB, class PUB>
void CProcess<SUB, PUB>::dcfg(vision_module::object_recognitionConfig &config,
            uint32_t level){
    ROS_INFO("\
Reconfigure Request: %s\n\
NUM_CAND : %d\n\
PUB_FEATURES : %d\n\
DISPLAY : %s",
        NODE_NAME,
        config.NUM_CAND,
        config.PUB_FEATURES,
        config.DISPLAY?"True":"False"
    );
    m_numCand =  config.NUM_CAND;
    m_pubFeatures =   config.PUB_FEATURES;
    m_display =   config.DISPLAY;
    m_sendImage = config.SEND_IMAGE;
    m_saveImage = config.SAVE_IMAGE;
    if(m_display);
    else cv::destroyAllWindows();
}
