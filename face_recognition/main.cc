
#include <stdio.h>
#include <time.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>//追加トゥアン

#include <vision_module/VisionOrder.h>
#include <vision_module/FaceInfo.h>
#include <dynamic_reconfigure/server.h>
#include <vision_module/face_recognitionConfig.h>
#include <vision_module/face_recognition/config.h>
#include <vision_module/face_recognition/FaceNet.h>
#include <vision_module/common/ROSImageConverter.h>
#include <vision_module/common/MySys.h>
#include <vision_module/common/VisionProcess.h>
#include <vision_module/common/TrainSVM.h>

#define CV_WINDOW_NAME "faces"
#define NODE_NAME "face_recognition"
#define SUB_TOPIC vision_module::VisionOrder::FACE_DETECTION_INFO.c_str()
#define PUB_TOPIC vision_module::VisionOrder::FACE_RECOGNITION_INFO.c_str()

#define NUM_CAND 2

template <class SUB, class PUB>
class CProcess : public CVisionProcess<SUB, PUB>, CTrainSVM {
public:
    CProcess(const char *nodeName,
            const char *subTopic, const char *pubTopic);
    ~CProcess();

protected:
    std::string m_fullNodeName;
    CFaceNet m_net;

    cv::Mat m_face;

    bool m_display;
    bool m_sendImage;
    bool m_saveImage;
    int m_numPict;
    bool m_server_command; //追加トゥアン

    dynamic_reconfigure::Server<
        vision_module::face_recognitionConfig> m_server;
    dynamic_reconfigure::Server<
        vision_module::face_recognitionConfig>::CallbackType m_f;


    int m_numEntries;

    char m_pictDir[256];
    char m_dataDir[256];
    char m_modelDir[256];
    char m_learnedDir[256];
    char m_tempDir[256];
    bool m_isLearning;
    bool m_learnOrder;
    CvPoint3D32f m_beforePos;

    bool faceLearn(const cv::Mat &roi);

public:
//CVProcess
    //追加トゥアン
    bool start_call(std_srvs::Trigger::Request &req,
        std_srvs::Trigger::Response &res);
    bool stop_call(std_srvs::Trigger::Request &req,
        std_srvs::Trigger::Response &res);
    //追加トゥアン
    void callback(const vision_module::FaceInfoConstPtr &msg);
    void dcfg(vision_module::face_recognitionConfig &config,
            uint32_t level);

//CMultiThread
    void setFace(const cv::Mat &face);
};

int main(int argc, char **argv){
    ros::init(argc, argv, NODE_NAME);
    CProcess<vision_module::FaceInfoConstPtr,
             vision_module::FaceInfo> proc(NODE_NAME, SUB_TOPIC, PUB_TOPIC);
     //追加トゥアン
    ros::NodeHandle n;
    ros::ServiceServer service_start = n.advertiseService("face_recognition/start", &CProcess<vision_module::FaceInfoConstPtr,
             vision_module::FaceInfo>::start_call, &proc);
    ros::ServiceServer service_stop = n.advertiseService("face_recognition/stop", &CProcess<vision_module::FaceInfoConstPtr,
             vision_module::FaceInfo>::stop_call, &proc);
    //追加トゥアン
    ros::spin();
    return 0;
}

template <class SUB, class PUB>
CProcess<SUB, PUB>::CProcess(const char *nodeName,
        const char *subTopic, const char *pubTopic)
                        : CVisionProcess<SUB, PUB>(nodeName,
                            subTopic, pubTopic),
                            m_net(FACE_MODEL, PY_INTERFACE,
                                    PYWORKSPACE) {
    m_server_command = true;//追加トゥアン

    m_net.setAgeList(AGE_LIST);
    m_net.setGenderList(GENDER_LIST);

    m_f = boost::bind(&CProcess::dcfg, this, _1, _2);
    m_server.setCallback(m_f);

    m_isLearning = false;
    m_learnOrder = false;

    time_t now;
    struct tm *ltm;
    time(&now);
    ltm = localtime(&now);
    sprintf(m_tempDir, "%s/%s:%d%02d%02d%02d%02d%02d_FACE",
                WORKSPACE,
                TEMP_DIR,
                ltm->tm_year + 1900,
                ltm->tm_mon + 1, ltm->tm_mday,
                ltm->tm_hour, ltm->tm_min, ltm->tm_sec);
    sprintf(m_pictDir, "%s/%s", m_tempDir, PICT_DIR);
    sprintf(m_dataDir, "%s/%s", m_tempDir, DATA_DIR);
    sprintf(m_modelDir, "%s/%s", m_tempDir, MODEL_DIR);
    sprintf(m_learnedDir, "%s/%s", m_tempDir, LEARNED_DIR);
    myMKDIR(m_tempDir, 0755);
    myMKDIR(m_pictDir, 0755);
    myMKDIR(m_dataDir, 0755);
    myMKDIR(m_modelDir, 0755);
    myMKDIR(m_learnedDir, 0755);
    myCP(FEATURE_FILE, m_dataDir);
    m_numEntries = 1;

    m_fullNodeName = ros::this_node::getName();
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
bool CProcess<SUB, PUB>::faceLearn(const cv::Mat &roi){
    if(roiBuffering(roi, m_numEntries, m_pictDir, m_numPict)){
        std::vector<std::string> entries;
        entries = myDIRENT(m_pictDir);

        char buf[256] = "";
        std::vector<float> features;
        ROS_INFO("%s : EXTRACTING FEATURES", this->_nodeName);
        for(std::vector<std::string>::iterator it = entries.begin();
                it != entries.end(); it++){

            if(checkType(it->c_str(), PICT_PREFIX));
            else continue;
            sprintf(buf, "%s/%s:%03d:0000.txt",
                        m_dataDir, FEAT_PREFIX, m_numEntries);
            FILE *fp = fopen(buf, "a");
            if(fp);
            else{
                printf("ERROR : %s : LINE %d", this->_nodeName, __LINE__);
                continue;
            }
            for(;it != entries.end(); it++){
                if(checkType(it->c_str(), PICT_PREFIX));
                else continue;
                sprintf(buf, "%s/%s", m_pictDir, it->c_str());
                cv::Mat mat = cv::imread(buf);
                features.clear();
                features = m_net.extractFeatures(mat);
                for(std::vector<float>::iterator f = features.begin();
                        f != features.end(); f++){

                    fprintf(fp, "%f\t", *f);
                }
                fprintf(fp, "\n");
            }
            it--;
            fclose(fp);
        }
        char src[256] = "", dst[256] = "",
                featFile[256] = "", labelFile[256] = "", hist[256] = "",
                model[256] = "";
        sprintf(src, "%s/*", m_pictDir);
        sprintf(dst, "%s/", m_learnedDir);
        sprintf(featFile, "%s/features.txt", m_dataDir);
        sprintf(labelFile, "%s/label.txt", m_dataDir);
        sprintf(hist, "%s/hists.svm.txt", m_dataDir);
        sprintf(model, "%s/faceModel.libsvm", m_modelDir);
        makeDataSet(m_dataDir, featFile, labelFile);
        makeHists(featFile, labelFile, hist);
        if(trainSVM(hist, model) == 0)changeSVM(model, 4096);
        ROS_INFO("%s : LEARNING FINISHED", this->_nodeName);
        myMV(src, dst);
        m_numEntries++;
        return true;
    }
    return false;
}

template <class SUB, class PUB>
void CProcess<SUB, PUB>::callback(const vision_module::FaceInfoConstPtr &msg){
    if (m_server_command){//追加トゥアン
    vision_module::FaceInfo res;
    int size = msg->faces.size();
    int total_width = 0;
    int max_height = 0;
    std::vector<scoreIndex> result;
    std::vector<float> features;
    std::vector<std::string> ages;
    std::vector<float> ageScores;
    std::vector<std::string> gender;
    std::vector<float> genderScores;

    //顔学習のテスト
    //顔を学習するための処理を書く
    //現在未完成
    if(m_learnOrder){
        char command[256];
        sprintf(command,
            "rosrun dynamic_reconfigure dynparam set %s FACE_LEARN 0",
            m_fullNodeName.c_str());
        system(command);
    }

    //タイムスタンプを受け継ぐ
    res.header = msg->header;
    copyTo(msg, res);

    //顔毎に認識処理を行なう
    for(int i = 0; i < size; i++){
        if(res.faces[i].height > max_height)
            max_height = res.faces[i].height;
        total_width += res.faces[i].width;
    }
    cv::Mat facesMat = cv::Mat::zeros(max_height, total_width, CV_8UC3);
    cv::Rect roi_rect;

    for(int i = 0; i < size; i++){
        vision_module::FaceData *face = &(res.faces[i]);
        cv::Mat roi = cv::Mat::zeros(face->height, face->width, CV_8UC3);
        vision_module::NBest generic;
        vision_module::NBest specific;

        result.clear();
        features.clear();
        ages.clear();
        ageScores.clear();
        gender.clear();
        genderScores.clear();

        //メッセージをOpenCVの形に変換
        setToUINT8(face->bgr, roi.data, roi.cols * roi.rows * 3);

        roi_rect.width = roi.cols;
        roi_rect.height = roi.rows;
        cv::Mat temp(facesMat, roi_rect);
        roi.copyTo(temp);
        roi_rect.x += roi.cols;

        printf("SIZE: %d, %d\n", roi.cols, roi.rows);
        printf("miyazawa0");
        //顔の特徴量を抽出
        features = m_net.extractFeatures(roi); //この関数で落ちてる
        printf("miyazawa1");
        //年齢を調べる
        m_net.how(roi, ages, ageScores, 1);
        printf("miyazawa2");
        //性別を調べる
        m_net.which(roi, gender, genderScores, 1);
        printf("miyazawa3");

        //メッセージに処理結果を代入
        face->age = atoi(ages[0].c_str());
        face->gender = gender[0].c_str();
        printf("miyazawa4");

        //特定人認識を行ない，結果を代入する
        int c = classify(features, result);
        for(int j = 0; j < NUM_CAND; j++){
            if(c == 0){
                specific.id = result[j].index;
                specific.score = result[j].score;
                face->specific.push_back(specific);
            }
        }
        printf("miyazawa5");

        //処理結果を出力
        for(int j = 0, e = res.faces[i].specific.size(); j < e; j++){
            ROS_INFO("%s : FACE SPECIFIC %2d: ID %2d, SCORE %.2f",
                        this->_nodeName, i,
                        res.faces[i].specific[j].id,
                        res.faces[i].specific[j].score);
        }
        ROS_INFO("%s : FACE %2d: AGE '%2d', GENDER '%s'",
                        this->_nodeName, i,
                        res.faces[i].age, res.faces[i].gender.c_str());
        ROS_INFO(
        "\n%s : FACE %2d: POS %.2f, %.2f, %.2f SIZE %.2f, %.2f, %.2f",
                    this->_nodeName, i,
                    res.faces[i].camera.x,
                    res.faces[i].camera.y,
                    res.faces[i].camera.z,
                    res.faces[i].szwht.x,
                    res.faces[i].szwht.y,
                    res.faces[i].szwht.z);
        if(!m_saveImage)face->bgr.clear();
    }
    this->_pub.publish(res);
    if(size){
        printf("si: %d, disp: %d\n", m_saveImage, m_display);
        if(m_saveImage)cv::imwrite(IMAGE_PATH, facesMat);
        if(m_display){
            cv::imshow(CV_WINDOW_NAME, facesMat);
            cv::waitKey(10);
        }
    }
    }//追加トゥアン
}

template <class SUB, class PUB>
void CProcess<SUB, PUB>::dcfg(vision_module::face_recognitionConfig &config,
            uint32_t level){
    ROS_INFO("\
Reconfigure Request: %s\n\
NUM_PICT: %d\n\
FACE_LEARN : %d\n\
DISPLAY : %s",
        NODE_NAME,
        config.NUM_PICT,
        config.FACE_LEARN,
        config.DISPLAY?"True":"False"
    );
    m_numPict = config.NUM_PICT;
    m_display = config.DISPLAY;
    m_sendImage = config.SEND_IMAGE;
    m_saveImage = config.SAVE_IMAGE;
    m_learnOrder = config.FACE_LEARN;
    if(config.FACE_LEARN){
        config.FACE_LEARN = false;
    }
    if(m_display);
    else cv::destroyAllWindows();
}

template <class SUB, class PUB>
void CProcess<SUB, PUB>::setFace(const cv::Mat &face){
    m_face = face.clone();
}
