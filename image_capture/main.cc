// Begin ImageCapture.cpp

#include <ros/ros.h>
#include <std_srvs/Trigger.h>//追加トゥアン

#include <vision_module/VisionOrder.h>
#include <dynamic_reconfigure/server.h>
#include <vision_module/image_captureConfig.h>
#include <vision_module/image_capture/config.h>
#include <vision_module/common/VisionProcess.h>
#include <vision_module/common/ROSImageConverter.h>
#include <stdio.h>
#include <time.h>

#define CV_COLOR_WINDOW "Image_Capture_COLOR"
#define CV_DEPTH_WINDOW "Image_Capture_DEPTH"

#define NODE_NAME "image_capture"
#define SUB_TOPIC RGBD_MSG
#define PUB_TOPIC vision_module::VisionOrder::IMAGE_CAPTURE_INFO.c_str()

template <class SUB, class PUB>
class CProcess : public CVisionProcess<SUB, PUB> {
public:
    CProcess(const char *nodeName,
            const char* subTopic, const char* pubTopic);
    ~CProcess();

protected:
    bool m_display;
    bool m_saveImage;
    dynamic_reconfigure::Server<
        vision_module::image_captureConfig> m_server;
    dynamic_reconfigure::Server<
        vision_module::image_captureConfig>::CallbackType m_f;
    bool m_server_command;//追加トゥアン
public:
    //追加トゥアン
    bool start_call(std_srvs::Trigger::Request &req,
        std_srvs::Trigger::Response &res);
    bool stop_call(std_srvs::Trigger::Request &req,
        std_srvs::Trigger::Response &res);
    //追加トゥアン

    void callback(const SUB &msg);
    void dcfg(vision_module::image_captureConfig &config,
            uint32_t level);
};

int main(int argc, char **argv){
    ros::init(argc, argv, NODE_NAME);

    CProcess<sensor_msgs::PointCloud2ConstPtr,
             vision_module::ImageInfo> proc(NODE_NAME, SUB_TOPIC, PUB_TOPIC);

    //追加トゥアン
    ros::NodeHandle n;
    ros::ServiceServer service_start = n.advertiseService("image_capture/start", &CProcess<sensor_msgs::PointCloud2ConstPtr,
             vision_module::ImageInfo>::start_call, &proc);
    ros::ServiceServer service_stop = n.advertiseService("image_capture/stop", &CProcess<sensor_msgs::PointCloud2ConstPtr, 
             vision_module::ImageInfo>::stop_call, &proc);
    //追加トゥアン

    ros::spin();
    return 0;
}

template <class SUB, class PUB>
CProcess<SUB, PUB>::CProcess(const char *nodeName,
        const char* subTopic, const char* pubTopic)
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
    CvPoint3D32f *points = NULL;
    try{
        vision_module::ImageInfo imageInfo;
        int size = msg->width * msg->height;

        //画像サイズを調べる
        if(size != 0){
        }else{
            ROS_INFO("%s : ERROR : Invalid image size",
                    NODE_NAME);
            return;
        }

        points = new CvPoint3D32f[size];

        imageInfo.header = msg->header;
        cv::Mat colorMat(msg->height, msg->width, CV_8UC3);
        cv::Mat depthMat(msg->height, msg->width, CV_8UC1);

        //メッセージをvisionが読める形に変換する
        if(convertToBgr(msg, imageInfo));
        else throw "convertToBgr";
        if(convertToPoints(msg, imageInfo));
        else throw "convertToPoints";
        if(convertToMat(msg, colorMat));
        else throw "convertToMat";
        if(convertToCvPC(msg, points, size));
        else throw "convertToCvPC";
        if(convertToMat(points, depthMat, size));
        else throw "convertToMat_depth";

        //処理内容をパブリッシュする
    //Start
    ros::NodeHandle n("~");
    double image_capture_rate;
    if (n.getParam("image_capture_rate", image_capture_rate))
    {
      ROS_INFO("Got param: %f", image_capture_rate);
    }
    else
    {
      ROS_ERROR("Failed to get param 'image_capture_rate'");
    }
    // n.param("image_capture_rate", image_capture_rate, 1.0);

    ros::Rate pub_rate(image_capture_rate);

        this->_pub.publish(imageInfo);
        // ros::spinOnce();
        pub_rate.sleep();
    //End
        // this->_pub.publish(imageInfo);
        if(m_saveImage){
            cv::imwrite(IMAGE_PATH, colorMat);
            cv::imwrite(DEPTH_IMAGE_PATH, depthMat);
        }
        //処理内容を保存する
        if(points)delete[] points;
        //処理結果を表示する
        if(m_display){
            cv::imshow(CV_COLOR_WINDOW, colorMat);
            cv::imshow(CV_DEPTH_WINDOW, depthMat);
            cv::waitKey(10);
        }
    }catch(char const *e){
        ROS_INFO("ERROR: %s : %s", this->_nodeName, e);
        if(points)delete[] points;
    }
    }//追加トゥアン
}

template <class SUB, class PUB>
void CProcess<SUB, PUB>::dcfg(vision_module::image_captureConfig &config,
            uint32_t level){
    ROS_INFO("\
Reconfigure Request: %s\n\
DISPLAY : %s",
        NODE_NAME,
        config.DISPLAY?"True":"False"
    );
    m_display = config.DISPLAY;
    m_saveImage = config.SAVE_IMAGE;
    if(m_display);
    else{
        cv::destroyAllWindows();
    }
}
