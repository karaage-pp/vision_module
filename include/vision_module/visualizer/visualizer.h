
#include <QtGui>
#include <QWidget>
#include <QMainWindow>
#include <ros/ros.h>
#include <vision_module/visualizer/ui_visualizer.h>
#include <vision_module/VisionOrder.h>
#include <vision_module/ImageInfo.h>
#include <vision_module/ObjectInfo.h>
#include <vision_module/FaceInfo.h>
#include <string>

#define VISION_IMAGE vision_module::VisionOrder::IMAGE_CAPTURE_INFO.c_str()
#define VISION_PLANE vision_module::VisionOrder::PLANE_DETECTION_INFO.c_str()
#define OBJECT_DETECTION vision_module::VisionOrder::OBJECT_DETECTION_INFO.c_str()
#define OBJECT_RECOGNITION vision_module::VisionOrder::OBJECT_RECOGNITION_INFO.c_str()
#define FACE_DETECTION vision_module::VisionOrder::FACE_DETECTION_INFO.c_str()
#define FACE_RECOGNITION vision_module::VisionOrder::FACE_RECOGNITION_INFO.c_str()

class CVisualizer : public QWidget, private Ui_Form {
    Q_OBJECT

public:
    CVisualizer(QWidget* parent = 0);
    ~CVisualizer();

    void cb_IC(const vision_module::ImageInfoConstPtr &msg);
    void cb_PD(const vision_module::ObjectInfoConstPtr &msg);
    void cb_OD(const vision_module::ObjectInfoConstPtr &msg);
    void cb_DARKNET(const vision_module::ObjectInfoConstPtr &msg);
    void cb_OR(const vision_module::ObjectInfoConstPtr &msg);
    void cb_FD(const vision_module::FaceInfoConstPtr &msg);
    void cb_FR(const vision_module::FaceInfoConstPtr &msg);

protected:
    ros::NodeHandle _nh;
    ros::Subscriber _subIC;
    ros::Subscriber _subPD;
    ros::Subscriber _subOD;
    ros::Subscriber _subDARKNET;
    ros::Subscriber _subOR;
    ros::Subscriber _subFD;
    ros::Subscriber _subFR;

private slots:
    void showImage_IC(const bool &val);
    void showImage_PD(const bool &val);
    void showImage_OD(const bool &val);
    void showImage_OR(const bool &val);
    void showImage_DARKNET(const bool &val);
    void showImage_FD(const bool &val);
    void showImage_FR(const bool &val);

};

