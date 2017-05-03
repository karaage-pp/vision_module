
#include <vision_module/visualizer/visualizer.h>

CVisualizer::CVisualizer(QWidget* parent) 
    : QWidget(parent) {

    setupUi(this);
    connect(ic_cbox, SIGNAL(toggled(bool)), 
                this, SLOT(showImage_IC(bool)));
    connect(pd_cbox, SIGNAL(toggled(bool)), 
                this, SLOT(showImage_PD(bool)));
    connect(od_cbox, SIGNAL(toggled(bool)), 
                this, SLOT(showImage_OD(bool)));
    connect(or_cbox, SIGNAL(toggled(bool)), 
                this, SLOT(showImage_OR(bool)));
    connect(darknet_cbox, SIGNAL(toggled(bool)), 
                this, SLOT(showImage_DARKNET(bool)));
    connect(fd_cbox, SIGNAL(toggled(bool)), 
                this, SLOT(showImage_FD(bool)));
    connect(fr_cbox, SIGNAL(toggled(bool)), 
                this, SLOT(showImage_FR(bool)));

}

CVisualizer::~CVisualizer(){
    if(_subIC)_subIC.shutdown();
    if(_subPD)_subPD.shutdown();
    if(_subOD)_subOD.shutdown();
    if(_subDARKNET)_subDARKNET.shutdown();
    if(_subOR)_subOR.shutdown();
    if(_subFD)_subFD.shutdown();
    if(_subFR)_subFR.shutdown();
}

void CVisualizer::cb_IC(const vision_module::ImageInfoConstPtr &msg){
}

void CVisualizer::cb_PD(const vision_module::ObjectInfoConstPtr &msg){
}

void CVisualizer::cb_OD(const vision_module::ObjectInfoConstPtr &msg){
}

void CVisualizer::cb_DARKNET(const vision_module::ObjectInfoConstPtr &msg){
}

void CVisualizer::cb_OR(const vision_module::ObjectInfoConstPtr &msg){
}

void CVisualizer::cb_FD(const vision_module::FaceInfoConstPtr &msg){
}

void CVisualizer::cb_FR(const vision_module::FaceInfoConstPtr &msg){
}


void CVisualizer::showImage_IC(const bool &val){
    char command[256];
    if(val){
        if(!_subIC)_subIC = _nh.subscribe(VISION_IMAGE, 1, 
                &CVisualizer::cb_IC, this);
    }else{
        if(_subIC)_subIC.shutdown();
    }
    sprintf(command, 
            "rosrun dynamic_reconfigure dynparam set %s DISPLAY %d", 
            "/vision_module/image_capture", val);
    system(command);
}

void CVisualizer::showImage_PD(const bool &val){
    char command[256];
    if(val){
        if(!_subPD)_subPD = _nh.subscribe(VISION_PLANE, 1, 
                &CVisualizer::cb_PD, this);
    }else{
        if(_subPD)_subPD.shutdown();
    }
    sprintf(command, 
            "rosrun dynamic_reconfigure dynparam set %s DISPLAY %d", 
            "/vision_module/plane_detection", val);
    system(command);
}

void CVisualizer::showImage_OD(const bool &val){
    char command[256];
    if(val){
        if(!_subOD)_subOD = _nh.subscribe(OBJECT_DETECTION, 1, 
                &CVisualizer::cb_OD, this);
    }else{
        if(_subOD)_subOD.shutdown();
    }
    sprintf(command, 
            "rosrun dynamic_reconfigure dynparam set %s DISPLAY %d", 
            "/vision_module/object_detection", val);
    system(command);
}

void CVisualizer::showImage_OR(const bool &val){
    char command[256];
    if(val){
        if(!_subOR)_subOR = _nh.subscribe(OBJECT_RECOGNITION, 1, 
                &CVisualizer::cb_OR, this);
    }else{
        if(_subOR)_subOR.shutdown();
    }
    sprintf(command, 
            "rosrun dynamic_reconfigure dynparam set %s DISPLAY %d", 
            "/vision_module/object_recognition", val);
    system(command);
}

void CVisualizer::showImage_DARKNET(const bool &val){
    char command[256];
    if(val){
        if(!_subDARKNET)_subDARKNET = _nh.subscribe(OBJECT_DETECTION, 1, 
                &CVisualizer::cb_DARKNET, this);
    }else{
        if(_subDARKNET)_subDARKNET.shutdown();
    }
    sprintf(command, 
            "rosrun dynamic_reconfigure dynparam set %s DISPLAY %d", 
            "/vision_module/darknet", val);
    system(command);
}

void CVisualizer::showImage_FD(const bool &val){
    char command[256];
    if(val){
        if(!_subFD)_subFD = _nh.subscribe(FACE_DETECTION, 1, 
                &CVisualizer::cb_FD, this);
    }else{
        if(_subFD)_subFD.shutdown();
    }
    sprintf(command, 
            "rosrun dynamic_reconfigure dynparam set %s DISPLAY %d", 
            "/vision_module/face_detection", val);
    system(command);
}

void CVisualizer::showImage_FR(const bool &val){
    char command[256];
    if(val){
        if(!_subFR)_subFR = _nh.subscribe(FACE_RECOGNITION, 1, 
                &CVisualizer::cb_FR, this);
    }else{
        if(_subFR)_subFR.shutdown();
    }
    sprintf(command, 
            "rosrun dynamic_reconfigure dynparam set %s DISPLAY %d", 
            "/vision_module/face_recognition", val);
    system(command);
}

