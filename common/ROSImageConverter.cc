// Begin ROSImageConverter.cpp

#include <stdio.h>
#include <limits>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <vision_module/common/ROSImageConverter.h>

#define RGBD_POS_B 16
#define RGBD_POS_X 0
#define RGBD_POINT_STEP 32
#define RGBD_LOOP_ADDEND_FOR_RGB 29
#define RGBD_LOOP_ADDEND_FOR_D 6

bool convertToMat(
            const sensor_msgs::ImageConstPtr &src, 
            cv::Mat &dst){

    try{
        dst = cv_bridge::toCvCopy(
            src, sensor_msgs::image_encodings::BGR8)->image.clone();
    }
    catch(cv_bridge::Exception &e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return false;
    }

    return true;
}

bool convertToMat(
            const sensor_msgs::PointCloud2ConstPtr &src, 
            cv::Mat &dst){

    if(src->width == dst.cols 
            && src->height == dst.rows
            && dst.type() == CV_8UC3
            && src->point_step == RGBD_POINT_STEP);
    else return false;
    
    unsigned int end = src->width * src->height * 3;
    const uint8_t *buf = src->data.data();
    buf += RGBD_POS_B;
    for(unsigned int i = 0; i < end; i+=3){
        for(unsigned int j = 0; j < 3; j++){
            dst.data[i + j] = *buf;
            buf++;
        }
        buf += RGBD_LOOP_ADDEND_FOR_RGB;
    }
    
    return true;
}

bool convertToMat(
            const CvPoint3D32f *src, 
            cv::Mat &dst, 
            const unsigned int &size, 
            const float &maxDist){

    if(dst.cols * dst.rows == size && dst.type() == CV_8UC1);
    else return false;

    float max = 0.0, min = 1000000, devider;
    for(unsigned int i = 0; i < size; i++){
        if(!isnan(src[i].z));
        else continue;
        if(max < src[i].z)max = src[i].z;
        else if(min > src[i].z)min = src[i].z;
    }
    
    devider = max - min;
    if(devider <= 0.0)return false;
    for(unsigned int i = 0; i < size; i++){
        dst.data[i] = 255 - (unsigned char)(((src->z -min) / devider) * 255);
        src++;
    }
    return true;
}

bool convertToMat(
            const vision_module::ImageInfoConstPtr &src, 
            cv::Mat &dst){

    dst = cv::Mat::zeros(src->height, src->width, CV_8UC3);

    unsigned char *matBuf = dst.data;
    const uint8_t *msgBuf = src->bgr.data();
    int it = dst.cols * dst.rows * 3;
    while(it--){
        *matBuf = (unsigned char)(*msgBuf);
        matBuf++;
        msgBuf++;
    }
    return true;
}

bool convertToMat(
            const vision_module::ObjectInfoConstPtr &src, 
            cv::Mat &dst){

    dst = cv::Mat::zeros(src->height, src->width, CV_8UC3);

    unsigned char *matBuf = dst.data;
    const uint8_t *msgBuf = src->bgr.data();
    int it = dst.cols * dst.rows * 3;
    while(it--){
        *matBuf = (unsigned char)(*msgBuf);
        matBuf++;
        msgBuf++;
    }
    return true;
}

bool convertToImgMsg(
            const cv::Mat &src, 
            sensor_msgs::ImageConstPtr &dst){
    
    dst = cv_bridge::CvImage(
                std_msgs::Header(), 
                sensor_msgs::image_encodings::BGR8, 
                src).toImageMsg();

    return true;
}

bool convertToCvPC(
                const sensor_msgs::PointCloud2ConstPtr &src, 
                CvPoint3D32f * dst, 
                const unsigned int &size){

    if(src->width * src->height == size);
    else return false;

    const float *buf = reinterpret_cast<const float*>(src->data.data());
    unsigned int it = size;
    while(it--){
        if(!isnan(*buf)){
            dst->x = *buf;
            dst->y = *(++buf);
            dst->z = *(++buf);
        }else{
            dst->x = 0.0;
            dst->y = 0.0;
            dst->z = 0.0;
            buf+=2;
        }
        dst++;
        buf += RGBD_LOOP_ADDEND_FOR_D;
    }

    return true;
}

bool convertToCvPC(
                const vision_module::ImageInfoConstPtr &src, 
                CvPoint3D32f *dst, 
                const unsigned int &size){

    if(size == src->width * src->height);
    else return false;

    const float *buf = src->points.data();
    int it = src->width * src->height;
    while(it--){
        dst->x = *(buf++);
        dst->y = *(buf++);
        dst->z = *(buf++);
        dst++;
    }
    return true;
}               

bool convertToCvPC(
                const vision_module::ObjectInfoConstPtr &src, 
                CvPoint3D32f *dst, 
                const unsigned int &size){

    if(size == src->width * src->height);
    else return false;

    const float *buf = src->points.data();
    int it = src->width * src->height;
    while(it--){
        dst->x = *(buf++);
        dst->y = *(buf++);
        dst->z = *(buf++);
        dst++;
    }
    return true;
}               

bool convertToBgr(
                const cv::Mat &src, 
                vision_module::ImageInfo &dst){

    if(src.type() == CV_8UC3);
    else return false;
    dst.width = src.cols;
    dst.height = src.rows;

    const unsigned char *matBuf = src.data;
    int sizeInBytes = dst.width * dst.height * 3;
    dst.bgr.assign(matBuf, matBuf + sizeInBytes); 
    return true;
}

bool convertToBgr(
                const sensor_msgs::PointCloud2ConstPtr &src, 
                vision_module::ImageInfo &dst){
    
    unsigned int end = 0;
    const uint8_t *buf = src->data.data();

    dst.width = src->width;
    dst.height = src->height;
    end = dst.width * dst.height * 3;

    dst.bgr.clear();
    buf += RGBD_POS_B;
    for(unsigned int i = 0; i < end; i+=3){
        for(unsigned int j = 0; j < 3; j++){
            dst.bgr.push_back(*buf);
            buf++;
        }
        buf += RGBD_LOOP_ADDEND_FOR_RGB;
    }
    return true;
}

bool convertToPoints(
                const CvPoint3D32f *src, 
                vision_module::ImageInfo &dst, 
                const unsigned int &size){

    dst.points.clear();
    const float *buf = reinterpret_cast<const float*>(src);
    dst.points.assign(buf, buf + (size * 3));
    return true;
}

bool convertToPoints(
                const sensor_msgs::PointCloud2ConstPtr &src, 
                vision_module::ImageInfo &dst){
    
    const float *buf 
                = reinterpret_cast<const float*>(src->data.data());
    int size;
    
    dst.width = src->width;
    dst.height = src->height;
    size = dst.width * dst.height;

    dst.points.clear();

    while(size--){
        if(!isnan(*buf)){
            dst.points.push_back(*buf);
            dst.points.push_back(*(++buf));
            dst.points.push_back(*(++buf));
        }else{
            dst.points.push_back(0.0);
            dst.points.push_back(0.0);
            dst.points.push_back(0.0);
            buf += 2;
        }
        buf += RGBD_LOOP_ADDEND_FOR_D;
    }
    return true;
}

bool copyTo(
        const vision_module::ImageInfoConstPtr &src, 
        vision_module::ImageInfo &dst){

    dst.width = src->width;
    dst.height = src->height;
    dst.bgr = src->bgr;
    dst.points = src->points;
    dst.label = src->label;

    return true;
}

bool copyTo(
        const vision_module::ImageInfoConstPtr &src, 
        vision_module::ObjectInfo &dst){

    dst.width = src->width;
    dst.height = src->height;
    dst.bgr = src->bgr;
    dst.points = src->points;
    dst.label = src->label;

    return true;
}

bool copyTo(
        const vision_module::ObjectInfoConstPtr &src, 
        vision_module::ObjectInfo &dst){

    /*const int size = src->object.size();
    for(int i = 0; i < size; i++){
        vision_module::ObjectData data;
        vision_module::ObjectData *buf = &(src->object[i]);
        data.width = buf->width;
        data.height = buf->height;
        data.camera.x = buf->camera.x;
        data.camera.y = buf->camera.y;
        data.camera.z = buf->camera.z;
        data.normal.x = buf->narmal.x;
        data.normal.y = buf->normal.y;
        data.normal.z = buf->normal.z;
        data.szwht.x = buf->szwht.x;
        data.szwht.y = buf->szwht.y;
        data.szwht.z = buf->szwht.z;
        data.
    }*/
    dst.objects = src->objects;
    return true;
}

bool copyTo(const vision_module::FaceInfoConstPtr &src, 
            vision_module::FaceInfo &dst){

    dst.faces = src->faces;
    return true;
}

bool setLabel(
        int *src, vision_module::ImageInfo &dst, const unsigned int &size){
    dst.label.clear();
    dst.label.assign(src, src + size);
    return true;
}

bool setLabel(
        int *src, vision_module::ObjectInfo &dst, const unsigned int &size){
    dst.label.clear();
    dst.label.assign(src, src + size);
    return true;
}

bool setLabel(
        const vision_module::ImageInfoConstPtr &src, 
        int *dst, unsigned int size){

    const int *srcBuf = src->label.data();
    
    if(src->label.size() == size);
    else return false;

    while(size--){
        *dst = *(srcBuf++);
        dst++;
    }
    return true;
}

bool setLabel(
        const vision_module::ObjectInfoConstPtr &src, 
        int *dst, unsigned int size){

    const int *srcBuf = src->label.data();
    
    if(src->label.size() == size);
    else return false;

    while(size--){
        *dst = *(srcBuf++);
        dst++;
    }
    return true;
}

bool setToVector(const cv::Mat &src, std::vector<uint8_t> &dst){
    const int size = src.cols * src.rows * 3;

    if(src.type() == CV_8UC3);
    else return false;

    dst.clear();
    dst.assign(src.data, src.data + size);

    return true;
}

bool setToUINT8(const std::vector<uint8_t> &src, unsigned char *dst, 
        unsigned int sizeInBytes){

    const uint8_t *srcBuf = src.data();
    while(sizeInBytes--){
        *dst = (unsigned char)(*srcBuf);
        srcBuf++;
        dst++;
    }
    return true;
}
