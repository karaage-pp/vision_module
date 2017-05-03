// Begin ROSImageConverter.hpp

#ifndef IMAGE_CONVERTER
#define IMAGE_CONVERTER

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <vision_module/ImageInfo.h>
#include <vision_module/ObjectInfo.h>
#include <vision_module/FaceInfo.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#define D_DEFAULT_MAX_DIST 8.0

extern bool convertToMat(
                const sensor_msgs::ImageConstPtr &src, 
                cv::Mat &dst);
extern bool convertToMat(
                const sensor_msgs::PointCloud2ConstPtr &src, 
                cv::Mat &dst);
extern bool convertToMat(
                const CvPoint3D32f *src, 
                cv::Mat &dst, 
                const unsigned int &size, 
                const float &maxDist = D_DEFAULT_MAX_DIST);
extern bool convertToMat(
                const vision_module::ImageInfoConstPtr &src, 
                cv::Mat &dst);
extern bool convertToMat(
                const vision_module::ObjectInfoConstPtr &src, 
                cv::Mat &dst);
extern bool convertToImgMsg(
                const cv::Mat &src, 
                sensor_msgs::ImageConstPtr &dst);
extern bool convertToCvPC(
                const sensor_msgs::PointCloud2ConstPtr &src, 
                CvPoint3D32f *dst, 
                const unsigned int &size);
extern bool convertToCvPC(
                const vision_module::ImageInfoConstPtr &src, 
                CvPoint3D32f *dst, 
                const unsigned int &size);
extern bool convertToCvPC(
                const vision_module::ObjectInfoConstPtr &src, 
                CvPoint3D32f *dst, 
                const unsigned int &size);
extern bool convertToBgr(
                const cv::Mat &src, 
                vision_module::ImageInfo &dst);
extern bool convertToBgr(
                const sensor_msgs::PointCloud2ConstPtr &src, 
                vision_module::ImageInfo &dst);
extern bool convertToPoints(
                const CvPoint3D32f *src, 
                vision_module::ImageInfo &dst, 
                const unsigned int &size);
extern bool convertToPoints(
                const sensor_msgs::PointCloud2ConstPtr &src, 
                vision_module::ImageInfo &dst);


extern bool copyTo(
                const vision_module::ImageInfoConstPtr &src, 
                vision_module::ImageInfo &dst);
extern bool copyTo(
                const vision_module::ImageInfoConstPtr &src, 
                vision_module::ObjectInfo &dst);
extern bool copyTo(
                const vision_module::ObjectInfoConstPtr &src, 
                vision_module::ObjectInfo &dst);
extern bool copyTo(
                const vision_module::FaceInfoConstPtr &src, 
                vision_module::FaceInfo &dst);
extern bool setLabel(
                int *src, 
                vision_module::ImageInfo &dst, 
                const unsigned int &size);
extern bool setLabel(
                int *src, 
                vision_module::ObjectInfo &dst, 
                const unsigned int &size);
extern bool setLabel(
                const vision_module::ImageInfoConstPtr &src, 
                int *dst, 
                unsigned int size);
extern bool setLabel(
                const vision_module::ObjectInfoConstPtr &src, 
                int *dst, 
                unsigned int size);

extern bool setToVector(
                const cv::Mat &src, 
                std::vector<uint8_t> &dst);
extern bool setToUINT8(
                const std::vector<uint8_t> &src, 
                unsigned char *dst, unsigned int sizeInBytes);

#endif

// End ROSImageConverter.hpp
