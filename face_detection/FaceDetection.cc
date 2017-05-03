
#include <stdio.h>
#include <vision_module/face_detection/FaceDetection.h>

CFaceDetection::CFaceDetection(){
}

CFaceDetection::~CFaceDetection(){
}

bool CFaceDetection::setModel(const char *modelFileName){
    return m_classifier.load(modelFileName);
}

void CFaceDetection::detectFace(const cv::Mat &mat){
    cv::Mat gray;
    cv::cvtColor(mat, gray, CV_RGB2GRAY);
    m_classifier.detectMultiScale(
        gray, m_faces, 1.1, 2, CV_HAAR_SCALE_IMAGE, cv::Size(30, 30));
}

void CFaceDetection::drawFaces(cv::Mat &mat, const cv::Rect &rect){
    cv::rectangle(mat, rect, cv::Scalar(255, 0, 0), 1);
}

const std::vector<cv::Rect> CFaceDetection::getFaces() const {
    return m_faces;
}
