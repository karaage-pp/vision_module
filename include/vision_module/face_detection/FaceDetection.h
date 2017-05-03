
#ifndef _FACE_DETECTION
#define _FACE_DETECTION

#include <opencv2/opencv.hpp>
#include <vector>

#define CASCADE_FILE ""

class CFaceDetection{
public:
    CFaceDetection();
    ~CFaceDetection();

protected:
    cv::CascadeClassifier m_classifier;
    std::vector<cv::Rect> m_faces;

public:
    bool setModel(const char *modelFileName);
    void detectFace(const cv::Mat &mat);
    void drawFaces(cv::Mat &mat, const cv::Rect &rect);
    const std::vector<cv::Rect> getFaces() const ;
};

#endif
