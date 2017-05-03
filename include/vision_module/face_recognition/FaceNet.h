#ifndef _FACENET
#define _FACENET

#include <Python.h>
#include <opencv2/opencv.hpp>
#include <vector>

#define PY_FILE "interface_face"

class CFaceNet{
public:
    CFaceNet(const char *faceModel, 
            const char *PyFile = PY_FILE, 
            const char *workspace = NULL);
    ~CFaceNet();

protected:
    PyObject *m_oModule;
    PyObject *m_oWho;
    PyObject *m_oHow;
    PyObject *m_oWhich;
    PyObject *m_oExtractFeatures;
    std::vector<std::string> m_ageList;
    std::vector<std::string> m_genderList;

    int sort(const std::vector<float> &input, 
            std::vector<int> &label, 
            std::vector<float> &data, 
            int num = 3);
    std::vector<float> PyList2Vector(PyObject *oList);
    PyObject *Array2PyList(unsigned char *buff, int size);
    PyObject *callFunc(PyObject *func, const cv::Mat &roiMat);

    int loadTextToVector(const char *fileName, 
            std::vector<std::string> &vect);

public:
    int setAgeList(const char *fileName);
    int setGenderList(const char *fileName);
    int setupClassifier(PyObject *oModule, const char *model);
    int who(const cv::Mat &roiMat, 
            std::vector<int> &ctgr, 
            std::vector<float> &prob, 
            int numCand = 2);
    int how(const cv::Mat &roiMat, 
            std::vector<std::string> &ctgr, 
            std::vector<float> &prob, 
            int numCand = 2);
    int which(const cv::Mat &roiMat, 
            std::vector<std::string> &ctgr, 
            std::vector<float> &prob, 
            int numCand = 1);
    std::vector<float> extractFeatures(const cv::Mat &roiMat);
};

#endif
