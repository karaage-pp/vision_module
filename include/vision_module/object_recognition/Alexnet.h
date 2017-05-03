
#ifndef _CALEXNET
#define _CALEXNET

#include <Python.h>
#include <opencv2/opencv.hpp>
#include <vector>

#define PYTHON_FILE "interface"
#define MAX_OBJECT_NUM 1000

struct result_cell{
    std::string name;
    int id;
    float score;
};

class CAlexnet{
public:
    CAlexnet(const char *pretrained, 
            const char *pythonFile = PYTHON_FILE, 
            const char *workspace = NULL, 
            const char *labelFile = NULL);
    ~CAlexnet();

protected:
    PyObject *m_oModule;
    PyObject *m_oClassify;
    PyObject *m_oExtractFeatures;

    PyObject *Array2PyList(unsigned char *buff, int size);
    PyObject *callFunc(PyObject *func, const cv::Mat &roiMat);

    //追加　ここから
    std::vector<std::string> m_labelLists;
    std::vector<float> m_probs;
    //ここまで

public:
    int setupClassifier(const char *pretrained);
    std::vector<float> classify(const cv::Mat &roiMat);
    std::vector<float> extractFeatures(const cv::Mat &roiMat);
    //追加　ここから
    std::string getLabelName(const int &rank);
    int getId(const int &rank);
    float getProb(const int &rank);
    result_cell getResult(const int &rank);
    //ここまで
};

#endif

