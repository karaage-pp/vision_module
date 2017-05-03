
#include <vision_module/object_recognition/Alexnet.h>

CAlexnet::CAlexnet(const char *pretrained,
                const char *pythonFile,
                const char *workspace,
                const char *labelFile){

    Py_Initialize();

    PyRun_SimpleString("import sys");
    PyRun_SimpleString("import os");
    PyRun_SimpleString("sys.path.append(os.getcwd())");
    if(workspace){
        char command[128];
        sprintf(command, "sys.path.append(\"%s\")",
                workspace);
        PyRun_SimpleString(command);
    }

    PyRun_SimpleString("sys.argv = [\"foo.bar\"]");

    PyObject *oModName = PyString_FromString(pythonFile);
    if(oModName == NULL)printf("pystring error\n");
    m_oModule = PyImport_Import(oModName);
    if(m_oModule == NULL){
        printf("ERROR: import error\n");
        PyErr_Print();
    }

    Py_DECREF(oModName);

    m_oClassify = PyObject_GetAttrString(m_oModule, "classify");
    if(m_oClassify == NULL)printf("ERROR: getAttrString classify\n");

    m_oExtractFeatures = PyObject_GetAttrString(m_oModule,
                                                "extractFeatures");
    if(m_oExtractFeatures == NULL){
        printf("ERROR: getAttrString extractFeatures\n");
        PyErr_Print();
    }

    setupClassifier(pretrained);

    //追加　ここから
    FILE *fp = fopen(labelFile, "r");
    if(fp == NULL){
        printf("AlexNet.cc:: no such file %s\n", labelFile);
    }else{
        char label[64];
        while(fscanf(fp, "%s\n", label) != EOF){
            m_labelLists.push_back(label);
        }
        fclose(fp);
    }
    //ここまで
}

CAlexnet::~CAlexnet(){
    Py_XDECREF(m_oModule);
    Py_XDECREF(m_oClassify);
    Py_XDECREF(m_oExtractFeatures);
    Py_Finalize();
}

PyObject *CAlexnet::Array2PyList(unsigned char *buff, int size){
    PyObject *list = PyList_New(0);
    for(int i = 0; i < size; i++){
        PyList_Append(list, PyInt_FromLong(buff[i]));
    }
    return list;
}

PyObject *CAlexnet::callFunc(PyObject *func, const cv::Mat &roiMat){
    PyObject *oImage = Array2PyList(roiMat.data,
                                    roiMat.cols * roiMat.rows * 3);
    PyObject *oWidth = PyInt_FromLong(roiMat.cols);
    PyObject *oHeight = PyInt_FromLong(roiMat.rows);

    PyObject *res = PyObject_CallFunctionObjArgs(
            func, oImage, oWidth, oHeight, NULL);
    Py_XDECREF(oImage);
    Py_XDECREF(oWidth);
    Py_XDECREF(oHeight);
    return res;
}

int CAlexnet::setupClassifier(const char *pretrained){

    PyObject *oPretrained = PyString_FromString(pretrained);

    PyObject *oSetup = PyObject_GetAttrString(
                                m_oModule, "setup");
    if(oSetup == NULL)printf("ERROR: getAttrString setup\n");

    PyObject_CallFunctionObjArgs(
        oSetup, oPretrained, NULL);

    Py_XDECREF(oPretrained);
    Py_XDECREF(oSetup);
    return 0;
}

std::vector<float> CAlexnet::classify(const cv::Mat &roiMat){
    PyObject *oResult = callFunc(m_oClassify, roiMat);
    int size = PySequence_Fast_GET_SIZE(oResult);
    double v;

    std::vector<float> probs;
    probs.clear();
    for(int i = 0; i < size; i++){
        v = PyFloat_AsDouble(PyList_GET_ITEM(oResult, i));
        probs.push_back((float)v);
    }

    Py_DECREF(oResult);
    m_probs = probs;
    return probs;
}

std::vector<float> CAlexnet::extractFeatures(const cv::Mat &roiMat){
    PyObject *oResult = callFunc(m_oExtractFeatures, roiMat);
    int size = PySequence_Fast_GET_SIZE(oResult);
    double v;

    std::vector<float> features;
    features.clear();
    for(int i = 0; i < size; i++){
        v = PyFloat_AsDouble(PyList_GET_ITEM(oResult, i));
        features.push_back((float)v);
    }
    Py_DECREF(oResult);
    return features;
}

//追加　ここから
std::string CAlexnet::getLabelName(const int &rank){
    if(m_probs.size() != 0 && m_labelLists.size() != 0);
    else return "NONE";

    std::vector<float> buff = m_probs;
    std::vector<float>::iterator iter;
    size_t index = 0;
    for(int i = 0; i < rank; i++){
        iter = std::max_element(buff.begin(), buff.end());
        index = std::distance(buff.begin(), iter);
        buff[index] = 0.0;
    }
    return m_labelLists[index];
}
//ここまで

//追加　ここから
int CAlexnet::getId(const int &rank){
    if(m_probs.size() != 0 && m_labelLists.size() != 0);
    else return 0;

    std::vector<float> buff = m_probs;
    std::vector<float>::iterator iter;
    size_t index = 0;
    for(int i = 0; i < rank; i++){
        iter = std::max_element(buff.begin(), buff.end());
        index = std::distance(buff.begin(), iter);
        buff[index] = 0.0;
    }
    return index;
}
//ここまで

//追加　ここから
float CAlexnet::getProb(const int &rank){
    if(m_probs.size() != 0 && m_labelLists.size() != 0);
    else return 0.0;

    std::vector<float> buff = m_probs;
    std::vector<float>::iterator iter;
    size_t index = 0;
    for(int i = 0; i < rank; i++){
        iter = std::max_element(buff.begin(), buff.end());
        index = std::distance(buff.begin(), iter);
        buff[index] = 0.0;
    }
    return m_probs[index];
}
//ここまで

//追加　ここから
result_cell CAlexnet::getResult(const int &rank){
    if(m_probs.size() != 0 && m_labelLists.size() != 0);
    else{
        result_cell result;
        result.name = "NONE";
        result.id = 0;
        result.score = 0.0;
        return result;
    }

    std::vector<float> buff = m_probs;
    std::vector<float>::iterator iter;
    size_t index = 0;
    for(int i = 0; i < rank; i++){
        iter = std::max_element(buff.begin(), buff.end());
        index = std::distance(buff.begin(), iter);
        buff[index] = 0.0;
    }
    result_cell result;
    result.name = m_labelLists[index];
    result.id = index;
    result.score = m_probs[index];
    return result;
}
//ここまで

