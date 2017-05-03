
#include <vision_module/face_recognition/FaceNet.h>

struct list{
    int label;
    float data;

    bool operator<(const list &right) const {
        return data == right.data ? label > right.label : data > right.data;
    }
};

CFaceNet::CFaceNet(const char *faceModel,
            const char *PyFile,
            const char *workspace){

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

    PyObject *oModName = PyString_FromString(PyFile);
    if(oModName == NULL)printf("pystring error\n");
    m_oModule = PyImport_Import(oModName);
    if(m_oModule == NULL){
        printf("ERROR: import error, %s\n", PyFile);
        PyErr_Print();
    }

    Py_DECREF(oModName);

    m_oWho = PyObject_GetAttrString(m_oModule, "who");
    if(m_oWho == NULL)printf("ERROR: getAttrString who\n");

    m_oHow = PyObject_GetAttrString(m_oModule, "how");
    if(m_oHow == NULL)printf("ERROR: getAttrString how\n");

    m_oWhich = PyObject_GetAttrString(m_oModule, "which");
    if(m_oWhich == NULL)printf("ERROR: getAttrString which\n");

    m_oExtractFeatures = PyObject_GetAttrString(m_oModule,
                                                "extractFeatures");
    if(m_oExtractFeatures == NULL){
        printf("ERROR: getAttrString extractFeatures\n");
        PyErr_Print();
    }

    setupClassifier(m_oModule, faceModel);
}

CFaceNet::~CFaceNet(){
    Py_XDECREF(m_oModule);
    Py_XDECREF(m_oWho);
    Py_XDECREF(m_oHow);
    Py_XDECREF(m_oWhich);
    Py_XDECREF(m_oExtractFeatures);
    Py_Finalize();
}

int CFaceNet::sort(const std::vector<float> &input,
            std::vector<int> &label,
            std::vector<float> &data,
            int num){

    std::vector<list> buff;
    list v;
    int size = input.size();

    buff.clear();
    label.clear();
    data.clear();
    for(int i = 0; i < size; i++){
        v.label = i;
        v.data = input[i];
        buff.push_back(v);
    }
    std::sort(buff.begin(), buff.end());
    std::vector<list>::iterator end = buff.end();
    int i = 0;
    for(std::vector<list>::iterator it = buff.begin();
            it != end; it++){
        label.push_back(it->label);
        data.push_back(it->data);
        i++;
        if(i >= num)break;
    }
    return 0;
}

std::vector<float> CFaceNet::PyList2Vector(PyObject *oList){
    std::vector<float> buff;
    int size = PySequence_Fast_GET_SIZE(oList);
    double v;

    buff.clear();
    for(int i = 0; i < size; i++){
        v = PyFloat_AsDouble(PyList_GET_ITEM(oList, i));
        buff.push_back((float)v);
    }
    return buff;
}

PyObject *CFaceNet::Array2PyList(unsigned char *buff, int size){
    PyObject *list = PyList_New(0);
    for(int i = 0; i < size; i++){
        PyList_Append(list, PyInt_FromLong(buff[i]));
    }
    return list;
}

PyObject *CFaceNet::callFunc(PyObject *func, const cv::Mat &roiMat){
    PyObject *oImage = Array2PyList(roiMat.data,
                                    roiMat.cols * roiMat.rows * 3);
    PyObject *oWidth = PyInt_FromLong(roiMat.cols);
    PyObject *oHeight = PyInt_FromLong(roiMat.rows);
    printf("sss3");
    PyObject *res = PyObject_CallFunctionObjArgs(
            func, oImage, oWidth, oHeight, NULL);
    printf("sss4");
    Py_DECREF(oImage);
    Py_DECREF(oWidth);
    Py_DECREF(oHeight);

    return res;
}

int CFaceNet::loadTextToVector(const char *fileName,
            std::vector<std::string> &vect){
    FILE *fp;
    char buf[32] = {0};

    vect.clear();
    if((fp = fopen(fileName, "r")) != NULL){
        while(fgets(buf, 32, fp) != NULL){
            char *p = NULL;
            p = strchr(buf, '\n');
            if(p != NULL){
                *p = '\0';
            }
            vect.push_back(buf);
        }
    }else{
        printf("No such file: %s\n", fileName);
        return -1;
    }
    return 0;
}

int CFaceNet::setAgeList(const char *fileName){
    return loadTextToVector(fileName, m_ageList);
}

int CFaceNet::setGenderList(const char *fileName){
    return loadTextToVector(fileName, m_genderList);
}

int CFaceNet::setupClassifier(PyObject *oModule, const char *model){

    PyObject *oModel = PyString_FromString(model);

    PyObject *oSetup = PyObject_GetAttrString(oModule, "setup");
    if(oSetup == NULL)printf("ERROR: getAttrString setup\n");

    PyObject_CallFunctionObjArgs(
        oSetup, oModel, NULL);

    Py_XDECREF(oModel);
    Py_XDECREF(oSetup);
    return 0;
}

int CFaceNet::who(const cv::Mat &roiMat,
            std::vector<int> &ctgr,
            std::vector<float> &prob,
            int numCand){
    PyObject *oResult = callFunc(m_oWho, roiMat);
    std::vector<float> res = PyList2Vector(oResult);

    ctgr.clear();
    prob.clear();
    sort(res, ctgr, prob, numCand);
    Py_DECREF(oResult);
    return 0;
}

int CFaceNet::how(const cv::Mat &roiMat,
            std::vector<std::string> &ctgr,
            std::vector<float> &prob,
            int numCand){
    PyObject *oResult = callFunc(m_oHow, roiMat);
    std::vector<float> res = PyList2Vector(oResult);
    std::vector<int> label;

    if(numCand > m_ageList.size()){
        printf("ERROR: CFaceNet::how, numCand is too big\n");
        return -1;
    }
    label.clear();
    ctgr.clear();
    prob.clear();
    sort(res, label, prob, numCand);
    for(std::vector<int>::iterator it = label.begin();
            it != label.end(); it++){
        ctgr.push_back(m_ageList[*it]);
    }
    Py_DECREF(oResult);
    return 0;
}

int CFaceNet::which(const cv::Mat &roiMat,
            std::vector<std::string> &ctgr,
            std::vector<float> &prob,
            int numCand){
    PyObject *oResult = callFunc(m_oWhich, roiMat);
    std::vector<float> res = PyList2Vector(oResult);
    std::vector<int> label;

    if(numCand > m_genderList.size()){
        printf("ERROR: CFaceNet::which, numCand is too big\n");
        return -1;
    }
    label.clear();
    ctgr.clear();
    prob.clear();
    sort(res, label, prob, numCand);
    for(std::vector<int>::iterator it = label.begin();
            it != label.end(); it++){
        ctgr.push_back(m_genderList[*it]);
    }
    Py_DECREF(oResult);
    return 0;
}

std::vector<float> CFaceNet::extractFeatures(const cv::Mat &roiMat){
    printf("miya1");
    PyObject *oResult = callFunc(m_oExtractFeatures, roiMat); //この関数の中で落ちてる
    printf("miya2");
    std::vector<float> features = PyList2Vector(oResult);
    printf("miya3");
    Py_DECREF(oResult);
    printf("miya4");
    return features;
}
