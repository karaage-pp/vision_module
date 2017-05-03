
#ifndef _TRAINSVM
#define _TRAINSVM

#define PICT_PREFIX "PICT"
#define FEAT_PREFIX "FEAT"
#define DIST_ERROR_RANGE 0.2

#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <vision_module/common/svm.h>

struct scoreIndex{
    int index;
    float score;

    scoreIndex(int i, float s){
        index = i;
        score = s;
    }

    bool operator>(const scoreIndex &right) const {
        return score == right.score ? index > right.index
                                    : score > right.score;
    }
};

class CTrainSVM{
public:

    CTrainSVM();
    ~CTrainSVM();

protected:
    int m_numData;
    CvPoint3D32f m_beforePos;
  
    svm_model *m_model;
    int m_numOfEntries;

    bool checkType(const char *fileName, const char *type);
    int getLabel(const char *fileName);

public:
    int setSVM(const char *model, const int &nodeSize);
    int changeSVM(const char *model, const int &nodeSize);
    int delSVM();
    int classify(const std::vector<float> &features, 
                    std::vector<scoreIndex> &result);
    int trainSVM(const char *histsFileName, 
                    const char *modelName);
    int recogSVM(const char *histsFileName, 
                    const char *modelName, 
                    const char *dstDir);
    int makeHists(const char *featuresFileName, 
                    const char *labelsFileName, 
                    const char *histsFileName);
    int makeDataSet(const char *srcDir, 
                    const char *dstFileName, 
                    const char *dstLabelFile);
    bool roiBuffering(const cv::Mat &roi,
                        const unsigned int &label, 
                        const char *dstDir, 
                        const int &numPict);
    bool learn(const char *pictDir,
                const char *featureList, 
                const char *dstDir);
    bool distanceFilter(const CvPoint3D32f &pos);
    int getNumLabelCollected();
    int getNumDataCollected();
};
#endif
