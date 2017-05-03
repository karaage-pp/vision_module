
#include <string.h>
#include <vision_module/common/TrainSVM.h>
#include <vision_module/common/MySys.h>

#define MAX_WORD_COUNT 131072
#define EXTRACTED_FILES "extractedFiles.txt"

CTrainSVM::CTrainSVM(){
    m_model = NULL;
    m_numData = 0;
}

CTrainSVM::~CTrainSVM(){
    delSVM();
}

bool CTrainSVM::checkType(const char *fileName, const char *type){
    char t[32] = "", other[64] = "";
    sscanf(fileName, "%[^:]:%[^ \n]", t, other);
    if(strcmp(t, type) == 0)return true;
    else return false;
}

int CTrainSVM::getLabel(const char *fileName){
    char type[32] = "", label[32] = "", num[32] = "";
    sscanf(fileName, "%[^:]:%[^:]:%[^ \n]", type, label, num);
    return atoi(label);
}

int CTrainSVM::setSVM(const char *model, const int &nodeSize){
    m_model = svm_load_model(model);
    m_numOfEntries = svm_get_nr_class(m_model);
    return 0;
}

int CTrainSVM::changeSVM(const char *model, const int &nodeSize){
    delSVM();
    return setSVM(model, nodeSize);
}

int CTrainSVM::delSVM(){
    svm_free_and_destroy_model(&m_model);
    return 0;
}

int CTrainSVM::classify(const std::vector<float> &features, 
                        std::vector<scoreIndex> &result){
    
    if(m_model);
    else return -1;

    int size = features.size();
    svm_node *nodes = new svm_node[size];
    double *probs = new double[m_numOfEntries];
    
    result.clear();
    for(int i = 0; i < size; i++){
        nodes[i].index = i + 1;
        nodes[i].value = features[i];
    }
    
    svm_predict_probability(m_model, nodes, probs);
    for(int i = 0; i < m_numOfEntries; i++){
        result.push_back(scoreIndex(i, probs[i]));
    }
    std::sort(result.begin(), result.end(), std::greater<scoreIndex>());

    delete[] nodes;
    delete[] probs;
    return 0;
}

int CTrainSVM::trainSVM(const char *histsFileName, 
                        const char *modelName){

    char command[128] = "";
    sprintf(command, "./svm-train -s 0 -t 0 -b 1 %s %s", 
                        histsFileName, modelName);
    return myExec(command);
}

int CTrainSVM::recogSVM(const char *histsFileName, 
                        const char *modelName, 
                        const char *dstDir){

    char command[128] = "";
    sprintf(command, 
            "./svm-predict -b 1 %s %s %s/result.txt > %s/resultAcc.txt", 
                        histsFileName, 
                        modelName, 
                        dstDir, 
                        dstDir);
    return myExec(command);
}

int CTrainSVM::makeHists(const char *featuresFileName, 
                            const char *labelsFileName, 
                            const char *histsFileName){

    FILE *fpFeat = fopen(featuresFileName, "r");
    FILE *fpLabel = fopen(labelsFileName, "r");
    FILE *fpHist = fopen(histsFileName, "w");
    char buf[MAX_WORD_COUNT] = "";
    char label[8] = "";

    if(fpFeat && fpLabel && fpHist);
    else{
        int status = 0;
        if(fpFeat){
            status += 1;
            fclose(fpFeat);
        }
        if(fpLabel){
            status += 2;
            fclose(fpLabel);
        }
        if(fpHist){
            status += 4;
            fclose(fpHist);
        }
        printf("TrainSVM: ERROR %d LINE, status %d", __LINE__, status);
    }

    while(fgets(buf, MAX_WORD_COUNT - 1, fpFeat)){
        if(fgets(label, 7, fpLabel));
        else break;

        sscanf(label, "%[^ \n]", label);
        fprintf(fpHist, "%s\t", label);
        char *tp = strtok(buf, "\n\t, ");
        int num = 1;
        
        fprintf(fpHist, "%d:%s\t", num++, tp);
        while(tp = strtok(NULL, "\n\t, ")){
            fprintf(fpHist, "%d:%s\t", num++, tp);
        }
        fprintf(fpHist, "\n");
    }
    fclose(fpFeat);
    fclose(fpLabel);
    fclose(fpHist);
    return 0;
}

int CTrainSVM::makeDataSet(const char *srcDir, 
                                const char *dstFileName, 
                                const char *dstLabelFile){

    char buf[MAX_WORD_COUNT] = "";
    std::string extractedFile = srcDir;
    std::vector<std::string> files = myDIRENT(srcDir);
    FILE *fp = NULL;
    FILE *fpExtracted = NULL;
    FILE *fpLabel = NULL;
   
    extractedFile += "/";
    extractedFile += EXTRACTED_FILES;
    
    if(files.size());
    else{
        printf("TrainSVM: '%s' no such directory\n", srcDir);
        return -1;
    }

    if(std::find(files.begin(), files.end(), std::string(EXTRACTED_FILES)) 
            != files.end()){
        std::vector<std::string>::iterator it 
                = std::find(files.begin(), files.end(), 
                            std::string(EXTRACTED_FILES));

        files.erase(it);
        fpExtracted = fopen(extractedFile.c_str(), "a+");
        if(fpExtracted);
        else{
            printf("TrainSVM: can't open '%s'\n", extractedFile.c_str());
            return -1;
        }

        while(fgets(buf, 255, fpExtracted)){
            if(buf[0] != '\n');
            else continue;
            sscanf(buf, "%[^ \n]", buf);
            std::vector<std::string>::iterator it 
                = std::find(files.begin(), files.end(), std::string(buf));
            if(it != files.end())files.erase(it);
            printf("'%s' is already extracted\n", buf);
        }
        fp = fopen(dstFileName, "a");
        fpLabel = fopen(dstLabelFile, "a");
    }else{
        fp = fopen(dstFileName, "w");
        fpExtracted = fopen(extractedFile.c_str(), "w");
        fpLabel = fopen(dstLabelFile, "w");
    }

    if(fp && fpExtracted && fpLabel);
    else{
        int status = 0;
        if(fp){
            status += 1;
            fclose(fp);
        }
        if(fpExtracted){
            status += 2;
            fclose(fpExtracted);
        }
        if(fpLabel){
            status += 4;
            fclose(fpLabel);
        }
        printf("TrainSVM: can't open file, status %d\n", status);
        return -1;
    }
    for(std::vector<std::string>::iterator it = files.begin();
            it != files.end(); it++){
        
        FILE *fpFeat = NULL;
        char filePath[256] = "";
        int label = -1;

        if(checkType(it->c_str(), FEAT_PREFIX));
        else continue;
        
        sprintf(filePath, "%s/%s", srcDir, it->c_str());
        fpFeat = fopen(filePath, "r");
        if(fpFeat);
        else{
            printf("TrainSVM: can't open '%s'\n", filePath);
            continue;
        }

        label = getLabel(it->c_str());
        while(fgets(buf, MAX_WORD_COUNT - 1, fpFeat)){
            if(buf[0] != '\n');
            else continue;
            fputs(buf, fp);
            fprintf(fpLabel, "%d\n", label);
        }
        
        printf("FILE: '%s' EXTRACTED\n", filePath);
        fprintf(fpExtracted, "%s\n", it->c_str());
        fclose(fpFeat);
    }
    fclose(fp);
    fclose(fpExtracted);
    fclose(fpLabel);
    return 0;
}

bool CTrainSVM::roiBuffering(const cv::Mat &roi, const unsigned int &label, 
                                const char *dstDir, const int &numPict){
    
    char name[128] = "";

    sprintf(name, "%s/%s:%03d:%04d.jpg", 
                dstDir, PICT_PREFIX, label, m_numData);
    cv::imwrite(name, roi);
    m_numData++;

    if(numPict <= m_numData){
        m_numData = 0;
        return true;
    }else return false;
}

bool CTrainSVM::learn(const char *pictDir, 
                        const char *featureList, 
                        const char *dstDir){
    std::vector<std::string> entries;
    entries = myDIRENT(pictDir);

    return false;
}

bool CTrainSVM::distanceFilter(const CvPoint3D32f &pos){
    float buf = 0.0;
    buf = pos.x - m_beforePos.x;
    if(buf < DIST_ERROR_RANGE && buf > -DIST_ERROR_RANGE);
    else return false;
    buf = pos.y - m_beforePos.y;
    if(buf < DIST_ERROR_RANGE && buf > -DIST_ERROR_RANGE);
    else return false;
    buf = pos.z - m_beforePos.z;
    if(buf < DIST_ERROR_RANGE && buf > -DIST_ERROR_RANGE);
    else return false;

    m_beforePos.x = pos.x;
    m_beforePos.y = pos.y;
    m_beforePos.z = pos.z;
    return true;
}

int CTrainSVM::getNumDataCollected(){
    return m_numData;
}

