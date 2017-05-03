
#include "darknet/network.h"
#include "darknet/region_layer.h"
#include "darknet/cost_layer.h"
#include "darknet/utils.h"
#include "darknet/parser.h"
#include "darknet/box.h"
#include "darknet/demo.h"
#include "darknet/option_list.h"
#include "opencv2/highgui/highgui.hpp"
#include <vector>
#include <string>

struct bbox{
    int left;
    int right;
    int top;
    int bottom;
};

class CDetector{
public:
    CDetector(char* datacfg, char* cfgfile, 
            char* weightfile, float thresh);
    ~CDetector();

protected:
    char** _n;
    network _net;
    layer _output;
    float _thresh;

    int _numobj;
    std::vector<float> _probs;
    std::vector<cv::Mat> _rois;
    std::vector<std::string> _names;
    std::vector<struct bbox> _bboxes;

public:
    void detect(const cv::Mat& input);
    std::vector<float> getProbs(){return _probs;}
    std::vector<cv::Mat> getRois(){return _rois;}
    std::vector<std::string> getNames(){return _names;}
    std::vector<struct bbox> getBBoxes(){return _bboxes;}
};

