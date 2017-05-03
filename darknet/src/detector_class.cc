
#include <stdio.h>
#include <stdlib.h>

#include <darknet/parser.h>
#include <darknet/utils.h>
#include <darknet/cuda.h>
#include <darknet/blas.h>
#include <darknet/connected_layer.h>
#include <darknet/convolutional_layer.h>
#include <darknet/detector_class.h>

#ifdef __cplusplus
extern "C" {
#endif
    extern image ipl_to_image(IplImage* src);
#ifdef __cplusplus
}
#endif


CDetector::CDetector(char* datacfg, char* cfgfile, 
            char* weightfile, float thresh){

    list *options = read_data_cfg(datacfg);
    char *name_list = option_find_str(options, "names", "names.list");

    _n = get_labels(name_list);
    _net = parse_network_cfg(cfgfile);
    _thresh = thresh;
    _output = _net.layers[_net.n - 1];

    load_weights(&_net, weightfile);
    set_batch_network(&_net, 1);

    free_list(options);
}

CDetector::~CDetector(){
//    free_ptrs((void **)_n, _output.w*_output.h*_output.n);
}

void CDetector::detect(const cv::Mat& input){
    IplImage* src = new IplImage(input);
    image im = ipl_to_image(src);
    image sized = resize_image(im, _net.w, _net.h);
    float nms=.4;

    box* boxes = static_cast<box*>(
            calloc(_output.w*_output.h*_output.n, sizeof(box)));
    float** probs = static_cast<float**>(
            calloc(_output.w*_output.h*_output.n, sizeof(float *)));
    for(int j = 0; j < _output.w*_output.h*_output.n; ++j)
        probs[j] = static_cast<float*>(
                calloc(_output.classes, sizeof(float *)));

    float* X = sized.data;
    network_predict(_net, X);
    get_region_boxes(_output, 1, 1, _thresh, probs, boxes, 0, 0);
    if(nms)do_nms_sort(
            boxes, probs, _output.w*_output.h*_output.n, _output.classes, nms);

    _names.clear();
    _probs.clear();
    _rois.clear();
    _bboxes.clear();
    int num = _output.w*_output.h*_output.n;
    for(int i = 0; i < num; i++){
        int cls = max_index(probs[i], _output.classes);
        float prob = probs[i][cls];

        if(prob < _thresh)continue;

        box b = boxes[i];
        int left  = (b.x-b.w/2.)*im.w;
        int right = (b.x+b.w/2.)*im.w;
        int top   = (b.y-b.h/2.)*im.h;
        int bot   = (b.y+b.h/2.)*im.h;
        
        if(left < 0)left = 0;
        if(right > im.w - 1)right = im.w - 1;
        if(top < 0)top = 0;
        if(bot > im.h - 1)bot = im.h - 1;

        cv::Mat roi = cv::Mat(input, cv::Rect(left, top, 
                    right - left, bot - top)).clone();
        struct bbox bb = {left, right, top, bot};

        _rois.push_back(roi);
        _probs.push_back(prob);
        _names.push_back(_n[cls]);
        _bboxes.push_back(bb);
    }

    delete src;
    free_image(im);
    free_image(sized);
    free(boxes);
    free_ptrs((void **)probs, _output.w*_output.h*_output.n);
    return;
}

