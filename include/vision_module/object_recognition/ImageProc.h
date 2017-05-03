
#ifndef _IMAGEPROC
#define _IMAGEPROC

#include <opencv2/opencv.hpp>

class CImageProc{
public:
    struct histLabel{
        unsigned int bin;
        unsigned int intensity;

        histLabel(const unsigned int &b, const unsigned int &i){
            bin = b;
            intensity = i;
        }

        bool operator>(const histLabel &right) const {
            return intensity == right.intensity ? bin > right.bin
                                            : intensity > right.intensity;
        }
    };

    CImageProc();
    ~CImageProc();

protected:
    std::vector<int> m_hist;
    std::vector<histLabel> m_result;
    cv::Mat m_mat;

public:
    int calcHistAsHSV(const cv::Mat &mat);
    unsigned int getNBest(const unsigned int &n);
    std::string getNBestName(const unsigned int &n);
    std::string labelToName(const unsigned int &label);
    int hsvToLabel(const uint8_t &h, 
                    const uint8_t &s, 
                    const uint8_t &v);
    cv::Mat makeMatFromHSV(const uint8_t &h, 
                            const uint8_t &s, 
                            const uint8_t &v);
};

#endif
