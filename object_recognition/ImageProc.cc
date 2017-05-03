
#include <vision_module/object_recognition/ImageProc.h>

#define CH_WIDTH 640
#define CH_HEIGHT 480
#define NUM_BASE_COLOR 9
#define NUM_PREFIX 4
#define NUM_REMAIN 4

CImageProc::CImageProc(){
}

CImageProc::~CImageProc(){
}

int CImageProc::calcHistAsHSV(const cv::Mat &mat){
    cv::cvtColor(mat, m_mat, CV_RGB2HSV);
    int size = NUM_BASE_COLOR * NUM_PREFIX + NUM_REMAIN;

    m_hist.clear();
    m_result.clear();
    for(int i = 0; i < size; i++)m_hist.push_back(0);
    
    for(int y = 0; y < m_mat.rows; y++)
        for(int x = 0; x < m_mat.cols; x++){
            cv::Vec3b &vec = m_mat.at<cv::Vec3b>(y, x);
            m_hist[hsvToLabel(vec[0], vec[1], vec[2])]++;
        }
    
    for(int i = 0; i < size; i++){
        m_result.push_back(histLabel(i, m_hist[i]));
    }
    std::sort(m_result.begin(), m_result.end(), 
                std::greater<histLabel>());
    return 0;
}

unsigned int CImageProc::getNBest(const unsigned int &n){
    if(m_result.size() > n);
    else return 0;
    return m_result[n].bin;
}

std::string CImageProc::getNBestName(const unsigned int &n){
    return labelToName(getNBest(n)).c_str();
}

std::string CImageProc::labelToName(const unsigned int &label){
    std::string res = "";
    int l = label;
  
    if(l >= NUM_BASE_COLOR * NUM_PREFIX){
        switch(l){
        case 36:return "Black";break;
        case 37:return "Dark Gray";break;
        case 38:return "Gray";break;
        case 39:return "White";break;
        default:return "Unknown";
        }
    }
    switch(l / NUM_BASE_COLOR){
    case 0:;break;
    case 1:res = "Dark ";break;
    case 2:res = "Somber ";break;
    case 3:res = "Pastel ";break;
    };

    l %= NUM_BASE_COLOR;

    switch(l){
    case 0:res += "Blue";break;
    case 1:res += "Turquoise";break;
    case 2:res += "Green";break;
    case 3:res += "Limegreen";break;
    case 4:res += "Yellow";break;
    case 5:res += "Orange";break;
    case 6:res += "Blown";break;
    case 7:res += "Red";break;
    case 8:res += "Purple";break;
    }
    return res;
}

int CImageProc::hsvToLabel(const uint8_t &h, 
                            const uint8_t &s,
                            const uint8_t &v){

    int res = 0;
    if(v < 42)return 36;
    else if(v < 127)res += 1;
    else if(v < 200)res += 2;
    else res += 3;

    if(s < 10);
    else if(s < 100)res += 3;
    else res += 6;

    switch(res){
    case 1:return 37;break;
    case 2:return 38;break;
    case 3:return 39;break;
    case 4:res = NUM_BASE_COLOR;break;
    case 5:res = NUM_BASE_COLOR * 2;break;
    case 6:res = NUM_BASE_COLOR * 3;break;
    case 7:res = NUM_BASE_COLOR;break;
    case 8:res = 0;break;
    case 9:res = 0;break;
    };

    if(h < 24);
    else if(h < 56)res += 1;
    else if(h < 66)res += 2;
    else if(h < 90)res += 3;
    else if(h < 101)res += 4;
    else if(h < 111)res += 5;
    else if(h < 119)res += 6;
    else if(h < 131)res += 7;
    else if(h < 164)res += 8;

    return res;
}

cv::Mat CImageProc::makeMatFromHSV(const uint8_t &h, 
                                    const uint8_t &s, 
                                    const uint8_t &v){

    cv::Mat mat(cv::Size(CH_WIDTH, CH_HEIGHT), 
                CV_8UC3, cv::Scalar::all(255));

    cv::cvtColor(mat, mat, CV_RGB2HSV);
    int size = CH_HEIGHT * CH_WIDTH;

    for(int y = 0; y < CH_HEIGHT; y++)
        for(int x = 0; x < CH_WIDTH; x++){
            cv::Vec3b &vec = mat.at<cv::Vec3b>(y, x);
            vec[0] = h;
            vec[1] = s;
            vec[2] = v;
        }
    cv::cvtColor(mat, mat, CV_HSV2RGB);
    uint8_t i = v < 130 ? 255 : 0;
    cv::putText(mat, labelToName(hsvToLabel(h, s, v)).c_str(), 
                cv::Point(10, 50), cv::FONT_HERSHEY_COMPLEX_SMALL, 
                1.2, cv::Scalar(i, i, i), 1, CV_AA);
    return mat.clone();
}
