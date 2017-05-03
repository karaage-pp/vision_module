
#include <opencv2/opencv.hpp>

//オブジェクトの最小面積
#define MIN_OBJECT_AREA  2000//100 
//オブジェクトの最小幅、最小高さ
#define MIN_OBJECT_WIDTH 0.04 //0.02
#define MIN_OBJECT_HEIGHT 0.04 //0.02

#define MIN_COUNT 50

namespace OD{
struct Point{
	int x;
	int y;
	CvPoint3D32f p3D;
};

struct ObjectInfo{
	CvPoint3D32f center;
	CvPoint3D32f normal;
	Point top;
	Point bottom;
	Point left;
	Point right;
    Point front;
    Point rear;
    int numPoint;
};

bool closer(ObjectInfo rLeft, ObjectInfo rRight);

}

class CObjectDetection{
public:
	CObjectDetection(int w, int h);
	~CObjectDetection();

protected:
	int m_width;
	int m_height;
	int m_reso;
	cv::Mat m_objectMat;
	std::vector<OD::ObjectInfo> m_objectInfo;
    std::vector<std::vector<int> > m_objPointCloud;

public:
	void filter(int *sIndx, CvPoint3D32f *pXYZ, float maxDist);
	std::vector<OD::ObjectInfo> detectObject(int *sIndx, 
											 CvPoint3D32f *pXYZ, 
											 unsigned int minObjectArea = MIN_OBJECT_AREA);

	std::vector<OD::ObjectInfo> getObjectInfo(){return m_objectInfo;}
	cv::Mat getObjectMat(){return m_objectMat;}
	cv::Mat getObjectImage(cv::Mat rawColorMat, 
						   int &ulx, int &uly, int &lrx, int &lry);
    std::vector<int> getObjectPointCloud(const unsigned int &number);
    bool isOnThisPlane(const CvPoint3D32f &center, 
                        const CvPoint3D32f &normal, 
                        const unsigned int &objNum, 
                        const CvPoint3D32f *pXYZ, 
                        const float maxDist = 0.1);
};
