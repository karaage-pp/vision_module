
/**********************************************************
PlaneDetection2.h		Copyright 2015.10.01 by N.Ikeda

平面検出プログラム

**********************************************************/

#include <opencv2/opencv.hpp>

class CPlaneDetection2{
public:
	struct planeInfo{
		CvPoint3D32f normal;
		CvPoint3D32f center;
		CvPoint3D32f upperMax;
		CvPoint3D32f lowerMax;
		CvPoint3D32f upperMin;
		CvPoint3D32f lowerMin;
	};

	struct color{
		uchar b;
		uchar g;
		uchar r;
	};

    struct CvPoint3D32f operator=(const CvPoint3D32f &p){
        CvPoint3D32f res;
        res.x = p.x;
        res.y = p.y;
        res.z = p.z;
        return res;
    }

public:
	CPlaneDetection2();
	~CPlaneDetection2();

protected:
	int m_tWidth;
	int m_tHeight;

	int m_numPlane;
	int *m_sIndx;
	std::vector<planeInfo> m_planeInfo;
	cv::Mat m_planeMat;
	cv::Mat m_filterMat;

    //configurable parameters
	int m_width;
	int m_height;
	int m_reso;
    
    float m_errorRange;
    int m_minSegmentSize;
    int m_searchIteration;
    float m_maxDist;
    float m_minDist;
    float m_maxDistGap;


	bool isSearched(const int &x, const int &y);
	bool isSearched(const int &num);
	bool isPlaneMember(const CvPoint3D32f &p, const CvPoint3D32f &pOnPlane, 
					  const CvPoint3D32f &normal);
	bool firstSearch(const int &x, const int &y, CvPoint3D32f *pXYZ, 
					 CvPoint3D32f &pOnPlane, CvPoint3D32f &normal);

	bool secondSearch(const int &x, const int &y, 
					  CvPoint3D32f *pXYZ, CvPoint3D32f &pOnPlane, 
					  CvPoint3D32f &normal);

	bool thirdSearch(CvPoint3D32f *XYZ, CvPoint3D32f &pOnPlane, 
					 CvPoint3D32f &normal, planeInfo &result);

	bool calcNormal(CvPoint3D32f *pXYZ, CvPoint3D32f &pOnPlane, 
					CvPoint3D32f &normal, int *pList, const int &pNum);

	//m_planeMatに各平面を描画するための関数
	bool setColor(const int &num, int n = 0);
	color getColor(int n);

	void filter(CvPoint3D32f *pXYZ, float maxDist);
	void distLaplacianFilter(CvPoint3D32f *pXYZ, float distThrd);

public:
	void init(int width, int height, float maxDist);
    void paramConfig(float errorRange, int minSegmentSize, 
                        int searchIteration, 
                        float maxDist, float minDist, float maxDistGap);
	int detectPlane(CvPoint3D32f *pXYZ, float maxDist);
	int isObjectOnAPlane(CvPoint3D32f bottom, CvPoint3D32f top, 
						 CvPoint3D32f &normal);
	std::vector<planeInfo> getPlaneInfo(){return m_planeInfo;}
	int* getSegmentIndx(){return m_sIndx;}
	cv::Mat getPlaneMat(){return m_planeMat;}
	cv::Mat getFilterMat(){return m_filterMat;}

	//プログラム確認用関数群
	cv::Mat getTriangleView(int x, int y);
	cv::Mat getRandomTriangleView();
};
