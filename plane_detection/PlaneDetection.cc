
#include <stdio.h>
#include <stdlib.h>
#include <omp.h>

#include <vision_module/plane_detection/PlaneDetection.h>
#include <vision_module/common/VectorCalculus.h>
#include <vision_module/common/Segmentation.h>

#define ERROR_RANGE 0.012
#define MIN_SEGMENT_SIZE 8000
#define IT	200
#define MAX_DIST 8.0
#define MIN_DIST 0.3
#define DEFAULT_INDX_NUM -1
#define MAX_DIST_GAP 0.1
#define SHOW_ALL_PLANAR_POINT
#define FILTERED -2

#define D	5

#define SEARCH_RECT_W 20
#define SEARCH_RECT_H 20

#define MIN_POINT_NUM 2

#define MIN_HEIGHT 0.04

#define COLOR_INTENSITY 50

struct Vertex{
	int x;
	int y;
};

float myAbs(const float &num){
    if(num >= 0.0)return num;
    else return -num;
}

CPlaneDetection2::CPlaneDetection2(){
	m_sIndx = NULL;
    paramConfig(ERROR_RANGE, MIN_SEGMENT_SIZE, IT, MAX_DIST, MIN_DIST,
                MAX_DIST_GAP);
}

CPlaneDetection2::~CPlaneDetection2(){
	if(m_sIndx)delete[] m_sIndx;
}

void CPlaneDetection2::init(int width, int height, float maxDist){
	//printf("%d                %d\n",width,height);
	m_width = width;
	m_height = height;
	m_maxDist = maxDist;
	m_reso = width * height;
	m_planeMat = cv::Mat::zeros(height, width, CV_8UC3);
	m_filterMat = cv::Mat::zeros(height, width, CV_8UC3);

	m_tWidth = width / D;
	m_tHeight = height / D;

	m_sIndx = new int[m_reso];
	numset(m_sIndx, DEFAULT_INDX_NUM, m_reso);
}

void CPlaneDetection2::paramConfig(float errorRange, int minSegmentSize,
                                    int searchIteration, float maxDist,
                                    float minDist, float maxDistGap){

    m_errorRange = errorRange;
    m_minSegmentSize = minSegmentSize;
    m_searchIteration = searchIteration;
    m_maxDist = maxDist;
    m_minDist = minDist;
    m_maxDistGap = maxDistGap;
}

bool CPlaneDetection2::isSearched(const int &x, const int &y){
	if(m_sIndx[y * m_width + x] != DEFAULT_INDX_NUM)return true;
	else return false;
}

bool CPlaneDetection2::isSearched(const int &num){
	if(m_sIndx[num] != DEFAULT_INDX_NUM)return true;
	else return false;
}

bool CPlaneDetection2::isPlaneMember(const CvPoint3D32f &p, const CvPoint3D32f &pOnPlane,
									const CvPoint3D32f &normal){
	if(getDistToPlane(p, pOnPlane, normal) < m_errorRange)return true;
	else return false;
}

bool CPlaneDetection2::firstSearch(const int &x, const int &y, CvPoint3D32f *pXYZ,
								   CvPoint3D32f &pOnPlane, CvPoint3D32f &normal){
	printf("ISSEARCH  DEF: %d   VAL: %d\n",DEFAULT_INDX_NUM,m_sIndx[y*m_width + x]);
	if(isSearched(x, y)){
		printf("F1\n");
		return false;
	}
	if(isSearched(x + m_tWidth, y)){
		printf("F2\n");
		return false;
	}
	if(isSearched(x, y + m_tHeight)){
		printf("F3\n");
		return false;
	}
	if(isSearched(x + m_tWidth, y + m_tHeight)){
		printf("F4\n");
		return false;
	}
	CvPoint3D32f vertex[2], vec[2];

	pOnPlane = pXYZ[y * m_width + x];
	vertex[0] = pXYZ[(y + m_tHeight) * m_width + x];
	vertex[1] = pXYZ[(y + m_tHeight) * m_width + x + m_tWidth];

	vec[0] = getVector(pOnPlane, vertex[0]);
	vec[1] = getVector(pOnPlane, vertex[1]);
	normal = getNormal(vec[0], vec[1]);

	pOnPlane.x = (vertex[0].x + vertex[1].x + pOnPlane.x) / 3.0;
	pOnPlane.y = (vertex[0].y + vertex[1].y + pOnPlane.y) / 3.0;
	pOnPlane.z = (vertex[0].z + vertex[1].z + pOnPlane.z) / 3.0;

	return isPlaneMember(pXYZ[y * m_width + x + m_tWidth], pOnPlane, normal);
}

bool CPlaneDetection2::secondSearch(const int &x, const int &y,
									CvPoint3D32f *pXYZ, CvPoint3D32f &pOnPlane,
									CvPoint3D32f &normal){

	//変数の宣言、初期化
	int *pList = new int[m_reso];
	numset(pList, -1, 512 * 424);
	int pNum = 0;
	int minPointNum = -1;

	//探索領域の規定、最小点数の規定
	Vertex outerUL, outerLR, innerUL, innerLR;
	if((x - SEARCH_RECT_W) >= 0){
		outerUL.x = x - SEARCH_RECT_W;
		if(x + m_tWidth + SEARCH_RECT_W < m_width)
			outerLR.x = x + m_tWidth + SEARCH_RECT_W;
		else outerLR.x = m_width -1;
	}else{
		outerUL.x = 0;
		outerLR.x = x + m_tWidth + SEARCH_RECT_W;
	}
	if((y - SEARCH_RECT_H) >= 0){
		outerUL.y = y - SEARCH_RECT_H;
		if(y + m_tHeight + SEARCH_RECT_H < m_height)
			outerLR.y = y + m_tHeight + SEARCH_RECT_H;
		else outerLR.y = m_height -1;
	}else{
		outerUL.y = 0;
		outerLR.y = y + m_tHeight + SEARCH_RECT_H;
	}
	innerUL.x = x;
	innerUL.y = y;
	innerLR.x = x + m_tWidth;
	innerLR.y = y + m_tHeight;
	minPointNum = (((outerLR.x - outerUL.x) * (outerLR.y - outerUL.y))
					- ((innerLR.x - innerUL.x) * (innerLR.y - innerUL.y)))
						/ MIN_POINT_NUM;

	//探索
	int sybym_width;
	for(int sy = outerUL.y; sy < innerUL.y; sy++){
		sybym_width = sy * m_width;
		for(int sx = outerUL.x; sx < outerLR.x; sx++){
			if(isSearched(sx, sy))continue;
			if(isPlaneMember(pXYZ[sybym_width + sx], pOnPlane, normal)){
				pList[pNum] = sybym_width + sx;
				pNum++;
			}
		}
	}
	for(int sy = innerUL.y; sy < innerLR.y; sy++){
		sybym_width = sy * m_width;
		for(int sx = outerUL.x; sx < innerUL.x; sx++){
			if(isSearched(sx, sy))continue;
			if(isPlaneMember(pXYZ[sybym_width + sx], pOnPlane, normal)){
				pList[pNum] = sybym_width + sx;
				pNum++;
			}
		}
		for(int sx = innerLR.x; sx < outerLR.x; sx++){
			if(isSearched(sx, sy))continue;
			if(isPlaneMember(pXYZ[sybym_width + sx], pOnPlane, normal)){
				pList[pNum] = sybym_width + sx;
				pNum++;
			}
		}
	}
	for(int sy = innerLR.y; sy < outerLR.y; sy++){
		sybym_width = sy * m_width;
		for(int sx = outerUL.x; sx < outerLR.x; sx++){
			if(isSearched(sx, sy))continue;
			if(isPlaneMember(pXYZ[sybym_width + sx], pOnPlane, normal)){
				pList[pNum] = sybym_width + sx;
				pNum++;
			}
		}
	}

	//点群の大きさに応じて処理
	if(pNum >= minPointNum){
		//平面の法線、基準点を再計算
		if(calcNormal(pXYZ, pOnPlane, normal, pList, pNum));
		else{
			delete[] pList;
			return false;
		}
		//この部分は精度を高めるために必要、精度を必要としないならコメントアウト
		numset(pList, -1, 512 * 424);
		pNum = 0;
		for(int it = 0; it < m_reso; it++){
			if(isPlaneMember(pXYZ[it], pOnPlane, normal)){
				pList[pNum] = it;
				pNum++;
			}
		}
		if(calcNormal(pXYZ, pOnPlane, normal, pList, pNum));
		else{
			delete[] pList;
			return false;
		}
		//
	}else{
		delete[] pList;
		return false;
	}
	delete[] pList;
	return true;
}

bool CPlaneDetection2::thirdSearch(CvPoint3D32f *pXYZ, CvPoint3D32f &pOnPlane,
								   CvPoint3D32f &normal, planeInfo &result){

	//変数の宣言、初期化
	int firstSegmentNum = m_numPlane + 2, lastSegmentNum = -1;
	bool isPlane = false;
	int *indxBuf = new int[m_reso];

    result.upperMax.y = -100.0;
    result.lowerMax.y = 100.0;
    float minX = 100.0, maxX = -100.0;

    numset(indxBuf, DEFAULT_INDX_NUM, m_reso);

	for(int it = 0; it < m_reso; it++)
		if(m_sIndx[it] != FILTERED
			&& isPlaneMember(pXYZ[it], pOnPlane, normal))indxBuf[it] = 1;

	segmentRegionGrow(m_width, m_height, indxBuf, 1, firstSegmentNum, lastSegmentNum);
	for(int it = firstSegmentNum; it <= lastSegmentNum; it++){
		if(getSegmentArea(m_reso, it, indxBuf) > m_minSegmentSize){
			isPlane = true;
			for(int it2 = 0; it2 < m_reso; it2++){
#ifdef SHOW_ALL_PLANAR_POINT
				if(indxBuf[it2] != it)continue;
#else
				if(indxBuf[it2] != it || isSearched(it2))continue;
#endif
				else{
                    if(result.upperMax.y < pXYZ[it2].y){
                        result.upperMax.y = pXYZ[it2].y;
                        result.upperMax.z = pXYZ[it2].z;
                    }else if(result.lowerMax.y > pXYZ[it2].y){
                        result.lowerMax.y = pXYZ[it2].y;
                        result.lowerMax.z = pXYZ[it2].z;
                    }
                    if(maxX < pXYZ[it2].x){
                        maxX = pXYZ[it2].x;
                    }else if(minX > pXYZ[it2].x){
                        minX = pXYZ[it2].x;
                    }
					m_sIndx[it2] = m_numPlane;
					setColor(it2);
				}
			}
		}
	}

	delete[] indxBuf;
	if(isPlane){
		result.upperMax.x = maxX;
        result.lowerMax.x = maxX;
        result.upperMin.x = minX;
        result.upperMin.y = result.upperMax.y;
        result.upperMin.z = result.upperMax.z;
        result.lowerMin.x = minX;
        result.lowerMin.y = result.lowerMax.y;
        result.lowerMin.z = result.lowerMax.z;
/*        printf("\
UL: %.2f, %.2f, %.2f\n\
UR: %.2f, %.2f, %.2f\n\
LL: %.2f, %.2f, %.2f\n\
LR: %.2f, %.2f, %.2f\n",
result.upperMin.x,
result.upperMin.y,
result.upperMin.z,
result.upperMax.x,
result.upperMax.y,
result.upperMax.z,
result.lowerMin.x,
result.lowerMin.y,
result.lowerMin.z,
result.lowerMax.x,
result.lowerMax.y,
result.lowerMax.z);*/
        m_numPlane++;
		return true;
	}else return false;
}

bool CPlaneDetection2::calcNormal(CvPoint3D32f *pXYZ, CvPoint3D32f &pOnPlane,
								  CvPoint3D32f &normal,
								  int *pList, const int &pNum){

	//変数の宣言、初期化
	float I[9] = {0.0, 0.0, 0.0, 0.0, 0.0,
				  0.0, 0.0, 0.0};
	float p[3];

	//各変数の値を計算
	for(int it = 0; it < pNum; it++){
		p[0] = pXYZ[pList[it]].x;
		p[1] = pXYZ[pList[it]].y;
		p[2] = pXYZ[pList[it]].z;

		I[0] += p[0] * p[0];
		I[1] += p[0] * p[1];
		I[2] += p[0];
		I[3] += p[1] * p[1];
		I[4] += p[1];
		I[5] += p[2] * p[0];
		I[6] += p[1] * p[2];
		I[7] += p[2];
	}
	//平面上の点座標の平均を計算、これを基準点として扱う
	pOnPlane.x = I[2] / (float)pNum;
	pOnPlane.y = I[4] / (float)pNum;
	pOnPlane.z = I[7] / (float)pNum;

	//最小二乗法で平面の法線ベクトルを計算
	I[8] = (I[0] * (I[3] * (float)pNum - I[4] * I[4]))
			+ (I[1] * (I[4] * I[2] * 2.0 - I[1] * pNum)
			- (I[3] * I[2] * I[2]));

	normal.x = (I[5] * (I[3] * (float)pNum - I[4] * I[4])
				+ I[1] * (I[4] * I[7] - I[6] * (float)pNum)
				+ I[2] * (I[4] * I[6] - I[3] * I[7])) / I[8];

	normal.y = (I[0] * (I[6] * (float)pNum - I[4] * I[7])
				+ I[5] * (I[4] * I[2] - I[1] * (float)pNum)
				+ I[2] * (I[7] * I[1] - I[6] * I[2])) / I[8];

	normal.z = -1.0;//平面のベクトルがカメラの方を向くようにしている

	return normalize(normal);
}

bool CPlaneDetection2::setColor(const int &num, int n){
	color c = getColor(n);
	m_planeMat.data[num * 3] = c.b;
	m_planeMat.data[(num * 3) + 1] = c.g;
	m_planeMat.data[(num * 3) + 2] = c.r;
	return true;
}

CPlaneDetection2::color CPlaneDetection2::getColor(int n){
	color res;

	if(n == 0){
		switch(m_numPlane % 3){
		case 0:res.b = 1;res.g = 0;res.r = 0;break;
		case 1:res.b = 0;res.g = 1;res.r = 0;break;
		case 2:res.b = 0;res.g = 0;res.r = 1;break;
		}
		switch(m_numPlane / 4){
		case 0:break;
		case 1:res.b++;break;
		case 2:res.g++;break;
		case 3:res.r++;break;
		default:res.b++;res.g++;res.r++;break;
		}
		res.b = res.b * COLOR_INTENSITY + 50;
		res.g = res.g * COLOR_INTENSITY + 50;
		res.r = res.r * COLOR_INTENSITY + 50;
	}else if(n == DEFAULT_INDX_NUM){
		res.b = 0;
		res.g = 0;
		res.r = 0;
	}else if(n == FILTERED){
		res.b = 100;
		res.g = 100;
		res.r = 100;
	}else{
		if(n < 0)printf("ERROR\n");
		switch(n % 3){
		case 0:res.b = 1;res.g = 0;res.r = 0;break;
		case 1:res.b = 0;res.g = 1;res.r = 0;break;
		case 2:res.b = 0;res.g = 0;res.r = 1;break;
		}
		switch(n / 4){
		case 0:break;
		case 1:res.b++;break;
		case 2:res.g++;break;
		case 3:res.r++;break;
		default:res.b++;res.g++;res.r++;break;
		}
		res.b = res.b * COLOR_INTENSITY + 50;
		res.g = res.g * COLOR_INTENSITY + 50;
		res.r = res.r * COLOR_INTENSITY + 50;
	}
	return res;
}

void CPlaneDetection2::filter(CvPoint3D32f *pXYZ, float maxDist){
	for(int it = 0; it < m_reso; it++){
		if(m_sIndx[it] != DEFAULT_INDX_NUM)continue;
		else{
			if(pXYZ[it].z > maxDist || pXYZ[it].z < m_minDist){
                m_sIndx[it] = FILTERED;
                m_filterMat.data[it * 3] = 255;
            }
		}
	}
}

void CPlaneDetection2::distLaplacianFilter(CvPoint3D32f *pXYZ, float distThrd){
	int m_heightm1 = m_height - 1;
	int m_widthm1 = m_width -1;
	int ybyw, ybywpx;
	for(int y = 1; y < m_heightm1; y++){
		ybyw = y * m_width;
		for(int x = 1; x < m_widthm1; x++){
			ybywpx = ybyw + x;
			if(myAbs(pXYZ[ybywpx - 1].z - pXYZ[ybywpx + 1].z) > distThrd
				|| myAbs(pXYZ[ybywpx - m_width].z - pXYZ[ybywpx + m_width].z) > distThrd){

                //m_sIndx[ybywpx - 1] = FILTERED;
                //m_sIndx[ybywpx + 1] = FILTERED;
				//m_sIndx[ybywpx - m_width] = FILTERED;
				//m_sIndx[ybywpx + m_width] = FILTERED;
				m_sIndx[ybywpx] = FILTERED;
                m_filterMat.data[ybywpx * 3 + 1] = 255;
			}
		}
	}
}

int CPlaneDetection2::detectPlane(CvPoint3D32f *pXYZ, float maxDist){
	if(m_sIndx);
	else{
		printf("PLANE DETECTION : クラスを初期化してください\n");
		return -1;
	}

	//各変数の宣言、初期化
	int x = 252, y = 212;
	int rdx = m_width - m_tWidth, rdy = m_height - m_tHeight;
	CvPoint3D32f pOnPlane, normal;
    planeInfo info;
	m_planeMat = cv::Mat::zeros(m_height, m_width, CV_8UC3);;
	m_filterMat = cv::Mat::zeros(m_height, m_width, CV_8UC3);
	filter(pXYZ, maxDist);
	m_planeInfo.clear();
	distLaplacianFilter(pXYZ, m_maxDistGap);
	m_numPlane = 0;

	for(int x=rand()%10;x<rdx;x+=100){
		for(int y =rand()%10;y<rdy;y+=100){
			//printf("x: %d  y: %d\n",x,y);
			if(firstSearch(x, y, pXYZ, pOnPlane, normal));
			else{
				continue;
			}
			if(secondSearch(x, y, pXYZ, pOnPlane, normal));
			else{
				continue;
			}
			if(thirdSearch(pXYZ, pOnPlane, normal, info)){
				info.normal = normal;
				info.center = pOnPlane;
				m_planeInfo.push_back(info);
			}else{
					continue;
				}
		}
	}

	//printf("@@@@@@@@@@@ END @@@@@@@@@@\n");
	return 0;
}

int CPlaneDetection2::isObjectOnAPlane(CvPoint3D32f bottom,
									   CvPoint3D32f top,
									   CvPoint3D32f &normal){
	planeInfo pInfo;
	for(int i = 0; i < m_numPlane; i++){
		pInfo = m_planeInfo[i];
		if((myAbs(getDotProduct(getVector(pInfo.center, bottom), pInfo.normal)) < 0.05)
			&& (getDotProduct(getVector(bottom, top), pInfo.normal) > MIN_HEIGHT)){

			normal = m_planeInfo[i].normal;
			return i;
		}
	}
	return -1;
}

cv::Mat CPlaneDetection2::getTriangleView(int x = 252, int y = 212){
	m_planeMat = cv::Mat::zeros(424, 512, CV_8UC3);

	cv::line(m_planeMat,
		cv::Point(x, y),
		cv::Point(x + m_tWidth, y + m_tHeight), cv::Scalar(0, 200, 0), 1, 4);
	cv::line(m_planeMat,
		cv::Point(x, y),
		cv::Point(x, y + m_tHeight), cv::Scalar(0, 200, 0), 1, 4);
	cv::line(m_planeMat,
		cv::Point(x, y + m_tHeight),
		cv::Point(x + m_tWidth, y + m_tHeight), cv::Scalar(0, 200, 0), 1, 4);

	return m_planeMat;
}

cv::Mat CPlaneDetection2::getRandomTriangleView(){
	int x = 252, y = 212;
	int rdx = m_width - m_tWidth, rdy = m_height - m_tHeight;
	x = rand() % rdx;
	y = rand() % rdy;

	m_planeMat = cv::Mat::zeros(m_height, m_width, CV_8UC3);
	cv::line(m_planeMat,
		cv::Point(x, y),
		cv::Point(x + m_tWidth, y + m_tHeight), cv::Scalar(255, 255, 255), 1, 4);
	cv::line(m_planeMat,
		cv::Point(x, y),
		cv::Point(x, y + m_tHeight), cv::Scalar(255, 255, 255), 1, 4);
	cv::line(m_planeMat,
		cv::Point(x, y + m_tHeight),
		cv::Point(x + m_tWidth, y + m_tHeight), cv::Scalar(255, 255, 255), 1, 4);

	return m_planeMat;
}
