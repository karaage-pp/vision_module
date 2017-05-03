
/**********************************************************
VectorCalculus.			Copyright 2015.10.01 by N.Ikeda

ベクトル演算プログラム

**********************************************************/

#include <opencv2/opencv.hpp>

//from p1 to p2
CvPoint3D32f getVector(const CvPoint3D32f &p1, const CvPoint3D32f &p2);
//normalized v1 * v2
CvPoint3D32f getNormal(const CvPoint3D32f &p1, const CvPoint3D32f &p2);
bool normalize(CvPoint3D32f &vec);
float getDistToPlane(const CvPoint3D32f &p1, const CvPoint3D32f &pOnPlane, 
					 const CvPoint3D32f &normal);
float getNorm(const CvPoint3D32f &p);
float getDotProduct(const CvPoint3D32f &p1, const CvPoint3D32f &p2);

void printVector(const CvPoint3D32f &v);
