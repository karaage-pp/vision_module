
#include <stdio.h>
#include <math.h>
#include <vision_module/common/VectorCalculus.h>

struct CvPoint3D32f operator-(const CvPoint3D32f &p1, const CvPoint3D32f &p2){
	CvPoint3D32f p;
	p.x = p1.x - p2.x;
	p.y = p1.y - p2.y;
	p.z = p1.z - p2.z;
	return p;
}

struct CvPoint3D32f operator/(const CvPoint3D32f &p1, const float &n){
	CvPoint3D32f p;
	p.x = p1.x / n;
	p.y = p1.y / n;
	p.z = p1.z / n;
	return p;
}

struct CvPoint3D32f operator*(const CvPoint3D32f &p1, const CvPoint3D32f &p2){
	CvPoint3D32f p;
	p.x = (p1.y * p2.z) - (p2.y * p1.z);
	p.y = (p1.z * p2.x) - (p2.z * p1.x);
	p.z = (p1.x * p2.y) - (p2.x * p1.y);
	return p;
}

struct CvPoint3D32f operator*(const CvPoint3D32f &p1, const float &n){
	CvPoint3D32f p;
	p.x = p1.x * n;
	p.y = p1.y * n;
	p.z = p1.z * n;
	return p;
}

CvPoint3D32f getVector(const CvPoint3D32f &p1, const CvPoint3D32f &p2){
	return p2 - p1;
}

CvPoint3D32f getNormal(const CvPoint3D32f &v1, const CvPoint3D32f &v2){
	CvPoint3D32f v;
	v = v1 * v2;
	return v / getNorm(v);
}

bool normalize(CvPoint3D32f &vec){
	vec = vec / getNorm(vec);
	if(getNorm(vec) < 1.1)return true;
	else return false;
}

float getDistToPlane(const CvPoint3D32f &p1, const CvPoint3D32f &pOnPlane, 
					 const CvPoint3D32f &normal){
	float value = getDotProduct(pOnPlane - p1, normal);
	if(value >= 0.0)return value;
	else return -value;
}

float getNorm(const CvPoint3D32f &p){
	return sqrt((p.x * p.x) + (p.y * p.y) + (p.z * p.z));
}

float getDotProduct(const CvPoint3D32f &p1, const CvPoint3D32f &p2){
	return (p1.x * p2.x) + (p1.y * p2.y) + (p1.z * p2.z);
}

void printVector(const CvPoint3D32f &v){
	printf("VECTOR : x : %4.8f\n", v.x);
	printf("VECTOR : y : %4.8f\n", v.y);
	printf("VECTOR : z : %4.8f\n", v.z);
}
