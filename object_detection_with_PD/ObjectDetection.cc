
#include <stdio.h>
#include <vision_module/object_detection_with_PD/ObjectDetection.h>
#include <vision_module/common/Segmentation.h>

//最初のオブジェクト番号
#define S_OBJECT_NUM 101
//物体を探す領域のセグメント番号
#define SEGMENT_NUM -1
//物体切り出しに用いる画像のマージン
#define MARGIN_W 10
#define MARGIN_H 20
#define FILTERED -2

bool OD::closer(ObjectInfo rLeft, ObjectInfo rRight){
	return rLeft.center.z < rRight.center.z;
}

CObjectDetection::CObjectDetection(int w, int h){
	m_width = w;
	m_height = h;
	m_reso = m_width * m_height;
	m_objectMat = cv::Mat::zeros(m_height, m_width, CV_8UC3);
    m_objPointCloud.clear();
}

CObjectDetection::~CObjectDetection(){
}

void CObjectDetection::filter(int *sIndx, CvPoint3D32f *pXYZ, float maxDist){
	for(int it = 0; it < m_reso; it++){
		if(sIndx[it] != SEGMENT_NUM)continue;
		else{
			if(pXYZ[it].z > maxDist || pXYZ[it].z < 0.5)sIndx[it] = FILTERED;
		}
	}
}

std::vector<OD::ObjectInfo> CObjectDetection::detectObject(
                                                int *sIndx, 
											    CvPoint3D32f *pXYZ, 
											    unsigned int minObjectArea){

	int endNum = -1;
	int minx = m_width, miny = m_height, maxx = 0, maxy = 0;
	OD::ObjectInfo buf;
	CvPoint3D32f pos;
    std::vector<int> pointCloud;
	
    m_objectInfo.clear();
    for(int i = 0, s = (int)m_objPointCloud.size(); i < s; i++)
        m_objPointCloud[i].clear();
    m_objPointCloud.clear();
	m_objectMat = cv::Mat::zeros(m_height, m_width, CV_8UC3);

	segmentRegionGrow(m_width, m_height, sIndx, 
                        SEGMENT_NUM, S_OBJECT_NUM, endNum);
	for(int it = S_OBJECT_NUM; it <= endNum; it++){
        buf.numPoint = getSegmentArea(m_reso, it, sIndx);
//        printf("DEBUG: numPoint = %d\n", buf.numPoint);
		if(buf.numPoint > minObjectArea){
            pointCloud.clear();
			int area = getSegmentArea(m_reso, it, sIndx);
			minx = m_width, miny = m_height, maxx = 0, maxy = 0;
			for(int y = 0; y < miny; y++){
				int ybym_width = y * m_width;
				for(int x = 0; x < minx; x++){
					if(sIndx[ybym_width + x] != it)continue;
					else{
						miny = y;
						minx = x;
						maxy = y;
						maxx = x;
						buf.top.x = x;
						buf.top.y = y;
						buf.top.p3D = pXYZ[ybym_width + x];
						buf.left.x = x;
						buf.left.y = y;
						buf.left.p3D = pXYZ[ybym_width + x];
                        buf.front.p3D = pXYZ[ybym_width + x];
                        buf.rear.p3D = pXYZ[ybym_width + x];
						pos.x = pXYZ[ybym_width + x].x;
						pos.y = pXYZ[ybym_width + x].y;
						pos.z = pXYZ[ybym_width + x].z;
                        pointCloud.push_back(ybym_width + x);
					}
				}
				for(int x = maxx + 1; x < m_width; x++){
					if(sIndx[ybym_width + x] != it)continue;
					else{
						maxx = x;
						buf.right.x = x;
						buf.right.y = y;
						buf.right.p3D = pXYZ[ybym_width + x];
                        if(buf.front.p3D.z > buf.right.p3D.z)
                            buf.front.p3D = buf.right.p3D;
                        else if(buf.rear.p3D.z < buf.right.p3D.z)
                            buf.rear.p3D = buf.right.p3D;

						pos.x += pXYZ[ybym_width + x].x;
						pos.y += pXYZ[ybym_width + x].y;
						pos.z += pXYZ[ybym_width + x].z;
                        pointCloud.push_back(ybym_width + x);
					}
				}
			}
			for(int y = miny + 1; y < m_height; y++){
				int ybym_width = y * m_width;
				for(int x = 0; x < minx; x++){
					if(sIndx[ybym_width + x] != it)continue;
					else{
						minx = x;
						maxy = y;
						buf.left.x = x;
						buf.left.y = y;
						buf.left.p3D = pXYZ[ybym_width + x];
						buf.bottom.x = x;
						buf.bottom.y = y;
						buf.bottom.p3D = pXYZ[ybym_width + x];
                        if(buf.front.p3D.z > buf.bottom.p3D.z)
                            buf.front.p3D = buf.bottom.p3D;
                        else if(buf.rear.p3D.z < buf.bottom.p3D.z)
                            buf.rear.p3D = buf.bottom.p3D;
						pos.x += pXYZ[ybym_width + x].x;
						pos.y += pXYZ[ybym_width + x].y;
						pos.z += pXYZ[ybym_width + x].z;
                        pointCloud.push_back(ybym_width + x);
					}
				}
				for(int x = minx; x < maxx; x++){
					if(sIndx[ybym_width + x] != it)continue;
					else{
						maxy = y;
						buf.bottom.x = x;
						buf.bottom.y = y;
						buf.bottom.p3D = pXYZ[ybym_width + x];
                        if(buf.front.p3D.z > buf.bottom.p3D.z)
                            buf.front.p3D = buf.bottom.p3D;
                        else if(buf.rear.p3D.z < buf.bottom.p3D.z)
                            buf.rear.p3D = buf.bottom.p3D;
						pos.x += pXYZ[ybym_width + x].x;
						pos.y += pXYZ[ybym_width + x].y;
						pos.z += pXYZ[ybym_width + x].z;
                        pointCloud.push_back(ybym_width + x);
					}
				}
				for(int x = maxx; x < m_width; x++){
					if(sIndx[ybym_width + x] != it)continue;
					else{
						maxy = y;
						maxx = x;
						buf.right.x = x;
						buf.right.y = y;
						buf.right.p3D = pXYZ[ybym_width + x];
						buf.bottom.x = x;
						buf.bottom.y = y;
						buf.bottom.p3D = pXYZ[ybym_width + x];
                        if(buf.front.p3D.z > buf.bottom.p3D.z)
                            buf.front.p3D = buf.bottom.p3D;
                        else if(buf.rear.p3D.z < buf.bottom.p3D.z)
                            buf.rear.p3D = buf.bottom.p3D;
						pos.x += pXYZ[ybym_width + x].x;
						pos.y += pXYZ[ybym_width + x].y;
						pos.z += pXYZ[ybym_width + x].z;
                        pointCloud.push_back(ybym_width + x);
					}
				}
			}
			buf.center.x = pos.x / (float)area;
			buf.center.y = pos.y / (float)area;
			buf.center.z = pos.z / (float)area;
/*			cv::rectangle(m_objectMat, 
							cv::Point(minx, miny), 
							cv::Point(maxx, maxy), 
							cv::Scalar(0, 0, 200), 3, 4);*/

			// add aoki 2016/2/22 from 
/*			cv::circle(m_objectMat, 
						cv::Point((maxx + minx)/2, (maxy + miny)/2), 
						2, 
						cv::Scalar(200,200,0),
						-1,
						CV_AA);*/
			// here
			
			m_objectInfo.push_back(buf);

//            pointCloud.clear();
//            for(int i = 0; i < m_reso; i++){
//                if(sIndx[i] != it)continue;
//                pointCloud.push_back(i);
//            }
//            printf("ObjectDetecion: point %d\n", 
//                    (int)pointCloud.size());
            m_objPointCloud.push_back(pointCloud);
			/*printf("OBJECT DETECTION : %d, %d, %d, %d\n", 
				minx, miny, maxx, maxy);*/
		}
//        else printf("OTHER: numPoint %d\n", buf.numPoint);
	}
	for(int i = 0; i < m_reso; i++){
		if(sIndx[i] >= S_OBJECT_NUM)m_objectMat.data[i * 3] = 255;
	}
	return m_objectInfo;
}

//自動変数で返しているので注意
cv::Mat CObjectDetection::getObjectImage(cv::Mat rawColorMat, 
										 int &ulx, int &uly, int &lrx, int &lry){

	if(ulx - MARGIN_W >= 0)ulx -= MARGIN_W;
	else ulx = 0;
	if(uly - MARGIN_H >= 0)uly -= MARGIN_H;
	else uly = 0;
	if(lrx + MARGIN_W < rawColorMat.cols)lrx += MARGIN_W;
	else lrx = rawColorMat.cols - 1;
	if(lry + MARGIN_H < rawColorMat.rows)lry += MARGIN_H;
	else lry = rawColorMat.rows - 1;
	cv::Mat objImg(rawColorMat, cv::Rect(ulx, uly, lrx - ulx, lry - uly));
	return objImg;
}

std::vector<int> CObjectDetection::getObjectPointCloud(const unsigned int &number){
    if(m_objPointCloud.size() > number);
    else{
        std::vector<int> dummy;
        return dummy;
    }
    return m_objPointCloud[number];
}

bool CObjectDetection::isOnThisPlane(const CvPoint3D32f &center, 
                                        const CvPoint3D32f &normal, 
                                        const unsigned int &objNum,
                                        const CvPoint3D32f *pXYZ, 
                                        const float maxDist){

    if(objNum < m_objPointCloud.size());
    else return false;

    int count = 0;
    for(std::vector<int>::iterator it = m_objPointCloud[objNum].begin(), 
                                    end = m_objPointCloud[objNum].end();
            it != end; it++){

        pXYZ[*it];
        CvPoint3D32f v;
        float dot = -1.0;

        v.x = pXYZ[*it].x - center.x;
        v.y = pXYZ[*it].y - center.y;
        v.z = pXYZ[*it].z - center.z;
        dot = v.x * normal.x + v.y * normal.y + v.z * normal.z;
        if(dot >= 0.02 && dot < maxDist)count++;
    }
    if(count > MIN_COUNT){
//        printf("true : count = %d/%d, OBJ: %d\n", 
//                    count, (int)m_objPointCloud[objNum].size(), objNum);
        return true;
    }
    else{
//        printf("false: count = %d/%d, OBJ: %d\n", 
//                    count, (int)m_objPointCloud[objNum].size(), objNum);
        return false;
    }
}
