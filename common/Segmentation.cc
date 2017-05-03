
#include <stdio.h>
#include <vector>
#include <string.h>
#include <limits.h>
#include <vision_module/common/Segmentation.h>

//池田法における定数
#define CODE_X 1250249349
#define CODE_Y 2824120404
#define DEFAULT 2

struct numPair{
	int num1;
	int num2;
};

void printSegment(const int &width, const int &height, int *map){
	if((width < 20) && (width > 0) && (height < 20) && (height > 0));
	else{
		printf("SEGMENTATION : ERROR\n");
	}
	printf("SEGMENTATION : DEBUG\n");
	for(int y = 0; y < height; y++){
		for(int x = 0; x < width; x++){
			printf("%2d ", map[y * width + x]);
		}
		printf("\n");
	}
}

/*周辺４近傍（左上、上、右上、左）のセグメントでstartNum以上の値で最小のものを出力
周辺４近傍でセグメントがなかった場合-1を出力
*/
int checkNeighbor(const int &width, const int &startNum, int &conflict, 
				  const int &x, const int &y, int *map){

	int res = INT_MAX;
	int segNum = -1;
	int count = 0;
	int num[2] = {-1, -1};
	conflict = -1;

	if(y >= 1){
		int ym1bywidth = width * (y - 1);
		if(x >= 1){
			segNum = map[ym1bywidth + x - 1];
			if(segNum >= startNum){
				num[0] = segNum;
				count++;
			}
			segNum = map[ym1bywidth + x];
			if(segNum >= startNum){
				if(count){
					if(num[0] != segNum){
						num[1] = segNum;
						count++;
					}
				}else{
					num[0] = segNum;
					count++;
				}
			}
			if((x + 1) < width){
				segNum = map[ym1bywidth + x + 1];
				if(segNum >= startNum){
					if(count){
						if(num[0] != segNum){
							num[1] = segNum;
							count++;
						}
					}else{
						num[0] = segNum;
						count++;
					}
				}
			}
			segNum = map[y * width + x - 1];
			if(segNum >= startNum){
				if(count){
					if(num[0] != segNum){
						num[1] = segNum;
						count++;
					}
				}else{
					num[0] = segNum;
					count++;
				}
			}
		}else{
			segNum = map[ym1bywidth + x];
			if(segNum >= startNum){
				if(count){
					if(num[0] != segNum){
						num[1] = segNum;
						count++;
					}
				}else{
					num[0] = segNum;
					count++;
				}
			}
			segNum = map[ym1bywidth + x + 1];
			if(segNum >= startNum){
				if(count){
					if(num[0] != segNum){
						num[1] = segNum;
						count++;
					}
				}else{
					num[0] = segNum;
					count++;
				}
			}
		}
	}else{
		if(x >= 1){
			segNum = map[(y * width) + x - 1];
			if(segNum >= startNum)return segNum;
			else return -1;
		}else return -1;
	}

	if(count == 0)return -1;
	else if(count == 1)return num[0];
	else{
		if(num[0] < num[1]){
			conflict = num[1];
			return num[0];
		}else{
			conflict = num[0];
			return num[1];
		}
	}
}

void segmentate(const int &width, const int &height, 
				int *map, const int &num, const int &startNum, int &endNum){

	//各種変数の宣言、初期化
	int reso = width * height;
	int segNum = -1;
	int conflict = -1;
	int count = 0;
	numPair *numIndx = new numPair[reso / 4];
	endNum = startNum;

	//セグメンテーション
	for(int y = 0; y < height; y++){
		int ybywidth = y * width;
		for(int x = 0; x < width; x++){
			if(map[ybywidth + x] == num){
				segNum = checkNeighbor(width, startNum, conflict, x, y, map);
				if(conflict < 0);
				else{
					numIndx[count].num1 = conflict;
					numIndx[count].num2 = segNum;
					count++;
				}
				if(segNum > 0)map[ybywidth + x] = segNum;
				else{
					map[ybywidth + x] = endNum;
					endNum++;
				}
			}
		}
	}
	while(--count > 0)
		for(int it = 0; it < reso; it++)
			if(map[it] == numIndx[count].num1)map[it] = numIndx[count].num2;
	
	endNum = startNum;
	for(int it = 0, cSegNum = startNum; it < reso; it++){
		if(map[it] > cSegNum){
			cSegNum = map[it];
			endNum++;
			for(int it2 = it; it2 < reso; it2++){
				if(map[it2] == cSegNum)map[it2] = endNum;
			}
		}
	}

	delete[] numIndx;
}

int sn(const int &width, const int &height, int *map, const int &n1, const int &n2, 
	   int &x, int &y, unsigned char &d){

	unsigned int codeX = CODE_X >> (d << 1);
	unsigned int codeY = CODE_Y >> (d << 1);
	char addend;
	int xpaddend, ypaddend;
	int num = -1;
	for(int it = 0; it < 8; it++){
		//printf("it = %d\ncodeX = %x, codeY = %x\n", it, codeX & 0x03, codeY & 0x03);
		if(codeX & 0x01){
			addend = -1;
		}else if((codeX >> 1) & 0x01){
			addend = 1;
		}else{
			addend = 0;
		}
		xpaddend = x + addend;
		if(xpaddend < width && xpaddend >= 0);
		else{
			codeX >>= 2;
			codeY >>= 2;
			continue;
		}

		if(codeY & 0x01){
			addend = -1;
		}else if((codeY >> 1) & 0x01){
			addend = 1;
		}else{
			addend = 0;
		}
		ypaddend = y + addend;
		if(ypaddend < height && ypaddend >= 0);
		else{
			codeX >>= 2;
			codeY >>= 2;
			continue;
		}
		//printf("x = %d, y = %d\n", xpaddend, ypaddend);
		num = map[ypaddend * width + xpaddend];
		if(num == n1 || num == n2){
			//printf("it = %d\n", it);
			x = xpaddend;
			y = ypaddend;
			d = (d + it + 6) % 8;
			return ypaddend * width + xpaddend;
		}
		codeX >>= 2;
		codeY >>= 2;
	}
	return -1;
}

void wob(const int &width, const int &height, int *map, 
					const int &x, const int &y, unsigned char d, 
					const int &n1, const int &n2){

	int curX = x, curY = y;
	//printf("in %d, %d\n", x, y);
	while(1){
		map[curY * width + curX] = n2;
		//printf("%d, %d, d = %d\n", curX, curY, d);
		//printSegment(width, height, map);
		//int num;
		//scanf("%d", &num);
		sn(width, height, map, n1, n2, curX, curY, d);
		if(curX == x && curY == y)break;
	}
}

void segmentIkedaSearch(const int &width, const int &height, int *map, 
					    const int &num, const int &startNum, int &endNum){

	int ybywidth;
	int segNum;
	endNum = startNum;

	for(int y = 0; y < height; y++){
		ybywidth = y * width;
		for(int x = 0; x < width; x++){
			segNum = map[ybywidth + x];
			if(segNum == num){
				wob(width, height, map, x, y, DEFAULT, num, endNum);
				endNum++;
			}else if(segNum >=  startNum){
				x++;
				for(; x < width; x++){
					if(map[ybywidth + x] == num
						|| map[ybywidth + x] == segNum)map[ybywidth + x] = segNum;
					else break;
				}
			}
		}
	}
	endNum--;
}

void regionGrowing(const int &width, const int &height, 
				int *map, const int &num, const int &x, const int &y, const int &segNum){

	int heightm1, widthm1, ybywidthpx = y * width + x, curX, reso = width * (height - 1);
	std::vector<int> stack;
	stack.push_back(ybywidthpx);
	heightm1 = height - 1;
	widthm1 = width - 1;

	map[ybywidthpx] = segNum;
	while(stack.size()){
		ybywidthpx = stack.back();
		curX = ybywidthpx % width;
		stack.pop_back();
		if(ybywidthpx >= width)
			if(map[ybywidthpx - width] == num){
				map[ybywidthpx - width] = segNum;
				stack.push_back(ybywidthpx - width);
		}
		if(ybywidthpx < reso)
			if(map[ybywidthpx + width] == num){
				map[ybywidthpx + width] = segNum;
				stack.push_back(ybywidthpx + width);
		}
		if(curX > 0)
			if(map[ybywidthpx - 1] == num){
				map[ybywidthpx - 1] = segNum;
				stack.push_back(ybywidthpx - 1);
		}
		if(curX < widthm1)
			if(map[ybywidthpx + 1] == num){
				map[ybywidthpx + 1] = segNum;
				stack.push_back(ybywidthpx + 1);
		}
	}
}

void segmentRegionGrow(const int &width, const int &height, 
				int *map, const int &num, const int &startNum, int &endNum){

	int ybywidth;
	endNum = startNum;
	for(int y = 0; y < height; y++){
		ybywidth = y * width;
		for(int x = 0; x < width; x++){
			if(map[ybywidth + x] != num)continue;
			else{
				regionGrowing(width, height, map, num, x, y, endNum);
				endNum++;
			}
		}
	}
	endNum--;
}

int getSegmentArea(const int &reso, const int &num, int *map){
	int area = 0;
	for(int it = 0; it < reso; it++){
		if(map[it] == num)area++;
	}
	return area;
}

void numset(int *list, const int &num, const int &size){
	for(int i = 0; i < size; i++)list[i] = num;
}
