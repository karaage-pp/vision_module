
/**********************************************************
Segmentation.h			Copyright 2015.10.02 by N.Ikeda

セグメンテーションプログラム
八近傍でのセグメンテーションを行います

使用時の注意
入力マップ中にはstartNum以上の値があってはいけません。
よって必ずstartNumの値はmap中の値以上のものを使うこと
また、入力マップのデフォルト値は-1で、入力値は1以上

**********************************************************/

/*マップの幅、高さ、マップのポインタ、セグメント化したい領域番号、セグメント番号、最後のセグメント番号
num部分をセグメンテーションし、各セグメントにstartNumから始まるセグメント番号を
割り当てる。endNumには最後のセグメント番号が代入される*/
void segmentate(const int &width, const int &height, 
				int *map, const int &num, const int &startNum, int &endNum);
//池田法(未完成)
void segmentIkedaSearch(const int &width, const int &height, 
				int *map, const int &num, const int &startNum, int &endNum);
//領域拡張法(4近傍)
void segmentRegionGrow(const int &width, const int &height, 
				int *map, const int &num, const int &startNum, int &endNum);

//セグメントの面積を得る
int getSegmentArea(const int &reso, const int &num, int *map);

void numset(int *list, const int &num, const int &size);


//デバッグ用関数　5*5までの大きさのマップを表示する
void printSegment(const int &width, const int &height, int *map);

/*デバッグセット
int map[25] = {
	 1, -1, -1,  1, -1, 
	-1,  1, -1,  1,  1, 
	-1, -1, -1,  1, -1, 
	 1,  1, -1, -1, -1, 
	 1,  1,  1,  1,  1
};
int endNum;
segmentate(5, 5, map, 1, 2, endNum);
printSegment(5, 5, map);
while(1){
	int num;
	scanf("%d", &num);
	if(num > 5)break;
	printf("area : %d\n", getSegmentArea(25, num, map));
}
*/

/*
	int map[100];
	int endNum;
	segmentIkedaSearch(10, 10, map, 1, 2, endNum);
	printSegment(10, 10, map);
	while(1){
		for(int i = 0; i < 100; i++){
			if(rand() % 2)map[i] = -1;
			else map[i] = 1;
		}
		segmentate(10, 10, map, 1, 2, endNum);
		printSegment(10, 10, map);
		int num;
		scanf("%d", &num);
		if(num > 5)break;
	}*/