# Plane Detection

## Overview
ポイントクラウドを用いた平面検出  

Author: Narumitu Ikeda, Kazuki Miyazawa

## Nodes

## Subscribed Topics
* **`~/image_capture_info`**  ([vision_module/ObjectInfo])  
RGBDのポイントクラウド情報  

## Published Topics
* **`~/plane_detection_info`**  ([vision_module/ObjectInfo])  
平面検出結果  

## Services
* **`~/start`**  ([std_srvs/Trigger])  

* **`~/stop`**  ([std_srvs/Trigger])  

## Parameters

* **`~/DISPLAY`**  (bool,dafault="")  
結果の描画 on/off

* **`~/ERROR_RANGE`**  (float,dafault="")  
検出の閾値  

* **`~/MAX_DIST`**  (float,dafault="")  
検出の閾値  
カメラから物体までの最大距離

* **`~/MIN_DIST`**  (float,dafault="")  
検出の閾値  
カメラから物体までの最小距離

* **`~/MAX_GAP`**  (float,dafault="")  
検出の閾値  

* **`~/MIN_SEGMENT_SIZE`**  (float,dafault="")  
検出の閾値  
最小の平面サイズ

* **`~/SEARCH_ITERATION`**  (int,dafault="")  
検出アルゴリズムの繰り返し回数

* **`~/SAVE_IMAGE`**  (bool,dafault="")  
結果の画像の保存 on/off
