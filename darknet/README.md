# Darknet

## Overview
darknet(yolo)による検出と認識  

Author: Narumitu Ikeda, Kazuki Miyazawa

## Nodes

## Subscribed Topics
* **`~/image_capture_info`**  ([vision_module/ObjectInfo])  
RGBDのポイントクラウド情報

## Published Topics
* **`~/object_detection_info`**  ([vision_module/ObjectInfo])  
検出・認識結果  

## Services
* **`~/start`**  ([std_srvs/Trigger])  

* **`~/stop`**  ([std_srvs/Trigger])  

## Parameters

* **`~/DISPLAY`**  (bool,dafault="")  
結果の描画 on/off

* **`~/MAX_DEPTH`**  (float,dafault="3")  
検出の閾値  
最大深さ

* **`~/MAX_DIST`**  (float,dafault="")  
検出の閾値  
カメラから物体までの距離

* **`~/MAX_HEIGHT`**  (float,dafault="")  
検出の閾値  
最大高さ

* **`~/MAX_WIDTH`**  (float,dafault="")  
検出の閾値  
最大幅

* **`~/MIN_HEIGHT`**  (float,dafault="")  
検出の閾値  
最小幅

* **`~/MIN_POINT`**  (int,dafault="")  
検出の閾値  
物体までの距離によって正規化された最小の物体の点群数

* **`~/SAVE_IMAGE`**  (bool,dafault="")  
結果の画像の保存 on/off
