# Face Detection

## Overview
顔検出  
opencvの顔検出に3次元形状によるフィルタリングを行う

Author: Narumitu Ikeda, Kazuki Miyazawa

## Nodes

## Subscribed Topics
* **`~/image_capture_info`**  ([vision_module/ObjectInfo])  
RGBDのポイントクラウド情報  

## Published Topics
* **`~/face_detection_info`**  ([vision_module/FaceInfo])  
顔検出結果  

## Services
* **`~/start`**  ([std_srvs/Trigger])  

* **`~/stop`**  ([std_srvs/Trigger])  

## Parameters
* **`~/DISPLAY`**  (bool,dafault="")  
結果の描画 on/off

* **`~/SAVE_IMAGE`**  (bool,dafault="")  
結果の画像の保存 on/off
