# Face Recognition

## Overview
顔認識  
年齢・特定人物認識・一般人物認識

Author: Narumitu Ikeda, Kazuki Miyazawa

## Nodes

## Subscribed Topics
* **`~/face_detection_info`**  ([vision_module/FaceInfo])  
顔検出結果  

## Published Topics
* **`~/face_recognition_info`**  ([vision_module/FaceInfo])  
顔認識結果  

## Services
* **`~/start`**  ([std_srvs/Trigger])  

* **`~/stop`**  ([std_srvs/Trigger])  

## Parameters

* **`~/DISPLAY`**  (bool,dafault="")  
結果の描画 on/off

* **`~/FACE_LEARN`**  (bool,dafault="")  
顔学習用

* **`~/NUM_PICT`**  (int,dafault="")  

* **`~/SAVE_IMAGE`**  (bool,dafault="")  
結果の画像の保存 on/off

* **`~/SEND_IMAGE`**  (bool,dafault="")  
