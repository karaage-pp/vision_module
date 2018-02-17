# Object Recognition

## Overview
物体認識  
位置・サイズ・特定物体認識・一般物体認識

Author: Narumitu Ikeda, Kazuki Miyazawa

## Nodes

## Subscribed Topics
* **`~/object_detection_info`**  ([vision_module/ObjectInfo])  
物体検出結果

## Published Topics
* **`~/object_recognition_info`**  ([vision_module/ObjectInfo])  
物体認識結果  

## Services
* **`~/start`**  ([std_srvs/Trigger])  

* **`~/stop`**  ([std_srvs/Trigger])  

## Parameters
* **`~/DISPLAY`**  (bool,dafault="")  
結果の描画

* **`~/NUM_CAND`**  (inf,dafault="3")  
認識結果の表示個数

* **`~/PUB_FEATURES`**  (bool,dafault="")  
AlexNetの中間層4096次元の特徴量を出力 on/off

* **`~/SAVE_IMAGE`**  (bool,dafault="")  
結果の画像の保存 on/off

* **`~/SEND_IMAGE`**  (,dafault="")  
