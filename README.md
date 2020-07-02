# ior2020_uav_L22_AERO
[Задание](http://robolymp.ru/files/ior2020/ibpla/IOR2020_online_iUAV_final_regulations_v2.pdf)  
[Инструкция](https://github.com/vas0x59/ior2020_uav_L22_AERO/blob/master/instruction.pdf)


# Final Code - ALL_IN_ONE.py

## ROS
Созданные ноды, топики, сообщения и сервисы


### Nodes

<ul style="list-style: none; font-size:15px; padding-left:10px">
 <li>l22_aero_vision/<b>color_r_c.py</b> - распознование цветных объектов</li>
 <li>l22_aero_vision/<b>viz.py</b> - визуализация в RViz</li>
 <li>l22_aero_code/<b>full_task.py</b> - основной код  </li>
</ul>

### Topics

<ul style="list-style: none; font-size:15px; padding-left:10px">
 <li><b>/l22_aero_color/markers</b> l22_aero_vision/ColorMarkerArray - список прямоугольных маркеров</li>
 <li><b>/l22_aero_color/circles</b> l22_aero_vision/ColorMarkerArray - список круглых маркеров</li>
 <li><b>/l22_aero_color/debug_img</b> sensor_msgs/Image - изображение для отладки </li>
 <li><b>/qr_debug</b> sensor_msgs/Image - изображение для отладки </li>
</ul>


### Messages
#### ColorMarker
```cpp
string color
int16 cx_img
int16 cy_img
float32 cx_cam
float32 cy_cam
float32 cz_cam
float32 size1
float32 size2
int16 type
```
#### ColorMarkerArray
```cpp
std_msgs/Header header
l22_aero_vision/ColorMarker[] markers
```
### Services
#### SetParameters
```
float32 rect_s1
float32 rect_s2
float32 circle_r
int32 obj_s_th
int32 offset_w
int32 offset_h
---
```
