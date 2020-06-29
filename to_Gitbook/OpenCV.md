## Цветные маркеры
```l22_aero_vision/src/color_r_c.py```

Для обработки изображения с камеры и детектирования объектов мы использовали функции из библиотеки OpenCV.

Алгоритм

1. Получение изображения и параметров камеры
1. Построение маски по определенному диапазону цветов (в формате HSV)
1. Детектирование контуров цветных объектов
1. Определение типа объекта, получение ключевых точек объекта на изображении
1. Определение положения квадратов и кругов с помощью solvePnP основываясь на реальных размерах объектов и точек на изображении ( [Opencv Docs](https://docs.opencv.org/3.4/d9/d0c/group__calib3d.html#ga549c2075fac14829ff4a58bc931c033d) )
1. Отправка результата в топики ```/l22_aero_color/markers```  и  ```/l22_aero_color/circles``` ( координаты относительно main_camera_optical )

Для упрощения работы были создани свои типы сообщений, а так же сервис для настройки параметров детектора во время посадки. (ColorMarker, ColorMarkerArray, SetParameters)


<!-- IMAGE or VIDEO -->
<img src="https://github.com/vas0x59/ior2020_uav_L22_AERO/raw/master/to_Gitbook/content/5_D1_2.png" height="355">
<video autoplay loop src="https://github.com/vas0x59/ior2020_uav_L22_AERO/raw/master/to_Gitbook/content/r1.mp4" height="360" ></video>
<p><i>Примеры распознование маркеров</i></p>

## Визуализация в RViz
```l22_aero_vision/src/viz.py```

Для отладки мы сделали скрипт визуализирующий координаты маркеров в среде RViz.

<!-- IMAGE or VIDEO -->
<video autoplay loop src="https://github.com/vas0x59/ior2020_uav_L22_AERO/raw/master/to_Gitbook/content/RViz.mp4" height="360" ></video>


