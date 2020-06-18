# Инструкция по запуску  

- Создайте workspace  
- Скачайте архив и разархивируйте в папку `src` workspace'a:  
`git clone https://github.com/vas0x59/ior2020_uav_L22_AERO`  
Убедитесь, что папки расположены примерно в таком виде:  
`catkin_ws/src/ior2020_uav_L22_AERO/l22_aero_code`  
`catkin_ws/src/ior2020_uav_L22_AERO/l22_aero_vision`    
- В корне workspace'a выполните:  
`catkin_make`  
`source devel/setup.bash`  
- Запустите код:  
`roslaunch l22_aero_code run.launch`  
