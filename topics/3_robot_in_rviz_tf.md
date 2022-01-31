# Добавлени робота в RVIZ. Понятие TF.

Давайте немного внимательнее рассмотрим нашу черепашку Turtlebot3. 
Сначала, как и в предыдущий раз, нужно запустить модель робота. 
```
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_gazebo turtlebot3_autorace.launch
```
Далее запусть в новом окне терминала дополнительный launch-файл из состава Turtlebot3.
```
roslaunch turtlebot3_bringup turtlebot3_remote.launch
```
И наконец создадим третье окно терминала и в нем уже запустим RVIZ одноименной командой.

![default_RVIZ](../assets/default_RVIZ.png)

![TF_error](../assets/TF_error.png)

![robot_RVIZ](../assets/robot_RVIZ.png)

![rviz_tf](../assets/rviz_tf.png)
