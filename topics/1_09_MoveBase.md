# Настройка MoveBase

- [DWA Local Planner](#dwa-local-planner)
- [Costmap Common Params](#costmap-common-params)
- [Local and Global CostMAP](#local-and-global-costmap)
- [Основные параметры. Move Base Params](#основные-параметры-move-base-params)
- [Чему научились?](#чему-научились)
- [Задание](#задание)
- [Вопросики](#вопросики)
- [Ресурсы](#ресурсы)

Навигационный стек имеет множество параметров, которые необходимо настраивать в зависимости от конфигурации робота. Как думаете, подойдут ли параметры двухкилограммовой черепашки к двухтонному грузовику с кинематикой Аккермана? Очень хотелось бы, но к сожалению к каждому объекту требуется свой подход и настройка. Давайте рассмотрим на примере черепашки, как изменение параметров влияет на характер её движения. 


## DWA Local Planner

В прошлом топике мы уже начали знакомиться с локальным планировщиков [DWA local planner](http://wiki.ros.org/dwa_local_planner), однако все параметры в нем были выставлены по default. Для лучшего понимания смысла параметров, рекомендуем немного изучить принцип работы данного планировщика на странице ROS Wiki, либо ещё где-то. Давайте посмотрим, что в нем вообще имеется, откроем наш `dwa_local_planner_params_waffle.yaml`. Из названия уже понятно, что там должны быть какие-то параметры :speak_no_evil:

```xml
DWAPlannerROS:

# Robot Configuration Parameters
  max_vel_x: 0.26
  min_vel_x: -0.26

  max_vel_y: 0.0
  min_vel_y: 0.0

# The velocity when robot is moving in a straight line
  max_vel_trans:  0.26
  min_vel_trans:  0.13

  max_vel_theta: 1.82
  min_vel_theta: 0.9

  acc_lim_x: 2.5
  acc_lim_y: 0.0
  acc_lim_theta: 3.2 

# Goal Tolerance Parameters
  xy_goal_tolerance: 0.05
  yaw_goal_tolerance: 0.17
  latch_xy_goal_tolerance: false

# Forward Simulation Parameters
  sim_time: 2.0
  vx_samples: 20
  vy_samples: 0
  vth_samples: 40
  controller_frequency: 10.0

# Trajectory Scoring Parameters
  path_distance_bias: 32.0
  goal_distance_bias: 20.0
  occdist_scale: 0.02
  forward_point_distance: 0.325
  stop_time_buffer: 0.2
  scaling_speed: 0.25
  max_scaling_factor: 0.2

# Oscillation Prevention Parameters
  oscillation_reset_dist: 0.05

# Debugging
  publish_traj_pc : true
  publish_cost_grid_pc: true

```

Так, ну мы вас не обмунули и вы, действительно, можете увидеть тут кучу каких-то параметров, которые пока что особо не понятно, за что отвечают. Настало время разобраться с этим. Все параметры сгруппированы в несколько категорий, а именно:

- `Robot Configuration Parameters` - параметры конфигурации робота 
- `The velocity when robot is moving in a straight line` - параметры при движении по прямой линии
- `Goal Tolerance Parameters` - параметры допустимого отклонения от цели
- `Forward Simulation Parameters` - параметры моделирования. (При планировании маршрута к цели, алгоритм проводит моделирование движения из текущего местоположения, для предсказания осуществимости траектории и выбора наилучшей)
- `Trajectory Scoring Parameters` - параметры оценки траектории
- `Oscillation Prevention Parameters` - параметры предотвращения колебаний
- `Debugging`- отладка

Запускаем наш созданный в прошлом топике launch `turtlebot3_gazebo_slam.launch`

Для того, чтобы проще было настраивать параметры конфигурации в запущенной системе (и не перезапускать по 20 раз) воспользуемся `rqt_reconfigure`

```bash
rosrun rqt_reconfigure rqt_reconfigure
```

<p align="center">
<img src=../assets/01_11_rqt_reconfigure.png width=900/>
</p>

Напимер, давайте посмотрим на параметр `max_vel_theta`. Для `waffle` этот параметр задан, как `max_vel_theta: 1.82`

<p align="center">
<img src=../assets/01_12_dwa_max_theta1_82.gif width=600/>
</p>

Окей, а что будет, если этот параметр понизить до 0.5? А потом до 0? Пробуем:

`max_vel_theta = 0.5`
<p align="center">
<img src=../assets/01_13_dwa_max_theta0_5.gif width=600/> 
</p>

`max_vel_theta = 0.0`
<p align="center">
<img src=../assets/01_14_dwa_max_theta0.gif width=600/>
</p>


> :muscle: Время сделать вывод... каким образом изменение параметра `max_vel_theta` сказывается на планировании маршрута робота?

> :muscle: Теперь попробуйте самостоятельно поменять параметры `sim_time` и `occdist_scale`. Сделайте вывод о том, как уменьшение/увеличение этих значений влияет на планирование маршрута робота.

Получилось? Здорово! Тогда двигаемся дальше

## Global Planner

Про глобальный планнер было сказано много слов, но пока что нигде в файлах не встречались упоминания о нем (особо). Дело в том, что move_base и в частности turtlebot в качестве глобального планировщика по умолчанию использует [navfn](http://wiki.ros.org/navfn?distro=noetic). Он довольно простенький и особо не имеет параметров. Давайте попробуем изменить стандарный глобальный планнер на [global_planner](http://wiki.ros.org/global_planner?distro=noetic) путем добавления в наш move_base.launch следующего параметра:

```bash
<param name="base_global_planner" value="global_planner/GlobalPlanner" />
```
> Если когда нибудь подумаете, что у вас плохо с фантазией, вспомните, что разработчик дал своему глобальному планнеру имя "GlobalPlanner" :joy:

Теперь при запуске будет использовать данный планнер, со стандартными параметрами. Смену планнера можно увидеть по новым названиям топиков для глобального пути: `/move_base/GlobalPlanner/plan` Список параметров планировщика можно глянуть на ROS wiki, но как и с локальным параметрами можно выгружать из конфига. Для этого в своей папке с конфигами создаем файл `global_planner_params.yaml` со следующим содержанием:

```bash
GlobalPlanner:                                  # Also see: http://wiki.ros.org/global_planner
  old_navfn_behavior: false                     # Exactly mirror behavior of navfn, use defaults for other boolean parameters, default false
  use_quadratic: true                           # Use the quadratic approximation of the potential. Otherwise, use a simpler calculation, default true
  use_dijkstra: true                            # Use dijkstra's algorithm. Otherwise, A*, default true
  use_grid_path: false                          # Create a path that follows the grid boundaries. Otherwise, use a gradient descent method, default false
  
  allow_unknown: true                           # Allow planner to plan through unknown space, default true
                                                #Needs to have track_unknown_space: true in the obstacle / voxel layer (in costmap_commons_param) to work
  planner_window_x: 0.0                         # default 0.0
  planner_window_y: 0.0                         # default 0.0
  default_tolerance: 0.0                        # If goal in obstacle, plan to the closest point in radius default_tolerance, default 0.0
  
  publish_scale: 100                            # Scale by which the published potential gets multiplied, default 100
  planner_costmap_publish_frequency: 0.0        # default 0.0
  
  lethal_cost: 253                              # default 253
  neutral_cost: 50                              # default 50
  cost_factor: 3.0                              # Factor to multiply each cost from costmap by, default 3.0
  publish_potential: true                       # Publish Potential Costmap (this is not like the navfn pointcloud2 potential), default true
```

Далее добавляем строчку в наш move_base.launch 
```bash
<rosparam file="$(find super_robot_package)/config/global_planner_params.yaml" command="load" />
```

> :muscle: Попробуйте изменить параметры use_dijkstra, use_grid_path и понять как меняется вид глобальной траектории. В каких случаях следует выставлять параметр allow_unknown в значение false?


## Costmap Common Params

В ROS, как правило, в стеке навигации существуют две карты, хранящие информацию о препятствиях в окружающей среде. Это local and global costmap. Из названия понятно, что одна из них глобальная - та, которая строит протяженный маршрут до конечной цели, а локальная - способна перестраивать маршруты непосредственно в те моменты, когда на свободном пространстве по какой-то причине появилось препятствие, которое следует объехать.

В `costmap_common_params` устанавливается, как правило, что будет касаться, как и локального, так и глобального планировщика.
Тут применяется карта стомоимости, предоставляющая сведения о препятсвиях в мире. Для того, чтобы обеспечить коррректную связь между ними - нужно связать карты затрат с теми устройствами, которые будут предоставлять им информацию об окружающей обстановке. В общем виде конфигурация параметров выглядит так :point_down:

```bash
obstacle_range: 2.5
raytrace_range: 3.0
footprint: [[x0, y0], [x1, y1], ... [xn, yn]]
#robot_radius: ir_of_robot
inflation_radius: 0.55

observation_sources: laser_scan_sensor point_cloud_sensor

laser_scan_sensor: {sensor_frame: frame_name, data_type: LaserScan, topic: topic_name, marking: true, clearing: true}

point_cloud_sensor: {sensor_frame: frame_name, data_type: PointCloud, topic: topic_name, marking: true, clearing: true}
```
Давайте тут уже остановимся чуть-чуть поподробнее. Итак:

- `obstacle_range` - максимальная дальность, при которой робот будет учитывать препятствия и наносить их на свою карту. В нашем случае - 2.5 метра.
- `raytrace_range` - максимальность дальность, при которой будет происходить чистка карты от динамически возникающих объектов. 
- `footprint` - размеры робота, предполагая, что его центр находится в точке (0;0),(0;0)
- `robot_radius` - размеры робота, если он круглый
- `inflation_radius` - увеличение ширины стен и препятствий. Например, если этот параметр задан, как 0.55, то тогда робот будет прокладывать маршрут таким образом, чтобы не притираться к увеличенным стенам/препятствиям на 0.55 метра. Посмотрите рисунок ниже, чтобы лучше понять это.

<p align="center">
<img src=../assets/01_15_tuning_inflation_radius.png width=700/>
</p>

- `observation_sources` - список из датчиков, которые будут публиковать данные в карту затрат
- `laser_scan_sensor` или `point_cloud_sensor`, где в качестве `sensor_frame` устанавливается система координат датчика, `data_type` - тип датчика, `topic` - имя топика, `marking` и `clearing` - определение разрешения на добавление и очистку соответсвенно препятствий с карты затрат. Стандартным типом данных для лидара в ROS является LaserScan, а для камер глубины или 3D лидаров - PointCloud. 

Надеемся, что теперь стало еще более понятно. Поэтому давайте откроим наш `costmap_common_params_[model_name].yaml` 


```bash
obstacle_range: 3.0
raytrace_range: 3.5

footprint: [[-0.205, -0.155], [-0.205, 0.155], [0.077, 0.155], [0.077, -0.155]]
#robot_radius: 0.17

inflation_radius: 1.0
cost_scaling_factor: 3.0

map_type: costmap
observation_sources: scan
scan: {sensor_frame: base_scan, data_type: LaserScan, topic: scan, marking: true, clearing: true}

```

> :muscle: Помейняйте `inflation_radius` нашего робота. Верно ли то, что при каком-то значении он не сможет проехать между препятствиями? Если да, то при каком?

### Local and Global Costmap

Разбираемся с глобальной картой стоимости, открываем соответствующий `.yaml` файл.

- `global_frame` - параметр, определяющий в какой системе координат будет карта стоимости.
- `robot_base_frame` - фрейм система координат робота, 
- `update_frequency` - частота обновления карты 
- `publish_frequency` - частота публикации карта
- `transform_tolerance`
- `static_map` - принимает значения либо `True` либо `False` и определяет должен ли робот локализировать себя на уже существующей, подгруженной карте
- `footprint` - Габариты робота на плосоксти


Таким образом, мы разобрали параметры нашего `global_costmap_params.yaml`

```bash
global_costmap:
  global_frame: map
  robot_base_frame: base_footprint

  update_frequency: 10.0
  publish_frequency: 10.0
  transform_tolerance: 0.5

  static_map: true
```

В `local_costmap_params.yaml` должно быть все относительно понятно из предыдущего разбора, за исключением параметра `rolling_window`, принимающий либо `True` либо `False`. Если параметр будет принимать значение `True`, то в таком случае карта стоимости будет оставаться центрированной вокруг робота, когда робот перемещается по миру. Параметры `width`, `height` и `resolution` задают ширину, высоту и разрешение в метрах карты затрат. Посмотрим на него:

```bash
local_costmap:
  global_frame: odom
  robot_base_frame: base_footprint

  update_frequency: 10.0
  publish_frequency: 10.0
  transform_tolerance: 0.5  

  static_map: false  
  rolling_window: true
  width: 3
  height: 3
  resolution: 0.05
```

## Основные параметры. Move Base params

Осталось совсем немного, давайте напоследок взглянем на наш `move_base_params.yaml`

```bash
shutdown_costmaps: false
controller_frequency: 10.0
planner_patience: 5.0
controller_patience: 15.0
conservative_reset_dist: 3.0
planner_frequency: 5.0
oscillation_timeout: 10.0
oscillation_distance: 0.2
```

К основным параметрам move_base относятся:

* base_global_planner - тип глобального планировщика 
* base_local_planner - тип локального планировщика 
* controller_frequency - частота работы навигации и отправки команд роботу
* controller_patience - время ожидания адекватных данных управления до очестки карты
* planner_frequency - частота одновления глобального палана
* planner_patience - время ожидания поиска пути до очистки карты
* oscillation_timeout - время восстановления данных (карты)
* oscillation_distance - расстояние которое нужно пройти для восстановления

## Чему научились?

В этот раз мы с вами более подробно погрузились в стек навигации наших роботов и разобрали все конфиги, которые в нем используются.

## Задание

Основное задание для вас - это пройти внимательно этот топик, почитайте внимательно про параметры, посмотрите на что они влияют. И убедитесь, что даже небольшое изменение одного из них может существенно сказаться на всем планировщике в целом. С этим нужно быть внимательным.

## Вопросики

1. Имеет ли смысл в глобальном планировщике использовать параметр rolling_window? Почему?
2. На какую карту помещаются резко возникающие препятствия?
3. Возможно ли очистить карту во время движения робота от динамически возникающих объектов (людей, кошечек, машинок)? Или они так и останутся на ней.

## Ресурсы

Более подробную информацию о настройке стека навигации вы можете найти:
* [Basic Navigation Tuning Guide](http://wiki.ros.org/navigation/Tutorials/Navigation%20Tuning%20Guide)
* [ROS Navigation Tuning Guide by Kaiyu Zheng](http://kaiyuzheng.me/documents/navguide.pdf)
* 11 глава [ROS Robot Programming book](https://community.robotsource.org/t/download-the-ros-robot-programming-book-for-free/51)