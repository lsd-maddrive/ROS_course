# Собираем телегу в Gazebo. Плагины.

Пришло время собрать свою собственную модель робота. 
Сделать это можно несколькими способами: с помощью URDF или SDF.
URDF или SDF это два разных формата описания робота. Оба этих формата для создания моделей используют язык XML. Отличаются форматы своим функционалом, 
несколько отличающимися способами описания различных элементов, но в целом довольно схожи. 

> Здесь мы более подробно остановимся на формате SDF (Simulation Description Format). Данный формат специально предназначем для моделирования в симуляторе Gazebo.

> Что такое Gazebo? Gazebo - это среда для моделирования роботов. Он позволяет быстро и точно создавать роботов, воссоздавать окружающую среду 
(как уличную, так и помещения) и безопасно и эффективно отлаживать алгоритмы управления роботом. Одной из ключевых возможностей Gazebo - это возможность 
имитировать работу различных датчиков таких, как лидар, радар, камера, инерциальный датчик. Для работы с датчиками и управления роботом пользователю предосталяются 
встроенные API.

> Робот созданный в формате SDF, как правило, состоит из следующих основных частей:
- Link (звено) - основные видимые элеметны конструкции: мобильные базы, колеса, звенья манипуляторов и тд
- Joint (соединение) - место сочленения двух звеньев.
- Sensor (датчик)

> Файл модели имеет расширение `*.sdf`. Макет описания робота можно представитьв следующем виде:
```xml
<?xml version='1.0'?>
<sdf version='1.4'>
  <model name="<model_name>" xmlns:xacro="http://www.ros.org/wiki/xacro">
    <link name='base_link'> <!--создание базы мобильного робота-->
      <visual name='base_link_visual'>
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
      </visual>
    </link>
    <joint name="wheel_joint" type="revolute">  <!--создание места сочленения базы и колеса-->
      <parent>base_link</parent>
      <child>wheel_steer_link</child>
      <pose relative_to="base_link">0 0 1 0 0 0</pose> <!--место сочленения позиционируется относительно центра мобильной базы-->
    </joint>
    <link name="wheel_steer_link">  <!--создание колеса мобильного робота-->
      <pose relative_to="wheel_steer_joint">0.25 0 0 0 0 0</pose>  <!--центр колеса позиционируется относительно места сочленения-->      
      <visual name='wheel_steer_visual'>
          <geometry>
            <box>
              <size>0.5 0.2 0.1</size>
            </box>
          </geometry>
      </visual>
    </link>
  </model>
</sdf>
```

### Рассмотрим более подробно некоторые основные элементы и особенности моделирования с использованием SDF
Первый элемент это link. Он включает в себя следующие параметры:
1) pose - положение звена. Желательно указывать относительно чего выбрано положение. Положение указывается в координатах `x y z roll pith yaw`.
2) collision - опеределяет область взаимодействия с другими объектами. Как правило форма этой области значительно упрощена и представляет собой куб, 
цилиндр или сферу.
3) visual - определяет внешний вид звена. Может быть задан простой фигурой. Существует фозможность загрузки сторонних моделей в формате STL.
4) inertial - в этом параметре задаются масса и момент инерции звена.

Второй элемент это joint. Он включает в себя следующие параметры:
1) тип сочленения: фиксированное, вращающееся, призматическое и другие.
2) pose - положение сочленения.
3) parent - обычно более старшее звено в сочленении.
4) child - более младшее звено, которое крепится к старшему.
5) axis - определяет оси, в которых сочленение может двигаться (вращаться). Этот параметр также может включать в себя ограничения на движение сочленения.


Третьим элементом является sensor. Его создание происходит в 3 этапа:
1) Создание звена, являющегося телом датчика (вместе с сочленением с остальным роботом, если это необхожимо)
2) Создание непосредственно самого датчика и задание его параметров. В SDF присутствует большое количество датчиков и все они требуют индивидуальной настройки.
3) Подключение плагина для имитации работы датчика и получения в ROS данных от него.

> Здесь приведены лишь краткие сведения по созданию модели. Полный перечень всех элементов модели, их описание и параметры доступны на официальном сайте 
> SDFormat: http://sdformat.org/spec?ver=1.8&elem=sdf

### Как заставить робот двигаться?
Это действительно не тривиальная задача. В открытом доступе приведено можество различных инструкций. Однако работают они далеко на со всеми версиями ROS и Gazebo.
Это еще одна причина выбора SDF как основного инструмента моделирования роботов в ROS. Он ниболее стабильно работает во всех версиях ROS и Gazebo, однако названия некоторых библиотек и функций может отличаться. Поэтому припоявлении ошибок компиляции следует обратиться к документации Gazebo: http://osrf-distributions.s3.amazonaws.com/gazebo/api/dev/index.html
Ниже приведен базовый пример того, как долен выглядеть простейший плагин модели. Файл с кодом должен иметь расширение `*.сс`. При необходимости добавить заголовочный файл он будет соответственно иметь расширение `*.hh`.
```c++
#include <ros/ros.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <functional>
// Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>

namespace gazebo
{
  /// \brief A plugin to control a Velodyne sensor.
  class MyPlugin : public ModelPlugin
  {
    /// \brief Constructor
    public: MyPlugin() {}
    
    /// \brief Pointer to the model.
    private: physics::ModelPtr model;

    /// \brief Pointer to the joint.
    private: physics::JointPtr joint;
    
    event::ConnectionPtr update_connection_;

    /// \brief A PID controller for the joint.
    private: common::PID pid;
    
    private: double velocity;

    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      // Safety check
      if (_model->GetJointCount() == 0)
      {
        std::cerr << "Invalid joint count, Velodyne plugin not loaded\n";
        return;
      }

      // Store the model pointer for convenience.
      this->model = _model;

      // Get the first joint. We are making an assumption about the model
      // having one joint that is the rotational joint.
      this->joint = _model->GetJoint(_sdf->GetElement("JointSteer")->Get<std::string>());
      
      if (_sdf->HasElement("velocity"))
        velocity = _sdf->Get<double>("velocity");

      // Setup a P-controller, with a gain of 0.1.
      this->pid = common::PID(0.1, 0, 0);

      // Apply the P-controller to the joint.
      this->model->GetJointController()->SetVelocityPID(
          this->joint->GetScopedName(), this->pid);
          
      // listen to the update event (broadcast every simulation iteration)
      this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
          std::bind(&ModelPush::OnUpdate, this));
    }
    
    public: void OnUpdate()
    {
      // Set the joint's target velocity. This target velocity is just
      // for demonstration purposes.
      this->model->GetJointController()->SetVelocityTarget(
          this->joint->GetScopedName(), velocity);    
    }
  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(MyPlugin)
}
```
Здесь следует пояснить некоторые моменты. 
1. Плагир всегда должен являеться классом.
2. "Точкой входа" в плагин является функция `Load`. В этой функции, как правило, в плагин передаются все необходимые для его работы параметры: ссылка на модель в 
Gszebo, ссылка на модель в SDF, параметры, которые загружаются из кода на SDF.
3. При необхожимости выполнять действия с определенной периодичность следует отслеживать событие обновления модели, которое имеет класс `event::Events::ConnectWorldUpdateBegin`. В это объект передается ссылка на `callback` функцию, которая и будет вызывать.
Для комиляции плагина предворительно следует создать пакет. В нем создать папки `sdf` и `src`, скоротых соответственно будут помещены модель и код плагина.
После создания пакета автоматически были созданы файлы `CMakeList.txt` и `package.xml`.
В файл `CMakeList.txt` следует поместить следующее:
```CMake
cmake_minimum_required(VERSION 3.0.2)
project(PackageName)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  xacro
)

find_package(gazebo REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")


## Declare a C++ library
add_library(MyPlugin
   src/MyPlugin.cc
)

target_link_libraries(MyPlugin
  ${GAZEBO_LIBRARIES}
)
```
В файл `Pakage.xml` следует поместить следующее:
```xml
  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>gazebo_ros</build_depend>
  <build_depend>roscpp</build_depend>
  <build_depend>xacro</build_depend>
  <build_export_depend>gazebo_ros</build_export_depend>
  <build_export_depend>roscpp</build_export_depend>
  <build_export_depend>xacro</build_export_depend>
  <exec_depend>gazebo_ros</exec_depend>
  <exec_depend>roscpp</exec_depend>
  <exec_depend>xacro</exec_depend>
```
После успешной компиляции плагина можно его подключить к нашей модели.
Подключение плагина к модели выглядит следующим образом:
```xml
<plugin name="MyPlugin" filename="libMyPlugin.so">
  <JointSteer>wheel_steer_joint</JointSteer> <!--Параметры, которые мы хотим передать плагину-->
  <velocity>10</velocity>
</plugin>
```
Осталось только запустить симуляцию нашего робота. 
Для этого написем простенький launch файл, который будет содержать команду сборки модели, создание пустой окружающей среды и загрузку в среду непосредственно самого робота. Выглядеть это буджет следующим образом:
```xml
<launch> 
  <param name="robot_description" command="$(find xacro)/xacro $(find <package_name>)/sdf/model.sdf.xacro" />
  <include file="$(find gazebo_ros)/launch/empty_world.launch"/> <!--можно загрузить любой другой мир из доступных в Gazebo или создать и сохранить свой-->
	<node name="mobot_spawn" pkg="gazebo_ros" type="spawn_model" output="screen" args="-sdf -param robot_description -model <model_name>" />
</launch>
```
Полезной функией является возможность разместить робота в нужной точке. Для этого нужно добавить в аргументы узла `mobot_spawn` следующую команду `-x $(arg x_pos) -y $(arg y_pos) -z $(arg z_pos)`.

### Задания:
1. Повторите приведенный пример. 
2. Попробуйте изменить внежний вид звеньев, тип сочленения. Попробуйте заменить в плагине функцию `SetVelocityTarget` на `SetPositionTarget`.
3. Соберите своего дифференциального робота (или любого другого на выбор). Дифференциальный робот будет состоять из базы, двух колес и  сферы (для опоры).
