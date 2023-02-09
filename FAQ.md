# Frequently asked questions (FAQ)

- [Как установить ROS?](#как-установить-ros)
- [Как установить пакеты ROS?](#как-установить-пакеты-ros)
- [Как удобно управляться в системе пакетов ROS?](#как-удобно-управляться-в-системе-пакетов-ros)
- [Как делать `git push` с паролем](#как-делать-git-push-с-паролем)

## Как установить ROS?

Установка ROS описана [на официальном сайте](http://wiki.ros.org/noetic/Installation/Ubuntu).

Не забываем после установки активировать **системное рабочее пространство**. Для этого нужно выполнить в терминале 2 команды:

```bash
# Активация ROS при каждом запуске терминала
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
# Активация ROS в этом терминале
source ~/.bashrc
```

## Как установить пакеты ROS?

Вот такой командой мы устанавливаем пакеты в Ubuntu:

```bash
sudo apt install [имя пакета]
```

Для ROS, например, пакеты имеют шаблон `ros-noetic-[название]`. Например, для установки `move-base` вызываем команду: 

```bash
sudo apt install ros-noetic-move-base
```

## Как удобно управляться в системе пакетов ROS?

Для работы с пакетами в консоли был разработан целый пакет - [rosbash](http://wiki.ros.org/rosbash). 

В него входят утилиты:

- `roscd` - перейти в пакет

```bash
roscd study_pkg
```

- `rosls` - отобразить список файлов пакета

```bash
rosls study_pkg
```

- `rosed` - изменить (открыть редактор) файл пакета

```bash
# Переменная EDITOR задает редактор, через который откроется файл
export EDITOR=code
rosed study_pkg CMakeLists.txt
```

- `roscp` - копировать файл пакета

```bash
# Копировать `CMakeLists.txt` в домашнюю директорию
roscp study_pkg CMakeLists.txt ~/
```

- `rosrun` - запустить узел в пакете

```bash
rosrun rospy_tutorials talker
```

Утилиты `rospd` и `rosd` работают со стеком директорий. Их использование можно опустить до специальных приемов.

## Как делать `git push` с паролем

Для пуша через HTTPS вариант в GitHub необходимо создать токен для доступа. Делается это в настройках (Settings) пользователя. Далее идем в Developer settings -> Personal access token.

> Можно напрямую по ссылке, если в браузере авторизированы: [тык](https://github.com/settings/tokens).

После этого создается токен, указывается длительно жизни токена (через это кол-во дней стухает) и даете доступ до `repo`.

Готово, обязательно сохрани его, показывается он один раз. Хотя ничего не мешает создавать его каждый раз, если так удобно.

Вот картинкой:

<p align="center">
<img src=assets/faq_token_creation.png />
</p>

<!-- 
## Устанавливаем пакет для "Hello ROS"

Следующим шагом скачаем и установим все нужные нам для работы пакеты ROS. Для установки чего-то на Ubuntu используется команда:
```bash
sudo apt-get install [имя пакета]
```
- Пакет для работы с TurtleBot3
    ```bash
    sudo apt-get install ros-noetic-turtlebot3-msgs ros-noetic-turtlebot3-gazebo
    ```
- Пакет стека навигации
    ```bash
    sudo apt-get install ros-noetic-navigation
    ```
- Пакет move_base
    ```bash
    sudo apt-get install ros-noetic-move-base
    ```
- Прочие пакеты для навигации и планирования
    ```bash
    sudo apt-get install ros-noetic-teb-local-planner ros-noetic-gmapping ros-noetic-hector-mapping
    ```
- Пакет turtlesim (это будет наш hello world) 
    ```bash
    sudo apt-get install ros-noetic-turtlesim
    ``` -->

