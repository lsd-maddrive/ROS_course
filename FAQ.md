# Frequently asked questions (FAQ)

- [Как установить ROS?](#как-установить-ros)
- [Как установить пакеты ROS?](#как-установить-пакеты-ros)
- [Как удобно управляться в системе пакетов ROS?](#как-удобно-управляться-в-системе-пакетов-ros)
- [Я делаю `git commit`, а он хочет e-mail и имя](#я-делаю-git-commit-а-он-хочет-e-mail-и-имя)
- [Как делать `git push` с паролем](#как-делать-git-push-с-паролем)
- [Создал удаленный репозиторий с README, а он не дает сделать push и pull с ошибкой `fatal: refusing to merge unrelated histories`](#создал-удаленный-репозиторий-с-readme-а-он-не-дает-сделать-push-и-pull-с-ошибкой-fatal-refusing-to-merge-unrelated-histories)

## Как установить ROS?

Установка ROS описана [на официальном сайте](http://wiki.ros.org/noetic/Installation/Ubuntu).

Не забываем после установки активировать **системное рабочее пространство**. Для этого нужно выполнить в терминале 2 команды:

```bash
# Активация ROS при каждом запуске терминала
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
# Активация ROS в этом терминале
source ~/.bashrc
```

Если используется в качестве оболочки zsh, то пользуется немного другим `setup` файлом:

```bash
# Активация ROS при каждом запуске терминала
echo "source /opt/ros/noetic/setup.zsh" >> ~/.zshrc
# Активация ROS в этом терминале
source ~/.zshrc
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

## Я делаю `git commit`, а он хочет e-mail и имя

Во время первого коммита Git может ругнуться:

```console
*** Please tell me who you are.

Run

  git config --global user.email "you@example.com"
  git config --global user.name "Your Name"

to set your account's default identity.
Omit --global to set the identity only in this repository.
```

Значит надо настроить данные о пользователе:

```bash
git config user.email "user@mail.ru"
git config user.name "User User"
```

> Использование опции `--global` настраивает почту и имя на всю систему. Без использования опции настройка применится только к репозиторию, в котором делается коммит.

## Как делать `git push` с паролем

Для пуша через HTTPS вариант в GitHub необходимо создать токен для доступа. Делается это в настройках (Settings) пользователя. Далее идем в Developer settings -> Personal access token.

> Можно напрямую по ссылке, если в браузере авторизированы: [тык](https://github.com/settings/tokens).

После этого создается токен, указывается длительно жизни токена (через это кол-во дней стухает) и даете доступ до `repo`.

Готово, обязательно сохрани его, показывается он один раз. Хотя ничего не мешает создавать его каждый раз, если так удобно.

Вот картинкой:

<p align="center">
<img src=assets/faq_token_creation.png />
</p>

## Создал удаленный репозиторий с README, а он не дает сделать push и pull с ошибкой `fatal: refusing to merge unrelated histories`

Это нередкая проблема на началах, когда отдельно создается локальный и отдельно удаленный репозиторий с README или другими файлами.

Чаще при начале проекта сначала создается удаленный и просто клонируется, но попробуем решить проблему!

Такая проблема при попытке push показывает:

```console
To github.com:KaiL4eK/my-new-super-repo.git
 ! [rejected]        main -> main (fetch first)
error: failed to push some refs to 'git@github.com:KaiL4eK/my-new-super-repo.git'
hint: Updates were rejected because the remote contains work that you do
hint: not have locally. This is usually caused by another repository pushing
hint: to the same ref. You may want to first integrate the remote changes
hint: (e.g., 'git pull ...') before pushing again.
hint: See the 'Note about fast-forwards' in 'git push --help' for details.
```

Подсказка pull не помогает с ошибкой:

```console
From github.com:KaiL4eK/my-new-super-repo
 * branch            main       -> FETCH_HEAD
fatal: refusing to merge unrelated histories
```

(Рекомендуем) Есть вариант сделать pull с опцией `--allow-unrelated-histories`:

```bash
git pull origin main --allow-unrelated-histories
```

Это сделает merge (слияние) историй коммитов и получится единая история, не забудь сделать push после этого!

(Не рекомендуем) Другой вариант - сделать force push, это перепишет удаленную историю локальной. История удаленных коммитов при это будет удалена.

```bash
git push -u origin main -f
```

Еще раз подсветим, **force push не рекомендуем**, так как любое изменение истории коммитов нарушает связность.

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

