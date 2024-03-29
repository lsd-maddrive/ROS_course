# Вопросы организации пакета (и не только…)

> На самом деле большая часть того, что будет рассмотрено в данной теме уже применялась в предыдущих уроках.
> Но вот было это все как-то не осмысленно: создание какого-то workspace, какие-то пакеты, git… и все в этом духе. 
Будем по порядку разбираться!

## И начнем не посредственно с того, как у нас организовано построение и хранение программ в ROS.
> Общая структура всего того, что мы можем сами создать в ROS, будет выглядеть, как `Workspace -> Packages -> Nodes/Messages/Models/Launches`
> А теперь расшифруем, что же это все значит:
### Workspace
Workspace в общем и целом является классическим для многих сред разработки рабочим пространством, 
котором хранятся исходные файлы программ, сведения о сборке, `CMake – файлы`, объекты и исполняемые программы.
Для создания workspace используется команда 
```bash
catkin_init_workspace
```

### Packages 
Пакет является своего рода проектом, в котором могут быть объединены разные подпрограммы и прочие файлы, необходимые для их работы.
Для создания пакета используется команда 
```bash
catkin_create_pkg
```

### Nodes
`Node` – является по своей сути основной рабочей единицей в ROS. 
Это объект или подпрограмма, которая выполняет запрограммированные в ней действия и производит обмен информацией с другими такими же подпрограммами. 

### Messages
Данный факт, пока может быть, был не очень очевиден, но все подпрограммы в ROS, 
как правило, обмениваются между собой информацией, например, мы выводили в `RVIZ` картинку с картой или траекторию движения. 
`RVIZ` по сути своей также является узлом, который принимал от других узлов (пока непонятно каких) информацию и выводил ее на экран. 
Организацию связи между узлами нам обеспечивают потоки данных, в ROS они называются `topic` (топики).
Но как там понять, что там в топиках заданные лежат, или что в эти топики отправить? 
Для этого существует такая структура, как `Message`. Наиболее близкой аналогией будет хорошо всем известная структура из Си. 
То есть `Message` есть некий тип данных, в котором могут лежать другие простые или сложные типы данных. 
В ROS существует множество готовых типов `Message`, но есть и возможность создавать свои. 

### Models
Под моделями в данном случае следует понимать уже увиденные нами на практике модели роботов в `Gazebo` или `RVIZ`. 
Их также можно создавать самостоятельно или использовать уже существующие. И обо всем этом мы поговорим немного позже.

### Launches
Можно было заметить, что когда мы запускали наш `TurtleBot3`, мы обращались к неким сущностям названия, которых оканчивались на `*.launch`, 
а для запуска использовалась команда `roslaunch`.
При этом на вводном уроке для запуска `turtlesim` использовали команду `rosrun`, а в именах расширение указано не было. 
И это все непросто так и неслучайно. 
Как уже говорилось ранее основной исполняемой единицей в ROS можно считать ноду, и для ее запуска будет использована конструкция вида 
```bash
rosrun <package_name> <node_name>
```
Но что делать, если мы хотим сразу запускать несколько нод, да еще и передавать им какие-то параметры? 
Конечно можно использовать несколько окон терминалов и все писать руками, но каждый раз это делать долго и неудобно. 
Для этого придумали скрипты для запуска – launch-файлы. 
Они, как правило, хранятся в том же пакете, что и запускаемы ноды, но ничего не мешает обращаться и к нодам из других пакетов. 
Для запуска используется команда 
```bash
roslaunch <package_name> <launch_file_name.launch>
```
Важно помнить, что внутри пакета все launch-файлы должны храниться в папке `launch`.
Подобная проблема с большим количеством повторяющихся действий преследует и `RVIZ`. 
Довольно утомительно каждый раз настраивать отображение нужной нам информации. 
Поэтому конфигурацию можно сохранять в файлах конфигурации `rviz`, и при необходимости вызывать их из launch-файла. Конфигурации `rviz` обычно хранят в пакете в папке rviz.

### Сборка пакета
Для сборки и компиляции всех файлов рабочего пространства используется команда 
```bash
catkin_make
```
При вызове данной команды в терминале появятся служебные сообщения о ходе сборки и также предупреждения и ошибки компиляции.
Если все хорошо, сборка прошла успешно, можно попробовать запустить наш проект. 

### Коллаборация, командная работа и просто удобное хранение
И для всего этого сейчас активно используются системы контроля версий, например, `github`.
- Сначала регистрируемся на сайте github.
- Создаем репозиторий.
- Идем в терминал. Здесь мы покдлючим SSH-ключи, чтобы подключаться с компьютера к своему репозиторию. В процессе гененрации и записи на некоторых этапах будут запрашиваться логины, пароли, имена файлом и т.д., рекомендуется это все пропускать, чтобы работать с дефолтными именами.
- Генерируем ssh-ключ 
``` bash
ssh-keygen -t ed25519 -C "your_mail@example.com"
```
- Добавляем ключ в `ssh-agent`
```bash
ssh-add ~/.ssh/your_key_name
```
- В Домашнем каталоге должна появиться папка `.ssh`. Заходим в нее и ищем файл id_ed25519.pub. Открываем его и копируем содержимое.
- Переходим обратно на сайт github. Переходим в раздел `settings`, а в нем во вкладку `SSH and GPG keys`. Жмем `New SSH key`.
- В новом окне придумываем имя ключа (можно то же, что и на компьютере - ed25519) и копируем во второе поле ранее скопированный из файла ключ.
- Сохраняем и выходим из настроек.
- Если на компьютере еще не установлен Git, делаем команду 
```bash
sudo apt install git
git config --global user.name <your name>
git config --global user.email <your email>
```
- Заходим в папку src нашего рабочего пространства и делаем команду 
```bash
git clone <address>
```
- Адрес нашего нового репозитория нужно скопировать, зайдя на страничку репозитория, кнопка `Code->SSH->Copy`.
- Итак, в папке src должна появиться новая папка с именем, как у репозитория.
- Перейдя в нее создадим файл `main.py` и в нем напишем программу `Hello world` на языке python:
```python
print('Hello world')
```
- Сохраним изменения произошедшие с в нашей папке = локальном репозитории командой
```bash
git commit -am 'add HW'
```
- И зальем это все обратно на удаленный репозиторий на сайте github.
```bash
git push 
```
- Если вы внесете с какой-то другой машины изменения в проект на сайте, а потом захотите эти изменения получить на данном компьютере, то можно использовать команду
```bash
git pull
```
## Задание:
1)	Создать репозиторий для своих проектов
2)	Создать на рабочей машине workspace 
3)	Подключить git
4)	Создать тестовый пакет  
5)	Написать launch-файл для запуска turtlesim
6)	Залить workspace в репозиторий
