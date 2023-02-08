# Я, ROS и мой первый пакет (не из пятерочки)

## Содержание

- [Содержание](#содержание)
- [Что нужно, чтобы начать?](#что-нужно-чтобы-начать)
- [Основная информация](#основная-информация)
  - [Подготовка рабочего пространства (workspace -\> ws)](#подготовка-рабочего-пространства-workspace---ws)
    - [Добавить подключение рабочего пространства в сессию](#добавить-подключение-рабочего-пространства-в-сессию)
  - [Проверка установки](#проверка-установки)
  - [Пространство готово - теперь создаем пакет](#пространство-готово---теперь-создаем-пакет)
  - [Сборка пакета](#сборка-пакета)
  - [Разместим новый пакет в репозиторий на GitHub](#разместим-новый-пакет-в-репозиторий-на-github)
  - [Первый узел](#первый-узел)
  - [Залить наработки в конце работы](#залить-наработки-в-конце-работы)
- [Что нужно сделать](#что-нужно-сделать)
- [Вопросики](#вопросики)
- [С чем познакомились?](#с-чем-познакомились)
- [Полезные ресурсы](#полезные-ресурсы)

## Что нужно, чтобы начать?

- Установленная [Ubuntu 20.04 LTS](https://releases.ubuntu.com/20.04/)
- Установленный [ROS Noetic](https://wiki.ros.org/noetic/Installation/Ubuntu)
- Установленный пакет `git`
- Созданный аккаунт на GitHub
- Готовность к изучению нового и интересного =)

## Основная информация

Работы с ROS начинается с того, что вам нужно понять концепцию организации кода и файлов. Главный аспект стоится на том, что код хранится в **пакетах**. **Пакет** - это набор файлов, объединенных единым смыслом или задачей. Например, пакет драйвера для камеры, пакет для подлключения джойстика, пакет колесного робота и т.д.

Пакетов в интернете очень много, но чтобы работать и разрабатывать свои пакеты, нужно создать **рабочее пространство для разработки**. Этим и займемся!

### Подготовка рабочего пространства (workspace -> ws)

> Для работы с собственным рабочим пространство должно быть подключено системное. Как это сделать есть в разделе [FAQ](../FAQ.md), но также все тонкости вы можете прояснить на практике!

Так как ROS видит пакеты только в рабочем пространстве, то необходимо создать собственный ws для работы с ним. Можно подглядеть в [исходную инфу на офф сайте](http://wiki.ros.org/catkin/Tutorials/create_a_workspace), а можно сделать следующие шаги:

Во-первых, создаем папочку, которая будет нашим ws. Назовем ее `catkin_ws`:

```bash
mkdir -p ~/catkin_ws/src
```

> Опция `-p` указывает, что нужно создать полный путь, даже если предстоящих папок нет.

В нашем случае папки `catkin_ws` нет, так что создаем весь путь с опцией `-p`. `src` папку внутри надо создавать, чтобы потом туда размещать пакеты. Мы делаем два действия одной командой =)

После этого переходим в папку и вызываем утилиту сборки (так можно в первый раз инициализировать ws):

```bash
cd ~/catkin_ws
catkin_make
```

После этого рабочее пространство готово и осталось добавить в файл `~/.bashrc` строку для автоматической настройки ROS на подключение нашего ws:

#### Добавить подключение рабочего пространства в сессию

```bash
. ~/catkin_ws/devel/setup.bash
```

Сделать это можно как и с системным:

```bash
echo ". ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

или открыть файл `~/.bashrc` и прописать ручками.

> Система ROS может видеть только одно рабочее простанство, создание нескольких пространств и добавление в `rc`-файл не даст результата

### Проверка установки

После перезапуска терминалов (для настройки сессий) можно проверить установку всех ws

```bash
echo $ROS_PACKAGE_PATH
```

В появившемся результате должен находиться путь до созданного рабочего простанства, а также путь до системного рабочего пространства. То есть, строка должна содержать результат похожий на 

```bash
/home/user/catkin_ws/src:/opt/ros/noetic/share
```

> Часто переменные окружения содержат список путей. А разделителем списка является символ '`:`'.

Как видно, первый путь является путем до нового ws, а второй - до системного.

### Пространство готово - теперь создаем пакет

Вся экосистема ROS основывается на концепции пакетов, которые включают различные компоненты. В этом топике мы знакомимся с возможностями создания пакетов, их редактирования, а также c инструментом сборки, который будет использоваться далее. Можно также подглядеть на [офф страницу =)](http://wiki.ros.org/ROS/Tutorials/CreatingPackage).

Для начала перейдем в наш ws, в директорию `src`:

```bash
cd ~/catkin_ws/src
```

Для создания пакета используется команда:

```bash
catkin_create_pkg [pkg_name] [dep1 dep2 ...]
```

В данной команде первым аргументом передается имя нового пакета, после перечисляются зависимости данного пакета.
Для начала создадим пакет и добавим поддержку библиотек python (rospy):

```bash
catkin_create_pkg study_pkg rospy
```

Далее наблюдаем созданную папку `study_pkg` и два главных файла внутри: `CMakeLists.txt` и `package.xml`

> Создайте пакет `super_name_study_pkg` и просмотрите файлы `CMakeLists.txt` и `package.xml`. Вместо "name" поставьте свое имя или фамилиюи (или как-то по-другому сделайте название пакета уникальным). Не забудьте, что на компьютере могут работать другие люди и ставить свои пакеты!
> Мы дальше будем использовать `study_pkg`, но вы то знаете, что это 

В содержании `package.xml` можно выделить основные блоки:  

Заголовок, в нем содержится основная инфа о пакете

```xml
<?xml version="1.0"?>
<package format="2">
  <name>study_pkg</name>
  <version>0.0.0</version>
  <description>The study_pkg package</description>
  <maintainer email="user@todo.todo">user</maintainer>
  <license>TODO</license>
```

Зависимости (инструмент сборки, сборка, экспорт, runtime (exec) - выполнение)

```xml
  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>roscpp</build_depend>
  <build_depend>rospy</build_depend>
  <build_export_depend>roscpp</build_export_depend>
  <build_export_depend>rospy</build_export_depend>
  <exec_depend>roscpp</exec_depend>
  <exec_depend>rospy</exec_depend>
```

Остальное можно также наблюдать внутри комметариев формата xml. Там также приведены некоторые описания строк и блоков.

Внутри `CMakeLists.txt` можно также видеть много закомментированных блоков, но в основном можно вытащить базовые куски на момент инициализации:  

Определение минимальной версии сборки `cmake` и название проекта

```cmake
cmake_minimum_required(VERSION 3.0.2)
project(study_pkg)
```

Поиск и подключение зависимостей

```cmake
find_package(catkin REQUIRED COMPONENTS
  rospy
)
```

Также создается базовая папка `src` для файлов исходных текстов.  

### Сборка пакета

А теперь сделаем пакет видимым системе ROS - для этого надо просто вызвать сборку в корне рабочего пространства. Перейдите в папку рабочего пространства:

```bash
cd ~/catkin_ws
```

и выполните команду сборки:

```bash
catkin_make
```

После успешного выполнения сборки убедимся в том, что экосистема ROS видит наш пакет! Перезапустите терминал (или вызовите `source ~/.bashrc`) и проверяйте список пакетов в системе:

```bash
rospack list
```

В списке должна быть строка:

```bash
...
study_pkg /home/user/catkin_ws/src/study_pkg
...
```

С помощью команды `rospack help` можно получить информацию об утилите и ее аргументах.

> Вообще рекомендую не забывать этот аргумент `help` (или опцию `-h`), так как он применим ко всем утилитам ROS экосистемы.

### Разместим новый пакет в репозиторий на GitHub

Для начала необходимо создать пустой репозиторий на github.com в своем аккаунте. Если чего-то не хватает - разбираемся вместе или по инструкциям из веба, их благо достаточно.

> Можно глянуть как [инициализировать git с дальнейшими действиями](https://help.github.com/articles/adding-an-existing-project-to-github-using-the-command-line/).

После каждой команды `git *` рекомендую выполнять `git status`, чтобы видеть результат действий

Теперь перейдем в папку пакета `study_pkg`:

```bash
roscd study_pkg
```

Инициализируем папку как локальный git репозиторий:

```bash
git init --initial-branch=main
```

Результат

```console
Initialized empty Git repository in /home/user/catkin_ws/src/study_pkg/.git/
```

Посмотрите состояние свежего репозитория командой:

```bash
git status
```

Теперь надо привязать локальный репозиторий к удаленному:

```bash
git remote add origin [Repo URL]
```

Repo URL - путь удаленного репозитория, берется со страницы репозитория из зеленой кнопки "Clone or download".
Например, для репозитория курса была использована следующая команда:

```bash
git remote add origin https://github.com/user/super_user_study_pkg.git
```

Как видно, в URL содержится имя владельца репозитория и его названия.

Настроим, чтобы ветка `main` локального репозитория следила за веткой `main` удаленного репозитория, для этого стянем все данные с ветки `main` удаленного репозитория (который при соединении в предыдущей команде мы назвали `origin`):

```bash
git pull origin main
```

> На удаленном репозитории ветка `main` называется `origin/main`

После этого можно учесть (индексировать) новые файлы пакета:

```bash
git add -A
```

> Опция `-A` добавляет все неучтенные файлы. Вместо нее можно просто перечислить файлы, которые необходимо добавить к учету в коммитах.

И сделать коммит в локальном репо:

```bash
git commit -am "First package commit"
```

> Опция `-a` делает `git add` ко всем изменениям учтенных (индексированных) файлов - упрощает нам задачу.
> Опция `-m` устанавливает коммент к коммиту. Коммент пишется после опции.

<details>
  <summary>Если гит не хочет делать коммит и пишет просьбу указать "Ты кто такой?"</summary>

```console
*** Please tell me who you are.

Run

  git config --global user.email "you@example.com"
  git config --global user.name "Your Name"

to set your account's default identity.
Omit --global to set the identity only in this repository.
```

значит надо настроить данные о пользователе:

```bash
git config user.email "user@mail.ru"
git config user.name "User User"
```

</details>

После остается только закинуть все сделанные коммиты (а их пока один штука) на удаленку:

```bash
git push --set-upstream origin main
```

> Для выполнения данной команды может потребоваться ввод имя пользователя и пароль.

И все! Можно смотреть на результаты на сайте!

### Первый узел

Мы же тут не просто пакеты создавать собрались? Пора прогать! Да, это будет Hello World, но не просто программа, а целый узел!

Суть в том, что все программы в ROS называются узлами. Это как в графе, где узлы графа соединены ребрами. Так вот мы в будущем узнаем, что узлы действительно соединыются в ROS, но в качестве ребер выступают каналы связи!

Но сегодня наша цель - написать первый просто узел =) Начнем!

В Python есть разделение `.py` файлов на модули исходных кодов и модули исполняемых программ. Отличие простое - первые нужны для хранения логики программ, а вторые для описания процесса запуска программ.

Для исполняемых программ внутри пакета сделайте директорию `scripts` и в ней создайте файл `first_node.py`:

```bash
roscd study_pkg
mkdir scripts
touch scripts/first_node.py
```

После этого открываем любимый редактор и пишем в файле простой кусочек кода:

```python
def main():
    print(f"Hello ROS World!")


if __name__ == "__main__":
    main()
```

Отлично, пробуем стартануть с помощью команды `rosrun`, которой надо передать имя пакета и имя скрипта дл запуска!

```bash
rosrun study_pkg first_node.py
```

Упс, ошибочка:

```console
[rosrun] Couldn't find executable named first_node below /home/user/catkin_ws/src/study_pkg
```

Что не так? ROS пытается запустить скрипт, но ему не хватает кое-чего!

Когда мы написали код в файле, то это просто текстовый файл, а чтобы запустить этого код, нужно сделать две вещи:

- Дать файлу права на исполнение;
- Прописать интерпретатор, с помощью которого будет запускаться скрипт

Права даются утилитой `chmod`:

```bash
chmod +x scripts/first_node.py
```

А для указания интерпертатор в начало файла первой строкой пропиши следующую строку:

```python
#!/usr/bin/env python3
```

> Эта строка указывает, чтобы запуск файла производился через интерпретатор `python3`.

Ну что, действия сделаны, пора проверить, заработает ли?

```bash
rosrun study_pkg first_node.py
```

И вот результат!

```console
Hello ROS World!
```

Отлично! Получилось, первый пакет с первым узлом готов! Отличное начало!

### Залить наработки в конце работы

После окончания работы на кодом нужно обязательно заливать наработки в репозиторий, чтобы их не потерять и всегда можно было стянуть последнюю версию даже с другой машины!

Нужно добавить новые файлы к индексу:

```bash
git add scripts/first_node.py
```

После этого сделать комит:

```bash
git commit -am "My first node in ROS, hooray!"
```

> Над комментарием комита можно еще подумать =)

И отправить новые комиты на репозиторий:

```bash
git push
```

> Да-да, теперь без `--set-upstream` опции, так как этот репо уже связан с удаленным

Молодец!

## Что нужно сделать

Вам нужно создать рабочее пространство, пакет (`super_<name>_study_pkg`) и репозиторий. После этого связать пакет и репозиторий, чтобы загружать свои наработки в него. И наконец написать простейший узел, который будет выводить в терминале нынешнее время (найдите в интернете, как это сделать на Python) с периодом 5 сек.

После заливания обновлений пакета проверьте на GitHub, что ваш пакет создан и код там последней версии!

## Вопросики

> Не на все из вопросов есть ответы в материале, но если вы можете ответить на них, то будьте уверены - вы знаете материал хорошо!

- Какие папки создались внутри рабочего пространства `catkin_ws`? Для чего каждая нужна?
- Какие файлы есть внутри нового пакета? Зачем каждый нужен?
- Какие есть команды у утилиты rospack? Что делает команда find?
- Что необходимо сделать для запуска узла на Python?
- В чем отличие `git` и GitHub? Как они связаны? Для чего они используются?

## С чем познакомились?

- Концепция пакета и рабочего пространства
- `catkin_make`
- `catkin_create_pkg`
- `rospack`
- `git`
- `roscd`
- `rosrun`

## Полезные ресурсы

- [Официальная страничка](http://wiki.ros.org/ROS/Tutorials)