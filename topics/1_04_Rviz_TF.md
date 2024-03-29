# Переходим к лидару

## Содержание

- [Содержание](#содержание)
- [Всё начинается с лидара](#всё-начинается-с-лидара)
- [Попытка отобразить исходную информацию из лидара - не пытка](#попытка-отобразить-исходную-информацию-из-лидара---не-пытка)
- [TF 101](#tf-101)
- [TF - всё относительно](#tf---всё-относительно)
- [Бооольше инструментов](#бооольше-инструментов)
  - [rqt\_tf\_tree](#rqt_tf_tree)
  - [tf\_echo](#tf_echo)
- [TF из Gazebo](#tf-из-gazebo)
- [Настройка Rviz](#настройка-rviz)
- [Чему научились?](#чему-научились)
- [Задание](#задание)
- [Вопросики](#вопросики)
- [Ресурсы](#ресурсы)
  - [Стандарты](#стандарты)


## Всё начинается с лидара

Мы с тобой уже научились запускать симулятор и организовывать запуск необходимых узлов и других launch как нам надо.

А ещё и аргументы добавлять, чтобы делать удобное управление запуском!

Это уже хорошие навыки, но давай теперь начнём осваивать каждую часть набора программ (узлов), которые позволяют роботу двигаться к цели, учитывая препятствия!

Начнём мы с лидара. На лекции тебе подробнее расскажу о том, что это и как работает, а мы пока будем смотреть, что нам делать, если у нас на роботе уже есть лидар.

## Попытка отобразить исходную информацию из лидара - не пытка

Давай запустим наш launch, который стартует симулятор:

```bash
roslaunch super_robot_package turtlebot3_world.launch
```

В симуляторе мы уже видели, что лидар у нас на вафельке стоит:

<p align="center">
<img src=../assets/01_waffle_lidar.png />
</p>

Этот лидар выдаёт информацию в виде точек - расстояний до препятствий вокруг на плоскости сканирования. Как же нам её увидеть в сыром виде?

Для этого нам, как обычно, поможет узел, который отображает информацию из робота (как робот видит окружение и какая в нём есть информация) - Rviz.

```bash
rviz
# или rosrun rviz rviz
```

Запускаем её и уже видим странную проблему:

<p align="center">
<img src=../assets/01_04_rviz_issue.png />
</p>

Как мы тогда до этого работали с Rviz? Всё же было хорошо. Странно очень, но давай попробуем добавить информацию от лидара для отображения: `Add` -> `By topic` -> `/scan` -> `LaserScan`.

<p align="center">
<img src=../assets/01_04_scan_issue.png />
</p>

Ещё какая-то проблема! Ну всё, дальше двигаться нельзя - ничего не показывается и есть ошибки.

Но давай посмотрим на первую ошибку, в "Global Status" ошибка "Unknown frame map". Это, обычно, происходит, если в "Global Options" -> "Fixed Frame" выбран несуществующий **фрейм**.

Что такое **фрейм** - чуть позже разберём, а пока выбери в Fixed Frame, например, `base_footprint`.

О, в "Global Status" теперь "OK"! Это уже результат! Но в LaserScan новая проблема:

<p align="center">
<img src=../assets/01_04_scan_new_issue.png />
</p>

Таакс, ну тут уже без новых знаний не обойдёмся...

Ну, поехали, узнаем, про какие *фреймы* идёт речь и почему они так нужны для Rviz!

## TF 101

TF - это очень удобная система в ROS, которая позволяет работать с системами координат (СК) в пространстве. Принято считать, что TF - это сокращение от "transform". То есть, эта система организует трансформацию между СК.

Например, в руке есть плечо, локоть и кисть, у каждой части есть своё положение в пространстве. Мы знаем, что расстояние от плеча до локтя не меняется, как и расстояние от локтя до кисти. Получается, у нас есть СК в плече, локте и кисти. Между плечом и локтём 30 см и между кистью и локтём 30 см. Вот так мы описали относительное положение между СК!

Сначала прямую руку прижмём к туловищу, рука идёт вниз. Допустим, ось Z идет наверх, X - прямо, Y - влево. Если у плеча `X=0, Y=0, Z=0`, то у локтя будет порядка -0.3 м по Z, а у кисти около -0.6 м. Всё остальное по нулям (`X=0, Y=0`).

Теперь поднимем прямую руку ровно наверх, Z локтя и кисти будут иметь те же значения, но положительные. Тут всё просто, так?

А теперь фокус, плечо сгибаем около 65 градусов, локоть на 35 градусов, какие координаты будут у локтя и кисти?

Если ты очень хорошо помнишь тригонометрию и принципы поворотов систем координат, то тебе не составит труда посчитать.

Дать время подумать? =)

Эта математика расчёта уже давно реализована и как раз TF позволяет нам не беспокоиться о том, что и как надо делать в плане расчётов!

Мы описываем отношения между системами координат (СК) и после этого, построив цепочку из СК, можем относительно одной получать информацию об остальных!

Это очень удобно! Но к чему слова? Нам же надо разобраться с проблемой отображения данных в Rviz, что он хочет?

Во-первых, термин **фрейм** в Rviz - это просто система координат. Тут всё просто.

Во-вторых, что за фрейм `base_scan` он хочет? Тут сложнее, но интереснее!

Лидар выдает нам 360 точек (по точке на каждый градус окружности вокруг) - это просто расстояния от лидара до препятствия, числа.

Но чтобы их отобразить, Rviz надо знать, относительно чего рисовать. Ведь лидар где-то на нашем роботе располагается, а значит надо сообщить rviz, где находится лидар на роботе, чтобы относительно этой точки нарисовать точки препятствий вокруг.

Для этого мы сообщим системе, как располагается `base_link` относительно `base_footprint`. `base_footprint` в данном случае выступает центром робота, спроецированном на пол (высота = 0).

Чтобы сообщить о том, насколько высоко располагается лидар относительно пола, добавим запуск узла, который создаёт **статический TF** между фреймами. Создадим новый launch c названием `turtlebot3_tf.launch`:

```xml
<node pkg="tf" type="static_transform_publisher" name="base_footprint_2_base_link" args="0 0 0.3 0 0 0 base_footprint base_scan 100" />
```

> Не забудь, что `<launch>` тэг должен обрамлять весь файл, то есть добавлять надо внутрь него!

Тут мы немного цепляем запуск узлов в launch-файла. Чтобы опубликовать статический TF (сообщить расположение `base_scan` относительно `base_footprint`), нам нужно запустить узел [static_transform_publisher](http://wiki.ros.org/tf#static_transform_publisher) из пакета [tf](https://wiki.ros.org/tf).

Чтобы это сделать мы прописываем в атрибутах тэга `<node>`:

- `pkg` - название пакета, из которого запускаем,
- `type` - название узла, который в пакете надо запустить,
- `name` - как узел будет называться в системе,
- `args` - аргументы узла.

Аргументы у static_transform_publisher узла следующие `x y z yaw pitch roll frame_id child_frame_id period_in_ms`. Разберём:

- `x, y, z` - линейное расположение `child_frame_id` относительно `frame_id`,
- `yaw pitch roll` - угловое расположение (поворот) `child_frame_id` относительно `frame_id`,
- `frame_id child_frame_id` - имена фрейма, между которыми создаём TF,
- `period_in_ms` - частота публикации, обычно ставится 100 или 1000, и норм.

Такс, ну мы много тут всего написали, но главное, что нам теперь надо запустить новый launch и посмотреть, помогло ли это отобразить данные из лидара:

```bash
roslaunch super_robot_package turtlebot3_tf.launch
```

Посмотрим Rviz и увидим, что данные отобразились! Отлично!

<p align="center">
<img src=../assets/01_04_rviz_scan_success.png />
</p>

> Для лучшего отображения точек в LaserScan->"Size (m)" можно поменять размер на 0.03, например

Мы смогли починить недостающую информацию для отображения данных с лидара! В системе не хватало информации о том, как расположен `base_scan` фрейм относительно `base_footprint`!

> :muscle: Попробуй поменять значения `x, y, z, yaw pitch roll` в аргументах TF узла и перезапускать после изменений launch с этим узлом. Посмотри, как это влияет на отображение.

## TF - всё относительно

Результат есть, круто, но не складывается ли ощущение, что мы просто угадали и всё сделали правильно? Ведь, если разбираешься с каким-нибудь инструментом, всегда нужно чётко осознавать, что, как и почему делается!

Во-первых, важно сейчас понять, что СК не существует сама по себе! Именно поэтому мы описываем не расположения фреймов самих по себе, а именно относительное расположение между ними. Так, чтобы отобразить, мы задаём базовый фрейм (Fixed Frame), относительно которого всё и рисуется.

Во-вторых, давай отобразим фреймы в Rviz, чтобы увидеть, как они расположены! `Add` -> `By display type` -> `TF`:

<p align="center">
<img src=../assets/01_04_rviz_tf_frames.png />
</p>

Так ведь удобнее, правда?

> :muscle: А теперь попробуй покататься с помощью teleop launch в пакете и скажи, какие из TF (TF - это соотношение между фреймами) являются **статическими**, а каким **динамическими**?

Мы пока не объясняли эти термины, но думаю, тут уже понятно, в чём разница =)

**Статический TF** (`base_footprint` -> `base_scan`) - положение между фреймами не меняются во времени, никогда!
**Динамический TF** (`odom` -> `base_footprint`) - положение между фреймами может меняться во времени.

Почему не бывает динамического фрейма? Всё просто, например, фрейм `base_footprint` относится как к динамическому TF, так и к статическому TF. Нет однозначности в этой характеристике.

Так, прекрасно, мы освоили ещё немного терминов и смогли отрисовать TF в Rviz, чтобы проще было ориентироваться! Отличный результат!

## Бооольше инструментов

Сейчас всё идет по плану, но часто в ходе разработки или отладки системы происходят проблемы и всплывают баги настройки, ой как часто...

Более того, мы смогли отобразить TF в Rviz, но в нём не понятно, как увидеть соотношение между фреймами и более того, численных значений преобразований (TF) между фреймами!

Значит, сейчас пока мы не можем в полной мере понять, что происходит внутри системы - погнали осваивать инструменты!

### rqt_tf_tree

Первый инструмент поможет нам понять, как соотносятся между собой фреймы и кто является источником информации о TF - `rqt_tf_tree`!

Просто стартуем:

```bash
rosrun rqt_tf_tree rqt_tf_tree
```

И вот мы видим простой и приятный интерфейс:

<p align="center">
<img src=../assets/01_04_rqt_tf_tree.png />
</p>

В этом интерфейсе мы видим, как фреймы зависимы!

> Важно, правильное дерево имеет формат **дерева**, то есть у нижестоящих фреймов не может быть два родителя или стрелочки не могут идти наверх!

> :muscle: Попробуй заменить в `turtlebot3_tf.launch` фрейм `base_footprint` на `base_link` и перезапусти. Отобрази `rqt_tf_tree`, видишь проблему? Ошибка в названии ведёт к созданию двух деревьев, такого тоже быть не должно! Верни обратно, как было =)

С помощью этого инструмента можно понять, как взаимосвязаны фреймы и есть ли ошибки в построении дерева TF, так как на этом достаточно часто происходят ошибки.

### tf_echo

Другой инструмент позволит нам понять численные характеристики той или иной TF - `tf_echo`.

```bash
rosrun tf tf_echo base_footprint base_scan
```

И вот результат:

```
At time 6673.459
- Translation: [0.000, 0.000, 0.300]
- Rotation: in Quaternion [0.000, 0.000, 0.000, 1.000]
            in RPY (radian) [0.000, -0.000, 0.000]
            in RPY (degree) [0.000, -0.000, 0.000]
At time 6674.462
- Translation: [0.000, 0.000, 0.300]
- Rotation: in Quaternion [0.000, 0.000, 0.000, 1.000]
            in RPY (radian) [0.000, -0.000, 0.000]
            in RPY (degree) [0.000, -0.000, 0.000]
At time 6675.463
- Translation: [0.000, 0.000, 0.300]
- Rotation: in Quaternion [0.000, 0.000, 0.000, 1.000]
            in RPY (radian) [0.000, -0.000, 0.000]
            in RPY (degree) [0.000, -0.000, 0.000]
```

Как видно, статическая TF не меняется - логично! =)

> :muscle: Отобрази TF "odom -> base_footprint". Посмотри значения между ними.

> :muscle: Разберись, как в tf_echo задать частоту отображения данных

Отлично! Вот у нас и получилось понять соотношение между фреймами, а также получить информацию о конкретном TF в численном виде, а не "на глаз"!

## TF из Gazebo

Важной хитростью работы с симулятором является то, что модель в симуляторе уже описана с учётом расположения датчиков и деталей между собой. Поэтому, в Gazebo есть узел, который может опубликовать все статические TF, описывающие робота...

Мы не скрывали это от вас, просто важно с некоторыми вещами разобраться поподробнее!

Давай отключим все launch и создадим файл, в который добавим:

- Запуск world.launch
- Запуск публикации информации о расположении частей робота (TF)

    ```xml
    <node pkg="robot_state_publisher" type="robot_state_publisher" name="robot_state_publisher">
        <param name="publish_frequency" type="double" value="50.0" />
    </node>
    ```

- Запуск rviz

    ```xml
    <node pkg="rviz" type="rviz" name="rviz" />
    ```

<details>
  <summary>Подсказка</summary>
      
    <launch>
        <include file="$(find super_robot_package)/launch/turtlebot3_world.launch" />
        
        <node pkg="robot_state_publisher" type="robot_state_publisher" name="robot_state_publisher">
            <param name="publish_frequency" type="double" value="50.0" />
        </node>

        <node pkg="rviz" type="rviz" name="rviz" />
    </launch>

</details>

А теперь стартуем его!

```bash
roslaunch super_robot_package turtlebot3_sim_start.launch
```

Ах да, Rviz же после перезапуска сбивается...

Ну, настрой снова TF, LaserScan и Fixed Frame.

Хм, а фреймвов стало гораздо больше, как же увидеть всё в этой мешанине? Не сложно, просто открой параметры TF и настрой те TF, который хочется видеть (`odom`, `base_footprint`, `base_scan`).

<p align="center">
<img src=../assets/01_04_rviz_tf_setup.png />
</p>

Отлично, так значит, можно сразу получить описание робота без необходимости прописывать каждый TF руками?

Не совсем, в этом случае расположения фреймов описаны в симуляторе, а значит в нём есть вся необходимая информация!

Если у тебя робот без модели в симуляторе, то придётся описывать ручками, но это не так страшно, как кажется! Берём рулетку и в бой =)

На деле у нас появился launch-файл, который стартует все необходимое в симуляторе, теперь можно идти дальше и изучать другие аспекты работы системы ROS на роботе!

> Но погоди отключать настроенный Rviz!

## Настройка Rviz

Такс, не спешим, у нас осталась одна нерешённая проблемка...

Мы тут перезапустили Rviz и приходится каждый раз настраивать! Это же уйма времени будет потрачена! Не годится.

Давай посмотрим `rviz -h` и найдём ооочень полезную строку:

```bash
-d [ --display-config ] arg A display config file (.rviz) to load
```

То есть, в rviz есть опция, которая позволяет подгрузить конфигурацию отображения из файла? Хм, это может быть полезно!

Закончи настройку отображения информации как тебе хочется (может точки больше сделаешь?) и жмём кнопку `File`->"`Save Config As`" (сверху слева):

<p align="center">
<img src=../assets/01_04_rviz_file_button_pos.png />
</p>

> Не нажимай "Save Config" или сочетание "Ctrl+S", так как это сохранит нынешнее отображение как настройки по-умолчанию!

В окне сохранения пройди в папку пакета и там создай папку `rviz` и туда сохрани в файл под названием `sim_initial.rviz`.

А тепееерь, магия, погнали в `turtlebot3_sim_start.launch` и там к запуску Rviz добавь `args="-d $(find super_robot_package)/rviz/sim_initial.rviz"`. Это по сути путь до нашей конфигурации в пакете.

Добавил? Отлично, перезапускай!

Всё на месте! Теперь тебе не придется каждый раз настраивать Rviz! А ещё тут есть приятный сорприз, если поменять что-либо в отображении и нажать "Ctrl+S", то это сохранится и в следующем запуске будет тут же! Шикарно!

А всего-то поняли, что есть проблема и в два счёта решили ее, проверив help!

> ! Практика показывает, что создание rviz конфигураций под каждую задачу является удобным способом хранения. Не пытайтесь держать одну конфигурацию под все задачи, например, "проверка работы модели в симуляторе", "отображение данных с лидара", "отображения инфы от планировщика" и т.д.

## Чему научились?

- Настройка Rviz
- Ознакомились с понятием TF и её удобствами
- Смогли сделать статическую публикацию TF в launch
- Научились отображать TF в Rviz
- Освоили инструменты `rqt_tf_tree` и `tf_echo`
- Узел `robot_state_publisher` для публикации информации о TF робота из описания в симуляторе

## Задание

Не забудь залить все свои обновления в Git и на сервер!

- Попробуй отобразить другую информацию в Rviz - теперь ты это умеешь! Посмотри, что происходит с остальными фреймами при движении.
- Посмотри дерево TF после перехода на `robot_state_publisher`.
- Посмотри значения TF между разными фреймами, все ли значения понятны?


## Вопросики

- Почему мы "объясняли" системе, как расположен `base_link` относительно именно `base_footprint`? Почему не относительно `odom`?
- В чём разница статических и динамических TF?
- Почему фрейм нельзя характеризовать статическим/динамическим?
- Как сохранить настройку Rviz? Как её восстановить?
- \* Что такое кватернион?

## Ресурсы

- [Хорошая статья с примером описания основных фреймов робота](http://wiki.ros.org/hector_slam/Tutorials/SettingUpForYourRobot)

### Стандарты

Вообще, это просто два основных стандарта касательно фреймов и TF, но мы очень рекомендуем их почитать:

- [REP-105](https://www.ros.org/reps/rep-0105.html) - описание основных фреймов;
- [REP-103](https://www.ros.org/reps/rep-0103.html) - описание основных единиц измерения и координатных преобразований.

