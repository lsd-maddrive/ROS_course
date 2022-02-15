# Связь между узлами ROS
В этой части рассмотрим основые способы передачи информации между нодами ROS.
К ним относятсы:
* topic
* service
* atcion

### Messages
Но топики (topic), сервисы (service) и действия (atcion) это лишь канал связи. Сообщения (Messages), передаваемые по топикам, строго типизированы. 
Многие типы уже заранее включены в ROS или в ходят в пакеты. И также существует возможноть создавать собственные типы сообщений. 
Ближайшей аналогией с типом сообщения может быть конструкция struct из языка Си. То есть это некий составной тип данных, которые включает в себя другие данные. 
Ниже в качестве примера приведена кострукция сообщения, содержащего измерения лидара, взятая с официального сайта ROS.
```
# Single scan from a planar laser range-finder
#
# If you have another ranging device with different behavior (e.g. a sonar
# array), please find or create a different message, since applications
# will make fairly laser-specific assumptions about this data

Header header            # timestamp in the header is the acquisition time of 
                         # the first ray in the scan.
                         #
                         # in frame frame_id, angles are measured around 
                         # the positive Z axis (counterclockwise, if Z is up)
                         # with zero angle being forward along the x axis
                         
float32 angle_min        # start angle of the scan [rad]
float32 angle_max        # end angle of the scan [rad]
float32 angle_increment  # angular distance between measurements [rad]

float32 time_increment   # time between measurements [seconds] - if your scanner
                         # is moving, this will be used in interpolating position
                         # of 3d points
float32 scan_time        # time between scans [seconds]

float32 range_min        # minimum range value [m]
float32 range_max        # maximum range value [m]

float32[] ranges         # range data [m] (Note: values < range_min or > range_max should be discarded)
float32[] intensities    # intensity data [device-specific units].  If your
                         # device does not provide intensities, please leave
                         # the array empty.
```

### Topic
Наиболее распространенным способом передачи информации является между узлами является топис (topic).
Ключевая особенность топиков состоит в том, что это однонаправленная связь. То есть существует всего два узла, один пишет сообщения, 
и один читает сообщения и обрабатывает информацию. На самом деле ROS устроен таким образом, что в один и тот же топик могут писать несколько узлов, 
как и читать из одного. Более того, нода может писать и сама же читать сообщения из топика в том случае, если это необходима (кстати такое имеет место быть,
например, в move_base). 
Следующий код наглядно демонстрирует, как создаются ноды для отправки и приема топиков. Рекомендуется тщательно изучить логику программ и запустить самостоятельно. Для добавления в пакет скриптов на python необходимо сделать следующую запись в файле CMakelist:
```
catkin_install_python(PROGRAMS
  scripts/talker.py 
  scripts/listener.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```
Далее приведен код ноды, прослушивающей сообщения:
```
import rospy   #подключение библиотеки rospy
from std_msgs.msg import String                 
#callback функция - вызывается всякий раз при возникновении отпределенного собития
#Выводит в консоль отчет о принятии узлом сообщения
def callback(data):      

    #Вывод на экран сообщения о приеме     
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)  
    
def listener():   
    #Функция инициализации узла-приемника                                                   
    rospy.init_node('listener', anonymous=True)

    #Инициализация приемника (функция принимает: 
    #название топика, тип message, функцию обработки принятого сообщения)
    rospy.Subscriber("chatter", String, callback)                   
                                                                    
    rospy.spin()

if __name__ == '__main__':
    listener()
```
И наконец отправим сообщения в эфир:
```
import rospy #подключение библиотеки rospy
from std_msgs.msg import String

#Функция инициализации узла и отправки сообщений
def talker():       
    #Инициализации передатчика (принимает: имя топика, тип message, объем очереди)                                           
    pub = rospy.Publisher('chatter', String, queue_size=10)  

    #Инициализация узла типа 'talker'   
    rospy.init_node('talker', anonymous=True)                   
    rate = rospy.Rate(10) #10hz

    #цикл повторяетс до тех пор, пока ROS запущен
    while not rospy.is_shutdown():      

        #формирование сообщения из фразы и текущего времени                        
        hello_str = rospy.get_caller_id() + "hello world %s" % rospy.get_time()    

        #вывод в консоль сообщения (для отладки)     
        rospy.loginfo(hello_str)  

        #передача сообщения по топику                              
        pub.publish(hello_str)                                  
        rate.sleep()

if __name__ == '__main__':
    talker()
```

### Service
Идея работы сервиса в некотором смысле похожа на работу топика, но имеет одно существенное отличие. При работе топиков любая нода может в него писать и из него читать информацию. Сервис же образует связь между двумя нодами: сервером и клиентом. Связь образуется по принципу: клиент формирует и оптравляет запрос, сервер запрос принимает, обрабатывает и отправлет клиенту ответ.
Протестируем работу этой арихитектуры со стандартным сервисом `std_srvs/SetBool.srv`. 
```
bool data
---
bool success 
string message 
```
В этом сервисе клиент отправляет запрос в виде логической переменной и принимает в ответ лигическую переменную и строку.
Ниже приведен код сервера:
```
import rospy
#импортирует стандартный тип сервиса SetBool - запрос, SetBoolResponse - ответ
from std_srvs.srv import SetBool, SetBoolResponse
 
def resp(mess):
 if mess.data==True:
    return  SetBoolResponse(True, 'data is true')
 else:
    return  SetBoolResponse(False, 'data is false')
 
 
rospy.init_node('service_respond')
 
service=rospy.Service('service_example',SetBool,resp)
 
rospy.spin()
```
и клиента:
```
import rospy
from std_srvs.srv import SetBool
import sys
 
rospy.init_node('use_service')
 
#wait the service to be advertised, otherwise the service use will fail
rospy.wait_for_service('service_example')
 
#setup a local proxy for the service
srv=rospy.ServiceProxy('service_example',SetBool)
 
#use the service and send it a value. In this case, I can send 1 or 0
service_example=srv(True)
 
#print the result from the service
print(service_example)
```

### Action
Action пошел еще дальше. Он перенял идею запроса-ответа у сервисов. Однако снова есть весомые отличия. Action предназначен для длительных операций. Если внимательно посмотреть на код из предыдущего примера, то можно заметить, что клиент сидит и ждет ответа от сервера. Если же серверу нужно длительное время на формирование ответа, такое поведение клиента не желательно. Поэтому в таком случае рекомендуется использовать Action. В Action существет возможноть получения промежуточных ответов о состоянии операций производимых серверов. 
Например, с помощью Action реализована связь с move_base. С помощью Action в move_base отправляется целевая точка для навигации, в процессе движения клиент получает информацию о том, как проходит движение, и в конце сервер отправляет сообщение об успешном (или нет) завершении движения.
Снова приведем простецшие примеры клиента и сервера. 
Сервер:
```
import rospy
import actionlib

 
def resp(mess):
 if mess.data==True:
    return  SetBoolResponse(True, 'data is true')
 else:
    return  SetBoolResponse(False, 'data is false')
 
 
rospy.init_node('service_respond')
 
service=rospy.Service('service_example',SetBool,resp)
 
rospy.spin()
```
и клиент:
```
import rospy
from std_srvs.srv import SetBool
import sys
 
rospy.init_node('use_service')
 
#wait the service to be advertised, otherwise the service use will fail
rospy.wait_for_service('service_example')
 
#setup a local proxy for the service
srv=rospy.ServiceProxy('service_example',SetBool)
 
#use the service and send it a value. In this case, I can send 1 or 0
service_example=srv(True)
 
#print the result from the service
print(service_example)
```
