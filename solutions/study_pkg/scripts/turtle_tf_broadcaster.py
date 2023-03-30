#!/usr/bin/env python
import rospy
import tf
import math
from tf.transformations import quaternion_from_euler
from turtlesim.msg import Pose

class AroundTheWorld:
    def __init__(self) -> None:
        self.spin_counter = 0
        rospy.init_node('tf_turtle')
        self.turtlename = rospy.get_param('~turtle_tf_name')
        self.sattelitename = rospy.get_param('~carrot_tf_name')
        rospy.Subscriber('input_pose',
                    Pose,
                    self.handle_turtle_pose)

    def handle_turtle_pose(self,msg):
        # Get broadcaster object
        br = tf.TransformBroadcaster()
        # Broadcast TF trasform (world -> turtlename)
        br.sendTransform((msg.x, msg.y, 0),
                        quaternion_from_euler(0, 0, msg.theta),
                        rospy.Time.now(),
                        self.turtlename,
                        "world")
        satellite = tf.TransformBroadcaster()
        satellite.sendTransform((math.sin(self.spin_counter) * 1.5, math.cos(self.spin_counter) * 1.5, 1), 
                                quaternion_from_euler(0, 0, self.spin_counter), rospy.Time.now(), self.sattelitename, self.turtlename)
        self.spin_counter += 0.01
        
        
if __name__=='__main__':
    aroundTf = AroundTheWorld()
    rospy.spin()
