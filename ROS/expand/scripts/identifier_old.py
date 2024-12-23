#!usr/bin/env python3

import rospy
import numpy as np
import time
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf2_ros as tf

from geometry_msgs.msg import Twist

def rushB(yaw,dt):
  B = np.array([[np.cos(yaw)*dt, 0],
                [np.sin(yaw)*dt, 0],
                [0, dt]])
  return B


class Indentifier:

    def __init__(self):
        # self.nr = rospy.get_param("number_of_robots") # Total number of robots 
        self.nr = 2 # Total number of robots 
        self.ip = []
        self.publist = []
    
        for i in range(1,self.nr+1):
            self.ip.append([rospy.get_param("/robot{}/map_merge/init_pose_x".format(i)),
                            rospy.get_param("/robot{}/map_merge/init_pose_y".format(i)),
                            rospy.get_param("/robot{}/map_merge/init_pose_z".format(i)),
                            rospy.get_param("/robot{}/map_merge/init_pose_yaw".format(i))])
            
            # self.publist.append(rospy.Publisher(("/robot{}/cmd_vel".format(i)),Twist,queue_size=1))
        self.pbber = rospy.Publisher("/robot4/cmd_vel", Twist ,queue_size=1)



    def robot_identify(self):
        
        VELO = -0.5
        OMEGA = 0.5

        msg = Twist()
        msg.linear.x = VELO
        msg.angular.z = OMEGA

        for i in range(5):
            self.pbber.publish(msg)
            time.sleep(0.1)

        msg.linear.x = VELO
        msg.angular.z = OMEGA

        # self.publist[1].publish(msg)
        start_timer = rospy.Time.now().to_sec()
        for i in range(2):
            self.pbber.publish(msg)
            time.sleep(0.1)

        tfBuffer = tf.Buffer()
        listener = tf.TransformListener(tfBuffer)

        trans_t_minus_1 = None

        while (trans_t_minus_1 is None):
            try:
                trans_t_minus_1 = tfBuffer.lookup_transform('robot4/map', 'robot4/base_link', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
        _,_,yaw = euler_from_quaternion([trans_t_minus_1.transform.rotation.x,trans_t_minus_1.transform.rotation.y,trans_t_minus_1.transform.rotation.z,trans_t_minus_1.transform.rotation.w])
        
        print('Actual State')
        print(trans_t_minus_1.transform.translation)
        print("theta: {}".format(yaw))

        state_t_minus_1 = np.array([[trans_t_minus_1.transform.translation.x], [trans_t_minus_1.transform.translation.y], [yaw]])
        print('Previous State')
        print(state_t_minus_1)   

        # time.sleep(0.5)

        try:
            trans_t_minus_1 = tfBuffer.lookup_transform('robot4/map', 'robot4/base_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

        _,_,yaw = euler_from_quaternion([trans_t_minus_1.transform.rotation.x,trans_t_minus_1.transform.rotation.y,trans_t_minus_1.transform.rotation.z,trans_t_minus_1.transform.rotation.w])
        print('Actual State')
        print(trans_t_minus_1.transform.translation)
        print("theta: {}".format(yaw))

        stop_timer = rospy.Time.now().to_sec()

        control_vector_t_minus_1 = np.array([[VELO],[OMEGA]])
        state_estimate_t = (state_t_minus_1) + np.matmul((rushB(yaw, -0.55 + stop_timer-start_timer)), (control_vector_t_minus_1)) 
    
        print('Estimated State')
        print(state_estimate_t)   




        time.sleep(1)
        msg.linear.x = 0
        msg.angular.z = 0.0
        self.pbber.publish(msg)
        # print(yaw)





if __name__ == '__main__':
    rospy.init_node('identifier_node')
    Idfy = Indentifier()
    Idfy.robot_identify()