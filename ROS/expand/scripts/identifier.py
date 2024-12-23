#!usr/bin/env python3

import rospy
import numpy as np
from numpy import linalg as LA
import time
import tf2_ros as tf
import yaml

from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

def rushB(yaw,dt):
  B = np.array([[np.cos(yaw)*dt, 0],
                [np.sin(yaw)*dt, 0],
                [0, dt]])
  return B


class Indentifier:

    def __init__(self):
        # self.nr = rospy.get_param("number_of_robots") # Total number of robots 
        self.nr = 4 # Total number of robots 
        self.ip = []
        self.publist = []

        self.num_det = 0 # Number of detected bounding boxes with LiDAR
        self.detlist = [] # Array of bounding box centroids as numpy arrays
        self.slamlist = np.zeros((2, self.nr)) # Array of SLAM proposed coordinates as numpy arrays
    
        self.tfBuffer = tf.Buffer()
        self.listener = tf.TransformListener(self.tfBuffer)


        for i in range(1,self.nr+1):
            self.ip.append([rospy.get_param("/robot{}/map_merge/init_pose_x".format(i)),
                            rospy.get_param("/robot{}/map_merge/init_pose_y".format(i)),
                            rospy.get_param("/robot{}/map_merge/init_pose_z".format(i)),
                            rospy.get_param("/robot{}/map_merge/init_pose_yaw".format(i))])
            
            # self.publist.append(rospy.Publisher(("/robot{}/cmd_vel".format(i)),Twist,queue_size=1))
        self.pbber = rospy.Publisher("/robot4/cmd_vel", Twist, queue_size=1)
        self.sbber = rospy.Subscriber("/centroids_and_vertices", Float32MultiArray, self.bb_subscriber)


    def bb_subscriber(self,data):
        # t = data.layout.dim[0]
        self.detlist = []
        self.num_det = data.layout.dim[0].size
        stride = int(data.layout.dim[1].size)
        for i in range(0,self.num_det):
            pt = np.array([[data.data[i*stride]],[data.data[i*stride + 1]]])
            self.detlist.append(pt)

        self.id_assigner()

    def id_assigner(self):

        self.slamlist = np.zeros((2,self.nr))

        # Get the SLAM Proposed [x y] coordinates
        for i in range(self.nr):
            try:
                temptf = self.tfBuffer.lookup_transform('robot{}/map'.format(i+1), 'robot{}/base_link'.format(i+1), rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
  
            self.slamlist[0][i] = temptf.transform.rotation.x + self.ip[i][0]
            self.slamlist[1][i] = temptf.transform.rotation.y + self.ip[i][1]

        # print(self.slamlist)
            
        detected = []

        for q in self.detlist:
            idx = (LA.norm(self.slamlist-q,axis=0)).argmin()
            detected.append(idx+1)

        print('I Detected robot')
        print(detected)

        





    def robot_identify(self):
        
        VELO = 0.5
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