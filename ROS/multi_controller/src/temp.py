#!/usr/bin/env python3

import rospy
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import Pose
import math
import random
import numpy
from cmath import sqrt
import rospy
from geometry_msgs.msg import Twist , Point
from operator import add

GOTO = [9,4.5]

class GazeboLinkPose:
  link_names = ["robot2::base_link","robot3::base_link","robot4::base_link","robot5::base_link","robot6::base_link","robot7::base_link"]
  min_dist = 100000000
  counter = 0
  ego_name = None
  closest_pos = Pose()
  my_pos = Pose()

  def __init__(self,ego_name):
    self.link_names.remove(ego_name)
    self.ego_name = ego_name

    if not self.link_names:
      raise ValueError("'link_name' is an empty string")

    self.states_sub = rospy.Subscriber("/gazebo/link_states", LinkStates, self.callback)

  def callback(self, data):
    try:
      ind_ego = data.name.index(self.ego_name)
      self.my_pos = data.pose[ind_ego]
      for i in self.link_names:
        ind = data.name.index(i)
        link_pose = data.pose[ind]
        temp_dist = math.sqrt((link_pose.position.x-self.my_pos.position.x)**2 + (link_pose.position.y-self.my_pos.position.y)**2)
        if self.min_dist > temp_dist:
        #   print("###### YESSS",i)
          self.min_dist = temp_dist
          self.closest_pos = link_pose

    except ValueError as e:
      pass

def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return yaw_z # in radians

def wrap2pi(theta):
    theta_1 = theta
    if abs(theta_1) >= 2*math.pi:
        theta_1 = math.remainder(theta_1,2*math.pi)
    if theta_1 >= math.pi:
        theta_new = -2*math.pi + theta
    elif theta_1 < -math.pi:
        theta_new = 2*math.pi + theta
    else:
        theta_new = theta_1
    return theta_new

def signf(x):
    if x > 0:
        out = 1
    elif x < 0:
        out = -1
    else:
        out = 0
    return out

def delwz(psi_rbt, x_rbt, x_neigh):
    kdelw = 0.5
    r_vec = numpy.array(x_rbt) - numpy.array(x_neigh)
    r = numpy.sqrt(numpy.dot(r_vec,r_vec))
    psi_vec = numpy.array([math.cos(psi_rbt), math.sin(psi_rbt)])
    dot_prod = numpy.dot(psi_vec, r_vec)
    cross_prod = numpy.cross(numpy.append(psi_vec,0),numpy.append(r_vec,0))
    Dwz = -kdelw*wrap2pi(signf(cross_prod[2])*dot_prod/pow(r,3))
    return Dwz

def delVx(psi_rbt, x_rbt, x_neigh):
    kdelv = 0.5
    r_vec = numpy.array(x_rbt) - numpy.array(x_neigh)
    r = numpy.sqrt(numpy.dot(r_vec,r_vec))
    psi_vec = numpy.array([math.cos(psi_rbt), math.sin(psi_rbt)])
    dot_prod = numpy.dot(psi_vec, r_vec)
    DVx = kdelv*dot_prod/pow(r,3)
    return DVx

def Wzref(det_tgt, x_Tgt, x_rbt, psi_rbt):
    kw = 1.0
    t_vec = numpy.array(x_Tgt) - numpy.array(x_rbt)
    psi_vec = numpy.array([math.cos(psi_rbt), math.sin(psi_rbt)])
    dot_prod = numpy.dot(psi_vec,t_vec)
    cross_prod = numpy.cross(numpy.append(psi_vec,0), numpy.append(t_vec,0))
    wzref = kw*wrap2pi(dot_prod*signf(cross_prod[2]))
    if not det_tgt:
      wzref = 0.0
    return wzref

def Vxref(det_tgt, x_Tgt, x_rbt, psi_rbt):
    kv = 1.0
    t_vec = numpy.array(x_Tgt) - numpy.array(x_rbt)
    psi_vec = numpy.array([math.cos(psi_rbt), math.sin(psi_rbt)])
    dot_prod = numpy.dot(psi_vec, t_vec)
    vxref = kv*dot_prod
    if not det_tgt:
      vxref = 0.0
    return vxref

def VxWxsearch(det_tgt, s_count, psi_fix_prev, psi_rbt):
   kw = 1.0
   Ts = 100.0
   psi_fix = psi_fix_prev
   if math.ceil(s_count/Ts) == math.floor(s_count/Ts):
      s_count = 0
      psi_fix = random.uniform(-math.pi,math.pi)
      print(psi_fix)
   Vx = 0.5
   psi_vec = numpy.array([math.cos(psi_rbt), math.sin(psi_rbt)])
   psi_r_vec = numpy.array([math.cos(psi_fix), math.sin(psi_fix)])
   dot_prod = numpy.dot(psi_vec,psi_r_vec)
   cross_prod = numpy.cross(numpy.append(psi_vec,0), numpy.append(psi_r_vec,0))
   Wz = kw*wrap2pi(dot_prod*signf(cross_prod[2]))
   if det_tgt:
      Vx = 0.0
      Wz = 0.0
   s_count = s_count + 1
   return Vx,Wz, s_count, psi_fix

if __name__ == '__main__':
  s_count = 0
  psi_fix_prev = random.uniform(-math.pi,math.pi)
  det_tgt = False
  fault = 1

  try:
    rospy.init_node('gazebo_link_pose', anonymous=True)
    gp = GazeboLinkPose("robot6::base_link")
    pub = rospy.Publisher("/robot6/cmd_vel" , Twist , queue_size=10)
    msg_to_publish = Twist()

    if time.time.now >= 5.0 and fault == 1:
        bias_pos_Tgt = [random.randint([-5,5]), random.randint([-5,5])]
        bias_pos_rbt_IMU = [random.randint([-5,5]), random.randint([-5,5])]
        bias_yaw_rbt_IMU = random.randint([-10,10])*math.pi/180.0
    else:
        bias_pos_Tgt = [0,0]
        bias_pos_rbt_IMU = [0,0]
        bias_yaw_rbt_IMU = 0.0

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
      ego_yaw_CA = euler_from_quaternion(gp.my_pos.orientation.x,gp.my_pos.orientation.y,gp.my_pos.orientation.z,gp.my_pos.orientation.w)
      ego_yaw = wrap2pi(bias_yaw_rbt_IMU + euler_from_quaternion(gp.my_pos.orientation.x,gp.my_pos.orientation.y,gp.my_pos.orientation.z,gp.my_pos.orientation.w))
      Vx, Wz, s_count, psi_fix_prev = VxWxsearch(det_tgt, s_count, psi_fix_prev,ego_yaw)
      msg_to_publish.linear.x  = Vxref(det_tgt, list(map(add,GOTO,bias_pos_Tgt)), list(map(add,[gp.my_pos.position.x,gp.my_pos.position.y],bias_pos_rbt_IMU)),ego_yaw) \
        + delVx(ego_yaw_CA, [gp.my_pos.position.x,gp.my_pos.position.y],[gp.closest_pos.position.x,gp.closest_pos.position.y]) + Vx
      msg_to_publish.angular.z = wrap2pi(Wzref(det_tgt, list(map(add,GOTO,bias_pos_Tgt)), list(map(add,[gp.my_pos.position.x,gp.my_pos.position.y],bias_pos_rbt_IMU)),ego_yaw) \
                                         + delwz(ego_yaw_CA, [gp.my_pos.position.x,gp.my_pos.position.y],[gp.closest_pos.position.x,gp.closest_pos.position.y])+Wz)

      pub.publish(msg_to_publish)
      rate.sleep()

  except rospy.ROSInterruptException:
    pass