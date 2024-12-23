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
import time
from operator import add
from aol_stack.msg import Target_deets

p2go = []
det_tgt = True

KREF_V = 0.5
KREF_W = 0.5
KDEL_V= [0.05, 0.8]
KDEL_W= [0.05, 0.8]
KSRCH = 0
Wcon = 0.3
DSAFE = 4
D_OBSSAFE = 1.0
SRCH_CHANGE = 100 

REF_MAX = 0.8
OBS_MAX = 0.5
SRCH_MAX = 0.5


class GazeboLinkPose:
  link_names = ["robot2::base_link","robot3::base_link","robot4::base_link","robot5::base_link","robot6::base_link","robot7::base_link"]
  min_dist = 100000000
  counter = 0
  ego_name = None
  closest_pos = [0, 0]
  my_pos = [0, 0, 0]
  target_pos = [float('nan'), float('nan')]

  def __init__(self,ego_name):
    self.link_names.remove(ego_name)
    self.ego_name = ego_name

    if not self.link_names:
      raise ValueError("'link_name' is an empty string")

    self.states_sub = rospy.Subscriber("/gazebo/link_states", LinkStates, self.callback)

  def euler_from_quaternion(self,x, y, z, w):
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

  def callback(self, data):
    try:
      ind_ego = data.name.index(self.ego_name)
      targ_ego = data.name.index('robot1::base_link')
      my_yaw = self.euler_from_quaternion(data.pose[ind_ego].orientation.x,data.pose[ind_ego].orientation.y,data.pose[ind_ego].orientation.z,data.pose[ind_ego].orientation.w)
      self.my_pos = [data.pose[ind_ego].position.x, data.pose[ind_ego].position.y, my_yaw]
      self.target_pos = [data.pose[targ_ego].position.x, data.pose[targ_ego].position.y]
      for i in self.link_names:
        ind = data.name.index(i)
        link_pose = data.pose[ind]
        temp_dist = math.sqrt((link_pose.position.x-self.my_pos[0])**2 + (link_pose.position.y-self.my_pos[1])**2)
        if self.min_dist > temp_dist:
        #   print("###### YESSS",i)
          self.min_dist = temp_dist
          self.closest_pos = [link_pose.position.x, link_pose.position.y]

    except ValueError as e:
      pass

class AOL_Claw:
  det_tgt = False
  tgt_vec = [0, 0]
  rbt_vec = [0, 0]
  obs_vec = [0, 0]
  psi_rbt = 0
  psi_fix_prev = 0
  s_count = 0

  def __init__(self, kref_v, kref_w, kdel_v, kdel_w, ksrch, constant_w, dsafe, d_obssafe, ref_max, obs_max, srch_max ,srch_Ts = 100.0, thresh_vel = -0.2):
    self.kref_v = kref_v
    self.kref_w = kref_w
    self.kdel_v = kdel_v
    self.kdel_w = kdel_w
    self.ksrch = ksrch
    self.constant_w = constant_w
    self.dsafe = dsafe
    self.d_obssafe = d_obssafe
    self.srch_Ts = srch_Ts
    self.thresh_vel = thresh_vel
    self.ref_max = ref_max
    self.obs_max = obs_max
    self.srch_max = srch_max

  def wrap2pi(self,theta):
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

  def signf(self, x):
      x = float(x)
      return (x > 0) - (x < 0)
  
  def vect_hat(self, vect):
   vect_mag = numpy.sqrt(numpy.dot(vect,vect))
   if vect_mag > 0.001:
      vecthat = vect/vect_mag
   else:
      vecthat = [0.0, 0.0]
   return vecthat
  
  def del_Wz(self):
    x_vec = numpy.array(self.obs_vec) - numpy.array(self.rbt_vec)
    x_mag = numpy.sqrt(numpy.dot(x_vec, x_vec))
    x_uni_vec = self.vect_hat(x_vec)
    psi_vec = numpy.array([math.cos(self.psi_rbt), math.sin(self.psi_rbt)])
    if x_mag > self.d_obssafe:
       if numpy.cross(psi_vec, x_vec) != 0 and numpy.dot(psi_vec, x_vec)>=0:
          return -self.kdel_w[0] * self.signf(numpy.cross(psi_vec, x_vec))*(numpy.dot(psi_vec, x_vec)/x_mag**3)
       elif numpy.cross(psi_vec, x_vec) == 0 and numpy.dot(psi_vec, x_vec)>=0:
          return -self.kdel_w[0] *(numpy.dot(psi_vec, x_vec)/x_mag**3)
       else:
          return 0
    else:
       if numpy.cross(psi_vec, x_vec) != 0:
          return -self.kdel_w[1] * self.signf(numpy.cross(psi_vec, x_vec))*(numpy.dot(psi_vec, x_uni_vec)+1)/x_mag**2
       else:
          return -self.kdel_w[1] * (numpy.dot(psi_vec, x_uni_vec)+1)/x_mag**2

  def del_Vx(self):
    x_vec = numpy.array(self.obs_vec) - numpy.array(self.rbt_vec)
    x_mag = numpy.sqrt(numpy.dot(x_vec, x_vec))
    x_uni_vec = self.vect_hat(x_vec)
    psi_vec = numpy.array([math.cos(self.psi_rbt), math.sin(self.psi_rbt)])
    
    if x_mag > self.d_obssafe:
       if numpy.dot(psi_vec, x_vec)>=0:
          return -self.kdel_v[0] *numpy.dot(psi_vec, x_uni_vec)/x_mag**2
       else:
          return 0
    else:
        return -self.kdel_v[1] *numpy.dot(psi_vec, x_uni_vec)/x_mag**2
    
  def Wzref(self):
    if det_tgt:
      x_vec = numpy.array(self.tgt_vec) - numpy.array(self.rbt_vec)
      x_mag = numpy.sqrt(numpy.dot(x_vec, x_vec))
      x_uni_vec = self.vect_hat(x_vec)
      psi_vec = numpy.array([math.cos(self.psi_rbt), math.sin(self.psi_rbt)])

      if x_mag > self.dsafe:
        if numpy.dot(psi_vec, x_uni_vec) >= 0:
          print("first if", numpy.cross(psi_vec, x_uni_vec))
          return numpy.cross(psi_vec, x_uni_vec)
        elif numpy.dot(psi_vec, x_uni_vec) < 0:
          print("second if",self.constant_w)
          return self.constant_w
        else:
          print("third if",self.kref_w*(self.signf(numpy.cross(psi_vec, x_uni_vec)))*self.constant_w)
          return self.kref_w*(self.signf(numpy.cross(psi_vec, x_uni_vec)))*self.constant_w
      else:
        if numpy.cross(psi_vec, x_vec) != 0:
          return -self.kref_w * self.signf(numpy.cross(psi_vec, x_vec))*(numpy.dot(psi_vec, x_uni_vec)+1)/x_mag**2
        else:
          return -self.kref_w * (numpy.dot(psi_vec, x_uni_vec)+1)
    else:
       return 0
    
  def Vxref(self):
    if det_tgt:
      x_vec = numpy.array(self.tgt_vec) - numpy.array(self.rbt_vec)
      x_mag = numpy.sqrt(numpy.dot(x_vec, x_vec))
      x_uni_vec = self.vect_hat(x_vec)
      psi_vec = numpy.array([math.cos(self.psi_rbt), math.sin(self.psi_rbt)])
      
      if x_mag > self.dsafe:
        vel = self.kref_v * (numpy.dot(psi_vec, x_uni_vec))*(numpy.sqrt(numpy.dot(x_vec, x_vec)) - self.dsafe)
      else:
         vel = - self.kref_v *numpy.dot(psi_vec, x_uni_vec)
      if vel > 0:
        return min(vel, self.ref_max)
      return 0 
    else:
       return 0
    
  def VxWxsearch(self):
    psi_fix = self.psi_fix_prev
    if math.ceil(self.s_count/self.srch_Ts) == math.floor(self.s_count/self.srch_Ts):
      self.s_count = 0
      psi_fix = random.uniform(-math.pi,math.pi)
    Vx = 0.2 # TODO
    psi_vec = numpy.array([math.cos(self.psi_rbt), math.sin(self.psi_rbt)])
    psi_r_vec = numpy.array([math.cos(psi_fix), math.sin(psi_fix)])
    dot_prod = numpy.dot(psi_vec,psi_r_vec)
    cross_prod = numpy.cross(numpy.append(psi_vec,0), numpy.append(psi_r_vec,0))
    Wz = self.ksrch*(dot_prod*self.signf(cross_prod[2]))
    if self.det_tgt:
      Vx = 0.0
      Wz = 0.0
    self.s_count = self.s_count + 1
    return min(Vx, self.srch_max),Wz

  def clawRun(self, det_tgt, rbt_vec, tgt_vec, obs_vec, psi_rbt):
      self.det_tgt = det_tgt
      self.tgt_vec = tgt_vec
      self.rbt_vec = rbt_vec
      self.psi_rbt = psi_rbt
      self.obs_vec = obs_vec
      
      msg_to_publish = Twist()
      Vx_srch, Wz_srch = self.VxWxsearch()

      # Calculate Vx
      msg_to_publish.linear.x = self.Vxref() + Vx_srch + self.del_Vx()
      print("X vel",msg_to_publish.linear.x, self.Vxref(), Vx_srch, self.del_Vx())
      
      # Calculate Wz
      msg_to_publish.angular.z = self.Wzref() + Wz_srch + self.del_Wz()
      print("W vel", msg_to_publish.angular.z, self.Wzref(), Wz_srch, self.del_Wz())

      if(msg_to_publish.linear.x<0):
        msg_to_publish.linear.x = 0
      
      return msg_to_publish

def clbk_cam(data):
   global p2go, det_tgt
   p2go = [data.x, data.y]
  #  if not math.isnan(p2go[0]) and not math.isnan(p2go[1]):
  #     det_tgt = True
  #  else:
  #     det_tgt = False

   det_tgt = True #TODO

def main():
  s_count = 0
  psi_fix_prev = random.uniform(-math.pi,math.pi)
  fault = 0

  if rospy.has_param('robot_prefix'):
    robot_prefix = rospy.get_param('robot_prefix')
  else:
     raise Exception("ERROR: robot_prefix not mentioned")

  print("Starting AOL_Controller for:",robot_prefix)

  try:
    rospy.init_node('gazebo_link_pose', anonymous=True)
    gp = GazeboLinkPose(robot_prefix+"::base_link")
    pub = rospy.Publisher("/"+robot_prefix+"/cmd_vel" , Twist , queue_size=10)
    sub_cam = rospy.Subscriber("/"+ robot_prefix +'/target_position', Target_deets, clbk_cam)
    msg_to_publish = Twist()
    
    claw = AOL_Claw(KREF_V, KREF_W, KDEL_V, KDEL_W, KSRCH, Wcon, DSAFE, D_OBSSAFE, REF_MAX, OBS_MAX, SRCH_MAX, SRCH_CHANGE)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
      if not math.isnan(gp.target_pos[0]):
        msg_to_publish = claw.clawRun(True, gp.my_pos[:2], gp.target_pos, gp.closest_pos, gp.my_pos[2])
        pub.publish(msg_to_publish)
      else:
        print("Waiting for coordinate:",robot_prefix)
      rate.sleep()

  except rospy.ROSInterruptException:
    pass

if __name__ == '__main__':
  main()
