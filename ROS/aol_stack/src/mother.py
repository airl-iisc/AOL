#! /usr/bin/env python3
'''
This is where the learning algorithm will go
Input: Camera_Detection
Output: Communication, coordinates_for_control
'''
# import ros stuff
import rospy
# import ros message
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf import transformations
from aol_stack.msg import Aol_comm, Target_deets
import random
import math
import numpy

def AOLfusionLayer1(xhatS_Tgt_i, xhat_Tgt_i_prev, alpha_hat_i, alphapr_hat_i, det_Tgt_i):
    if det_Tgt_i > 0.0: # tgt detected
        alpha_i = (alpha_hat_i)/(alpha_hat_i + alphapr_hat_i)
    else: # tgt undetected
        alpha_i = 0.0

    xhatI_Tgt_i = alpha_i*numpy.array(xhatS_Tgt_i) + (1-alpha_i)*numpy.array(xhat_Tgt_i_prev)

    return xhatI_Tgt_i, alpha_i

# communicate (i, xhatI_Tgt_i, xhat_Tgt_i_prev, det_Tgt_i, what_ii) and get the same in turn from others
# put the ID of neighbors in the 1D array neigh_ID_i based on who is in range and who is not, with some probability
# stack all the other info together from the neighbors in 2D arrays xhatI_Tgt_ij, xhat_Tgt_ij_prev, det_Tgt_ij, what_ij
# xhatI_Tgt_ij should be an array of 2 X n_i, where n_i is the no. of neighbors robot i has

def AOLfusionLayer2(xhatI_Tgt_i, xhatI_Tgt_ij, what_ii, what_ij):
    wii = what_ii/(what_ii + sum(what_ij))
    j = 0
    wij = numpy.zeros(shape=(len(what_ij),1))
    for whatij in what_ij:
        wij[j][:] = whatij/(what_ii + sum(what_ij))
        j = j + 1
    xhat_Tgt_i = wii*xhatI_Tgt_i + sum(numpy.multiply(numpy.array(wij),numpy.array(xhatI_Tgt_ij)));
    return xhat_Tgt_i.tolist(), wii

def vec_dist(vec1,vec2):
    dist = math.sqrt(pow(vec1[0]-vec2[0],2)+pow(vec1[1]-vec2[1],2))
    return dist

def LearningPhaseAOLver1(t_count, det_Tgt_i, xhatI_Tgt_ij, xhatI_Tgt_i, xhatS_Tgt_i, xhat_Tgt_i_prev, xhat_Tgt_j_prev, what_ij, what_i, alpha_hat_i, alphapr_hat_i):
    Tp = 15.0
    eta_a = 0.01
    eta_w = 15.0

    whatij = numpy.append(what_ij, what_i)
    #print(whatij)
    xhat_Tgt_ij_p = [xhat_Tgt_j_prev, xhat_Tgt_i_prev]
    wmax = max(whatij)
    ID_star = numpy.array(whatij).argmax()
    xhat_star = xhat_Tgt_ij_p[ID_star][:]
    #print(xhat_star, ID_star)

    if math.ceil(t_count/Tp) == math.floor(t_count/Tp):
        what_i = 1.0
        alpha_hat_i = 1.0
        alphapr_hat_i = 1.0
    print(xhatS_Tgt_i)
    lss_alpha = min(vec_dist(xhatS_Tgt_i,xhat_star)/15.0,1.0)
    lss_alphapr = min(vec_dist(xhat_Tgt_i_prev,xhat_star)/15.0,1.0)

    alpha_hat_i_nxt = alpha_hat_i*math.exp(-eta_a*lss_alpha)
    alphapr_hat_i_nxt = alphapr_hat_i*math.exp(-eta_a*lss_alphapr)
    what_i_nxt = what_i*math.exp(-eta_w*(1-det_Tgt_i))

    return alpha_hat_i_nxt, alphapr_hat_i_nxt, what_i_nxt

def LearningPhaseAOLver2(t_count, det_Tgt_i, what_i, alpha_hat_i, alphapr_hat_i, del_alpha_i, del_detTgt_i, alpha_i):
    Tp = 15.0

    eta_w = 15.0
    e_a1 = 10.0
    e_a2 = 5.0
    p_mag = 0.1

    if math.ceil(t_count/Tp) == math.floor(t_count/Tp):
        what_i = 1.0
        alpha_hat_i = 1.0
        alphapr_hat_i = 1.0

    if t_count == 0:
        e_pa1 = random.choice([0, 1])*p_mag
        e_pa2 = p_mag - e_pa1
    else:
        e_pa1 = 0
        e_pa2 = 0

    alpha_hat_i_nxt = alpha_hat_i*math.exp(e_a1*del_alpha_i*del_detTgt_i + e_pa1*(1-det_Tgt_i) + e_a2*alpha_i*det_Tgt_i)
    alphapr_hat_i_nxt = alphapr_hat_i*math.exp(-e_a1*del_alpha_i*del_detTgt_i + e_pa2*(1-det_Tgt_i) + e_a2*(1-alpha_i)*det_Tgt_i)
    what_i_nxt = what_i*math.exp(-eta_w*(1-det_Tgt_i))

    return alpha_hat_i_nxt, alphapr_hat_i_nxt, what_i_nxt

def LearningPhaseAOLver3(t_count, det_Tgt_i, what_i, alpha_hat_i, alphapr_hat_i, del_alpha_i, del_detTgt_i, alpha_i, del_w_ii, w_ii):
    Tp = 15.0

    e_w1 = 8
    e_w2 = 0

    e_a1 = 10.0
    e_a2 = 5.0

    p_mag = 0.1

    if math.ceil(t_count/Tp) == math.floor(t_count/Tp):
        what_i = 1.0
        alpha_hat_i = 1.0
        alphapr_hat_i = 1.0
        e_pw = random.uniform(0,1)*p_mag

    if t_count == 0:
        e_pa1 = random.randint([0, 1])*p_mag
        e_pa2 = p_mag - e_pa1
        e_pw = random.uniform(0,1)*p_mag
    else:
        e_pa1 = 0.0
        e_pa2 = 0.0
        e_pw = 0.0

    alpha_hat_i_nxt = alpha_hat_i*math.exp(e_a1*del_alpha_i*del_detTgt_i + e_pa1*(1-det_Tgt_i) + e_a2*alpha_i*det_Tgt_i)
    alphapr_hat_i_nxt = alphapr_hat_i*math.exp(-e_a1*del_alpha_i*del_detTgt_i + e_pa2*(1-det_Tgt_i) + e_a2*(1-alpha_i)*det_Tgt_i)
    what_i_nxt = what_i*math.exp(e_w1*del_w_ii*del_detTgt_i + e_pw*(1-det_Tgt_i) + e_w2*w_ii*det_Tgt_i)

    return alpha_hat_i_nxt, alphapr_hat_i_nxt, what_i_nxt

class AOL_Learning_Algo:

    # robot params
    robot_names = ["robot2","robot3","robot4","robot5","robot6","robot7"]
    ego_robot = ""
    r_id = None

    # ego robot's variables
    x_prev_learned = [0, 0]

    x_camera = [0,0]
    x_confidence = 0
    x_confidence_prev = 0

    # Layer variables
    x_layer1 = []
    alpha_hat_i = 1
    alphapr_hat_i = 1
    alpha_i = 0
    alpha_previous_i = 0

    wii = 1
    wii_previous = 1

    what_ii = 1

    # Neighbours comm
    neighbour_data = {}
    xhat_layer1_n = None
    weights_n = None
    xhat_prev_n = None


    def clbk_comm(self, data):
        self.neighbour_data[data.r_id] = [data.x_layer1,data.y_layer1, data.what_ii, data.x_prev_learned, data.y_prev_learned]

    def clbk_camera(self, msg):
        self.x_camera = [msg.x, msg.y]
        self.x_confidence_prev = self.x_confidence
        self.x_confidence = msg.conf_score

    def __init__(self, ego_robot, r_id):
        self.ego_robot = ego_robot
        self.r_id = r_id

        rospy.init_node('AOL_Learning_Algo', anonymous=True)
        self.robot_names.remove(ego_robot)
        for i in self.robot_names:
            rospy.Subscriber("/"+i+"_comm",Aol_comm,self.clbk_comm)
        sub_target_pos = rospy.Subscriber('/'+self.ego_robot+'/target_position', Target_deets, self.clbk_camera)
        self.pub_comm = rospy.Publisher('/'+self.ego_robot+"_comm", Aol_comm, queue_size=10)
        rate = rospy.Rate(10)
        time_counter = 0

        while not rospy.is_shutdown():

            # Layer 1
            self.alpha_previous_i = self.alpha_i
            self.x_layer1, self.alpha_i = AOLfusionLayer1(self.x_camera, self.x_prev_learned, self.alpha_hat_i, self.alphapr_hat_i, self.x_confidence)
            print("After Layer 1: ",self.x_layer1, self.alpha_i)

            # SEND DATA TO NEIGHBOURS x_layer1, x_prev_learned, what_ii, conf_score
            msg_to_send = Aol_comm()
            msg_to_send.r_id = self.r_id
            msg_to_send.x_layer1 = self.x_layer1[0]
            msg_to_send.y_layer1 = self.x_layer1[1]
            msg_to_send.x_prev_learned = self.x_prev_learned[0]
            msg_to_send.y_prev_learned = self.x_prev_learned[1]
            msg_to_send.what_ii = self.what_ii
            msg_to_send.conf_score = self.x_confidence

            self.pub_comm.publish(msg_to_send)


            # Compile Data from neighbours
            temp_xhat = []
            temp_weights = []
            temp_xprev = []
            for key, value in self.neighbour_data.items():
                temp_xhat.append([value[0],value[1]])
                temp_weights.append(value[2])
                temp_xprev.append([value[3],value[4]])

            self.xhat_layer1_n = numpy.array(temp_xhat)
            self.weights_n = numpy.array(temp_weights)
            self.xhat_prev_n = numpy.array(temp_xprev)

            # Layer 2
            self.wii_previous = self.wii
            self.x_prev_learned, self.wii = AOLfusionLayer2(self.x_layer1, self.xhat_layer1_n, self.what_ii, self.weights_n)

            print("After Layer 2: ", self.x_prev_learned, self.wii)

            # AOL v1
            # self.alpha_hat_i, self.alphapr_hat_i, self.what_ii =  LearningPhaseAOLver1(time_counter, self.x_confidence, self.xhat_layer1_n, self.x_layer1, self.x_camera, self.x_prev_learned, self.xhat_prev_n, self.weights_n, self.what_ii, self.alpha_hat_i, self.alphapr_hat_i)

            # AOL v2
            Del_alphai = self.alpha_i - self.alpha_previous_i
            Del_det_Tgt_i = self.x_confidence - self.x_confidence_prev

            self.alpha_hat_i, self.alphapr_hat_i, self.what_ii = LearningPhaseAOLver2(time_counter, self.x_confidence, self.what_ii, self.alpha_hat_i, self.alphapr_hat_i, Del_alphai, Del_det_Tgt_i, self.alpha_i)

            # AOL v3
            # Del_wi = self.wii - self.wii_previous

            # self.alpha_hat_i, self.alphapr_hat_i, self.what_ii = LearningPhaseAOLver3(time_counter, self.x_confidence, self.what_ii, self.alpha_hat_i, Del_alphai, Del_det_Tgt_i, self.alpha_i, Del_wi, self.wii)

            print("After Learning Layer: ", self.alpha_hat_i, self.alphapr_hat_i, self.what_ii, self.alphapr_hat_i)

            rate.sleep()


def main():
    robot_prefix = rospy.get_param('robot_prefix')
    robot_id = int(robot_prefix[-1])
    AOL_Learning_Algo(robot_prefix, robot_id)


if __name__ == "__main__":
    main()

