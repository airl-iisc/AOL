from cmath import sqrt
import math
import random
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
        e_pa1 = random.randint([0, 1])*p_mag
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
def main():
    ### sample run ###

    xhatS_Tgt_i = [2.0,3.0]
    xhat_Tgt_i_prev = [1.0,7.0]
    alpha_hat_i = 0.5
    alphapr_hat_i = 0.5
    det_Tgt_i = 0.9

    '''
    i-> ego
    nan -> 0,0

    xhatS_Tgt_i: Target as seen by camera (vec)
    xhat_Tgt_i_prev: Previously final estimate (vec, init as 0,0)

    alpha_hat_i: weight 1 (init as 1)
    alphapr_hat_i: weight 2 (init as 1)

    det_Tgt_i: Detection score
    '''

    xhatI_Tgt_i, alpha_i = AOLfusionLayer1(xhatS_Tgt_i, xhat_Tgt_i_prev, alpha_hat_i, alphapr_hat_i, det_Tgt_i)
    print(xhatI_Tgt_i, alpha_i)

    xhatI_Tgt_ij = numpy.array([[1.0,2.0],[2.0,7.0],[9.0,-1.0]])
    what_ii = 0.5
    what_ij = numpy.array([0.2,0.5,0.1])
    #print(xhatI_Tgt_ij[0][:])

    #print(numpy.multiply(numpy.array([[1],[2],[3]]),xhatI_Tgt_ij))
    #print(sum(numpy.multiply(numpy.array([[1],[2],[3]]),xhatI_Tgt_ij)))

    '''
    xhatI_Tgt_i: Previous Layer x out (vec)
    xhatI_Tgt_ij: neighbours output of layer 1 (vec)

    alpha_hat_i: weight 1 (init as 1)
    alphapr_hat_i: weight 2 (init as 1)

    det_Tgt_i: Detection score
    '''

    xhat_Tgt_i, wii = AOLfusionLayer2(xhatI_Tgt_i, xhatI_Tgt_ij, what_ii, what_ij)
    print(xhat_Tgt_i, wii)

    wii_prev = 0.3
    alpha_i_prev = 0.1
    det_Tgt_i_prev = 0.8

    Del_wi = wii - wii_prev
    Del_alphai = alpha_i - alpha_i_prev
    Del_det_Tgt_i = det_Tgt_i - det_Tgt_i_prev

    wii_prev = wii
    alpha_i_prev = alpha_i

    t_count = 20
    xhat_Tgt_j_prev = numpy.array([[1.5,1.5],[2.1,7.1],[8.8,-1.1]])

    alpha_hat_i_nxt, alphapr_hat_i_nxt, what_i_nxt = LearningPhaseAOLver1(t_count, det_Tgt_i, xhatI_Tgt_ij, xhatI_Tgt_i, xhatS_Tgt_i, xhat_Tgt_i_prev, xhat_Tgt_j_prev, what_ij, what_ii, alpha_hat_i, alphapr_hat_i)
    print(alpha_hat_i_nxt, alphapr_hat_i_nxt, what_i_nxt)

    alpha_hat_i_nxt, alphapr_hat_i_nxt, what_i_nxt = LearningPhaseAOLver2(t_count, det_Tgt_i, what_ii, alpha_hat_i, alphapr_hat_i, Del_alphai, Del_det_Tgt_i, alpha_i)
    print(alpha_hat_i_nxt, alphapr_hat_i_nxt, what_i_nxt)

    alpha_hat_i_nxt, alphapr_hat_i_nxt, what_i_nxt  = LearningPhaseAOLver3(t_count, det_Tgt_i, what_ii, alpha_hat_i, alphapr_hat_i, Del_alphai, Del_det_Tgt_i, alpha_i, Del_wi, wii)
    print(alpha_hat_i_nxt, alphapr_hat_i_nxt, what_i_nxt)

if __name__ =="__main__":
    main()
