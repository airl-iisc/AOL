#!env/usr/bin/env python3
# import numpy as np

# merged_map = np.random.rand(5,5)* 10
# result = merged_map - np.multiply(np.logical_and(merged_map > 0,  merged_map < 4).astype(float),merged_map)
# # print(merged_map)

# input1 = float(input()) 
# input2 = float(input()) 

# result = [input1-10.48039,input2-(-4.5)]
# print(result)

# import scipy
# import matplotlib.pyplot as plt
# a = []
# b = [1] #[2,4,1,7,5,6,0,9,8,10,3]
# #for i in range(10):
# # den = scipy.stats.norm(1,1).pdf(1)
# for i in range(10):
#     a.append(scipy.stats.norm(i,1).pdf(5))
# print(a)
# print(len(a))

# plt.plot(a)
# plt.show()

# import numpy as np
 
# # Author: Addison Sears-Collins
# # https://automaticaddison.com
# # Description: A state space model for a differential drive mobile robot
 
# # A matrix
# # 3x3 matrix -> number of states x number of states matrix
# # Expresses how the state of the system [x,y,yaw] changes 
# # from t-1 to t when no control command is executed.
# # Typically a robot on wheels only drives when the wheels are commanded
# # to turn.
# # For this case, A is the identity matrix.
# # A is sometimes F in the literature.
# A_t_minus_1 = np.array([[1.0,  0,   0],
#                         [  0,1.0,   0],
#                         [  0,  0, 1.0]])
                         
# # The estimated state vector at time t-1 in the global 
# # reference frame
# # [x_t_minus_1, y_t_minus_1, yaw_t_minus_1]
# # [meters, meters, radians]
# state_estimate_t_minus_1 = np.array([0.0,0.0,0.0])
 
# # The control input vector at time t-1 in the global 
# # reference frame
# # [v, yaw_rate]
# # [meters/second, radians/second]
# # In the literature, this is commonly u.
# control_vector_t_minus_1 = np.array([4.5, 0.05])
 
# # Noise applied to the forward kinematics (calculation
# # of the estimated state at time t from the state
# # transition model of the mobile robot). This is a vector
# # with the number of elements equal to the number of states
# process_noise_v_t_minus_1 = np.array([0.01,0.01,0.003])
 
# yaw_angle = 0.0 # radians
# delta_t = 1.0 # seconds
 
# def getB(yaw,dt):
#   """
#   Calculates and returns the B matrix
#   3x2 matix -> number of states x number of control inputs
#   The control inputs are the forward speed and the
#   rotation rate around the z axis from the x-axis in the 
#   counterclockwise direction.
#   [v, yaw_rate]
#   Expresses how the state of the system [x,y,yaw] changes
#   from t-1 to t due to the control commands (i.e. control
#   input).
#   :param yaw: The yaw (rotation angle around the z axis) in rad 
#   :param dt: The change in time from time step t-1 to t in sec
#     """
#   B = np.array([[np.cos(yaw)*dt, 0],
#                 [np.sin(yaw)*dt, 0],
#                 [0, dt]])
#   return B
 
# def main():
#   state_estimate_t = A_t_minus_1 @ (
#     state_estimate_t_minus_1) + (
#     getB(yaw_angle, delta_t)) @ (
#     control_vector_t_minus_1) + (
#     process_noise_v_t_minus_1)
 
#   print(f'State at time t-1: {state_estimate_t_minus_1}')
#   print(f'Control input at time t-1: {control_vector_t_minus_1}')
#   print(f'State at time t: {state_estimate_t}') # State after delta_t seconds
 
# main()
# import numpy as np
# from numpy import linalg as LA

# a = np.array([[3,4.2],[3,4.2],[3.1,4],[3,8],[3,11]])
# b = np.array([3,4])

# idx = (LA.norm(a-b,axis=1)).argmin()
# print(idx)


# print(a-b)
# print(LA.norm(a-b))

# Normalization of the state matrix

# import numpy as np
#  a = np.array([[5, 10, 3, 1],[5, 11, 3, 1],[5, 12, 3, 1],[5, 13, 3, 1]],dtype=float)
# a[:,3] = np.divide(a[:,3], np.linalg.norm(a[:,3],1))
# print(a[:,3])
# print(np.sum(a[:,:2]*a[:,3,np.newaxis],axis = 0))

# Averaging Quaternions
# import numpy as np
# from tf.transformations import euler_from_quaternion, quaternion_from_euler

# def weighted_average_quaternions(quaternions, weights):
#     """
#     Average multiple quaternions with specific weights

#     :params quaternions: is a Nx4 numpy matrix and contains the quaternions
#         to average in the rows.
#         The quaternions are arranged as (w,x,y,z), with w being the scalar

#     :params weights: The weight vector w must be of the same length as
#         the number of rows in the

#     :returns: the average quaternion of the input. Note that the signs
#         of the output quaternion can be reversed, since q and -q
#         describe the same orientation
#     :raises: ValueError if all weights are zero
#     """
#     # Number of quaternions to average
#     samples = quaternions.shape[0]
#     mat_a = np.zeros(shape=(4, 4), dtype=np.float64)
#     weight_sum = 0

#     for i in range(0, samples):
#         quat = quaternions[i, :]
#         mat_a = weights[i] * np.outer(quat, quat) + mat_a
#         weight_sum += weights[i]

#     if weight_sum <= 0.0:
#         raise ValueError("At least one weight must be greater than zero")

#     # scale
#     mat_a = (1.0/weight_sum) * mat_a

#     # compute eigenvalues and -vectors
#     eigen_values, eigen_vectors = np.linalg.eig(mat_a)

#     # Sort by largest eigenvalue
#     eigen_vectors = eigen_vectors[:, eigen_values.argsort()[::-1]]

#     # return the real part of the largest eigenvector (has only real part)
#     return np.real(np.ravel(eigen_vectors[:, 0]))

# def qfe(theta):
#     q = quaternion_from_euler(0,0,theta)
#     return q

# a = np.array([[5, 10, 3.14, 1],[5, 11, 3.14, 2],[5, 12, 3.14, 1],[5, 13, 3.14, 1]],dtype=float)

# quat_list = np.array([qfe(xi) for xi in a[:,2]])
# print(quat_list)

# final_pose = np.zeros((3,1),dtype=float) 
# print(final_pose[:2])
# theta = np.pi/180 * 90
# quat = quaternion_from_euler(0,0,theta)
# print(quat)

# theta2 = np.pi/180 * -45
# quat2 =quaternion_from_euler(0,0,theta2)
# print(quat2)

# # quat3 = (0.5*np.absolute(quat) + 0.5*np.absolute(quat2) )
# # print(np.linalg.norm(quat+ quat2))
# quatstack = np.vstack((quat,quat2))
# print(quatstack)
# weights = [5,4]
# quat3 = weighted_average_quaternions(quatstack,weights)
# print(quat3)

# _,_,theta_result = euler_from_quaternion(quat3)
# print(theta_result*180/np.pi)

import numpy as np
import math
def weighter(current_state, predicted_state):
    diff = np.subtract(predicted_state,current_state)
    print(diff)
    error = np.linalg.norm(diff,1,axis = 1) # [x_pred y_pred theta_pred]-[x y theta]
    print(error)
    # weight = clip(math.exp(-error))
    # return weight 

def clip(data):
    if data > 1e6:
        data = 1e6
    if data < 0 :
        data = 0 
    return data

state_matrix_t = np.array([[5, 10, 3.14, 1,1],[5, 11, 3.14, 2,2],[5, 12, 3.14, 1,1],[5, 13, 3.14, 1,1]],dtype=float)
state_matrix_t_predicted = np.array([4,10,1.57])
# print(state_matrix_t[:,:3])
print(weighter(state_matrix_t[:,:3],state_matrix_t_predicted))



