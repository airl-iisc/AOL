#!usr/bin/env python3
import numpy as np
import math 
from tf.transformations import quaternion_from_euler,euler_from_quaternion


class POCO:

    def __init__(self,robot_id,initial_state):
        # Initialize the required data
        self.estimate_list = np.zeros((5,3),dtype=float) # Array of three estimates (SLAM, LiDAR, Predicted) [x, y, theta, weightxy, weighttheta]
        self.final_pose = np.zeros((3,1),dtype=float) # Final Pose for the robot
        self.rid = robot_id
        self.init_state = initial_state
        self.state_t_minus_1 = np.zeros((3,1),dtype=float)

    def qfe(theta):
        q = quaternion_from_euler(0,0,theta)
        return q
    
    def efq(q):
        _,_,theta = euler_from_quaternion(q)
        return theta
    
    def clip(self,data):
        if data > 1e6:
            data = 1e6
        if data < 0 :
            data = 0 
        return data

    def rushB(yaw,dt):
        B = np.array([[np.cos(yaw)*dt, 0],
                        [np.sin(yaw)*dt, 0],
                        [0, dt]])
        return B

    def predictor(state_t_minus_1, dt, control_vector_t_minus_1,self):
        state_estimate_t = (state_t_minus_1) + np.matmul((self.rushB(state_t_minus_1[2], dt)), (control_vector_t_minus_1)) 
        return state_estimate_t
    
    def weighter(current_state, predicted_state,self):
        error = np.linalg.norm(predicted_state-current_state) # [x_pred y_pred theta_pred]-[x y theta]
        weight = self.clip(math.exp(-error))
        return weight       
    
    def normalize_weights(self):
        self.estimate_list = np.divide(self.estimate_list[:,3], np.linalg.norm(self.estimate_list[:,3],1))

    def weighted_average_quaternions(quaternions, weights):
        # Taken from https://scikit-surgerycore.readthedocs.io/en/latest/_modules/sksurgerycore/algorithms/averagequaternions.html#average_quaternions
        samples = quaternions.shape[0]
        mat_a = np.zeros(shape=(4, 4), dtype=np.float64)
        weight_sum = 0

        for i in range(0, samples):
            quat = quaternions[i, :]
            mat_a = weights[i] * np.outer(quat, quat) + mat_a
            weight_sum += weights[i]

        if weight_sum <= 0.0:
            raise ValueError("At least one weight must be greater than zero")

        mat_a = (1.0/weight_sum) * mat_a
        eigen_values, eigen_vectors = np.linalg.eig(mat_a)
        eigen_vectors = eigen_vectors[:, eigen_values.argsort()[::-1]]
        return np.real(np.ravel(eigen_vectors[:, 0]))

    def weighted_average(self):
        # Part 1: Averaging x and y
        self.final_pose[:2] = np.sum(self.estimate_list[:,:2]*self.estimate_list[:,3,np.newaxis],axis = 0)

        # Part 2: Averaging theta
        # Convert thetas to quaternions
        quat_list = np.array([self.qfe(xi) for xi in self.final_pose[:,2]])
        # Apply weighted average
        avg_quat = self.weighted_average_quaternions(quat_list,self.final_pose[:,4])
        _,_,self.final_pose[2] = euler_from_quaternion(avg_quat)

    def algorithm(self,state_t_minus_1,control_vector_t_minus_1,state_t):
        state_estimate_t = self.predictor(state_t_minus_1,control_vector_t_minus_1)
        self.weighter(state_estimate_t,state_t)
        self.normalize_weights()
        self.weighted_average()

    


    

