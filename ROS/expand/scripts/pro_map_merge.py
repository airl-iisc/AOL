#!/usr/bin/env python3

import rospy
import numpy as np
from numpy.lib.stride_tricks import as_strided
from nav_msgs.msg import OccupancyGrid, MapMetaData
# import sys
# np.set_printoptions(threshold=sys.maxsize)

BOT_NUM = 4
MAP_RES = 0.05

def sar(ind): #shift scale and round
    return  1000 + round(ind/MAP_RES)
    

def eo(ind):
     return ind*MAP_RES

def occupancygrid_to_numpy(msg):
        data = np.asarray(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
        # return np.ma.array(data, mask=data==-1, fill_value=-1)
        return data, msg.info


def numpy_to_occupancy_grid(arr, info=None):
        if not len(arr.shape) == 2:
                raise TypeError('Array must be 2D')
        if not arr.dtype == np.int8:
                raise TypeError('Array must be of int8s')

        grid = OccupancyGrid()
        if isinstance(arr, np.ma.MaskedArray):
                # We assume that the masked value are already -1, for speed
                arr = arr.data
        grid.data = arr.ravel()
        grid.info = info or MapMetaData()
        grid.info.height = arr.shape[0]
        grid.info.width = arr.shape[1]

        return grid

class pro_map_merge:

    def __init__(self):
        
        self.merged_map = np.full((2000,2000),-1,dtype=np.int8)

        self.ip = []
        self.map1 = None
        self.map2 = None
        self.map3 = None
        self.map4 = None
        self.m1i = MapMetaData()
        self.m2i = MapMetaData()
        self.m3i = MapMetaData()
        self.m4i = MapMetaData()

        self.mergedmapinfo = MapMetaData()
        self.mergedmapinfo.resolution = MAP_RES
        self.mergedmapinfo.origin.position.x = -50
        self.mergedmapinfo.origin.position.y = -50 

        rospy.init_node('pro_map_merge_node', anonymous=True)
        self.loop_rate = rospy.Rate(1)


        for i in range(1,BOT_NUM+1):
            self.ip.append([rospy.get_param("/robot{}/map_merge/init_pose_x".format(i)),
                                    rospy.get_param("/robot{}/map_merge/init_pose_y".format(i)),
                                    rospy.get_param("/robot{}/map_merge/init_pose_z".format(i)),
                                    rospy.get_param("/robot{}/map_merge/init_pose_yaw".format(i))])

        rospy.Subscriber("robot1/map", OccupancyGrid, self.map1_callback)
        rospy.Subscriber("robot2/map", OccupancyGrid, self.map2_callback)
        rospy.Subscriber("robot3/map", OccupancyGrid, self.map3_callback)
        rospy.Subscriber("robot4/map", OccupancyGrid, self.map4_callback)

        self.pub = rospy.Publisher('merged_output',OccupancyGrid,queue_size=10)
        
        rospy.sleep(3)
        self.merge_maps()
        

    def merge_maps(self):

        while not rospy.is_shutdown():
            self.merger()
            # if self.merged_map is not None:
            message = numpy_to_occupancy_grid(self.merged_map,self.mergedmapinfo)
            message.header.stamp = rospy.Time.now()
            message.header.frame_id = 'robot1/map'
            self.pub.publish(message)
            self.loop_rate.sleep()

    def map1_callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + " Updated map 1")
        self.map1, self.m1i = occupancygrid_to_numpy(data)

    def map2_callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + " Updated map 2")
        self.map2, self.m2i = occupancygrid_to_numpy(data)

    def map3_callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + " Updated map 3")
        self.map3, self.m3i = occupancygrid_to_numpy(data)

    def map4_callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + " Updated map 4")
        self.map4, self.m4i = occupancygrid_to_numpy(data)


    def merger(self):
        #  if self.map1 is not None and self.map2 is not None and self.map3 is not None and self.map4 is not None:
        # self.merged_map[sar(self.init_poses[0][1]):sar(self.init_poses[0][1])+self.map1info.height,sar(self.init_poses[0][0]):sar(self.init_poses[0][0])+self.map1info.width] += self.map1 + 1 #np.clip(self.map1 + 1,a_min=0,a_max=100)
        # self.merged_map[sar(self.init_poses[1][1]):sar(self.init_poses[1][1])+self.map2info.height,sar(self.init_poses[1][0]):sar(self.init_poses[1][0])+self.map2info.width] += self.map2 + 1 #np.clip(self.map2 + 1,a_min=0,a_max=100)
        # self.merged_map[sar(self.init_poses[2][1]):sar(self.init_poses[2][1])+self.map3info.height,sar(self.init_poses[2][0]):sar(self.init_poses[2][0])+self.map3info.width] += self.map3 + 1 #np.clip(self.map3 + 1,a_min=0,a_max=100)
        # self.merged_map[sar(self.init_poses[3][1]):sar(self.init_poses[3][1])+self.map4info.height,sar(self.init_poses[3][0]):sar(self.init_poses[3][0])+self.map4info.width] += self.map4 + 1 #np.clip(self.map4 + 1,a_min=0,a_max=100)

        # of1y =  - eo(self.m1i.height) - self.m1i.origin.position.y 
        # of2y =  - eo(self.m2i.height) - self.m2i.origin.position.y 
        # of3y =  - eo(self.m3i.height) - self.m3i.origin.position.y 
        # of4y =  - eo(self.m4i.height) - self.m4i.origin.position.y 

        # of1x =  - eo(self.m1i.width) - self.m1i.origin.position.x 
        # of2x =  - eo(self.m2i.width) - self.m2i.origin.position.x 
        # of3x =  - eo(self.m3i.width) - self.m3i.origin.position.x 
        # of4x =  - eo(self.m4i.width) - self.m4i.origin.position.x 

        of1y =  self.m1i.origin.position.y + 0.8 #eo(self.m1i.height) + 
        of2y =  self.m2i.origin.position.y + 0.8 #eo(self.m2i.height) + 
        of3y =  self.m3i.origin.position.y + 0.8 #eo(self.m3i.height) +
        of4y =  self.m4i.origin.position.y + 0.8 #eo(self.m4i.height) + 

        of1x =  self.m1i.origin.position.x + 0.8 #eo(self.m1i.width) +
        of2x =  self.m2i.origin.position.x + 0.8 #eo(self.m2i.width) +
        of3x =  self.m3i.origin.position.x + 0.8 #eo(self.m3i.width) +
        of4x =  self.m4i.origin.position.x + 0.8 #eo(self.m4i.width) +


        self.merged_map[sar(self.ip[0][1] + of1y):sar(self.ip[0][1] + of1y)+self.m1i.height,sar(self.ip[0][0] + of1x):sar(self.ip[0][0] + of1x)+self.m1i.width] += self.map1 + 1
        self.merged_map[sar(self.ip[1][1] + of2y):sar(self.ip[1][1] + of2y)+self.m2i.height,sar(self.ip[1][0] + of2x):sar(self.ip[1][0] + of2x)+self.m2i.width] += self.map2 + 1
        self.merged_map[sar(self.ip[2][1] + of3y):sar(self.ip[2][1] + of3y)+self.m3i.height,sar(self.ip[2][0] + of3x):sar(self.ip[2][0] + of3x)+self.m3i.width] += self.map3 + 1
        self.merged_map[sar(self.ip[3][1] + of4y):sar(self.ip[3][1] + of4y)+self.m4i.height,sar(self.ip[3][0] + of4x):sar(self.ip[3][0] + of4x)+self.m4i.width] += self.map4 + 1        

        # self.merged_map = np.subtract(self.merged_map,np.multiply(np.logical_and(self.merged_map > 0,  self.merged_map < 200).astype(float),self.merged_map)) # 102 is the exception case where 2 are observing environment and one says empty one says full so we prefer full
        self.merged_map = np.subtract(self.merged_map,np.multiply(np.logical_and(self.merged_map > 0,  self.merged_map < 99, self.merged_map != 101).astype(np.int8),self.merged_map))
        self.merged_map = np.clip(self.merged_map,a_min=-1,a_max=100)
        
if __name__ == '__main__':
    promm = pro_map_merge()