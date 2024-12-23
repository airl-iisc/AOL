import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid
import matplotlib.pyplot as plt
from skimage.morphology import skeletonize, binary_dilation, disk

class OccupancyGridSubscriber:
    def __init__(self, r):
        self.sub = rospy.Subscriber('/map', OccupancyGrid, self.callback)
        self.skeleton = None
        self.r = r

    def callback(self, data):
        # convert the data to a numpy array
        grid = np.array(data.data).reshape((data.info.height, data.info.width))

        # Crop the grid where its value is -1
        y,x = np.where(grid != -1)
        cropped_grid = grid[np.min(y):np.max(y), np.min(x):np.max(x)]
        
        # apply a threshold only on the part of the grid that is not -1
        binary_grid = np.where(cropped_grid > 50, 1, 0)
        # mirror the grid vertically
        binary_grid = np.flipud(binary_grid)
        # inflate the binary grid
        inflated_binary_grid = binary_dilation(binary_grid, disk(self.r))
        # invert the binary grid
        inflated_binary_grid = np.where(inflated_binary_grid == 1, 0, 1)
        # skeletonize the binary grid
        self.skeleton = skeletonize(inflated_binary_grid)

if __name__ == '__main__':
    rospy.init_node('occupancy_grid_subscriber')
    r = 8 # set the radius
    ogs = OccupancyGridSubscriber(r)
    while not rospy.is_shutdown():
        if ogs.skeleton is not None:
            plt.imshow(ogs.skeleton, cmap='gray')
            plt.show()
            ogs.skeleton = None # reset the skeleton 
    rospy.spin()
