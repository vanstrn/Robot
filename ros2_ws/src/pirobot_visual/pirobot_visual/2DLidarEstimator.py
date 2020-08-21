
import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np

from sensor_msgs.msg import LaserScan

from sklearn.neighbors import NearestNeighbors

class ContinuousMapping(Node):
    def __init__(self,grid_size=[241,241],grid_dim=0.1):
        """V2 works based on:
        -Always at 0,0
        -Map averaged over a number of cycles to produce a confidence.
        """
        super().__init__('motor_controller')

        #Creating the subscribers that read Lidar points
        self.subscriber = self.create_subscription(LaserScan, "/scan", self.scanCallback,10)

        self.grid_dim=grid_dim
        self.grid_size=grid_size
        #Create a starting location and orientation
        self.loc = [0,0] #
        self.orientation = 0 #radians

        #Creating interfaces to plot the Lidar data
        plt.ion()
        self.fig = plt.figure(figsize=(8,8))
        self.ax1 = self.fig.add_subplot(111)
        self.first = True
        self.pointCloud=[]

    def scanCallback(self,data):
        self.get_logger().info("Begin Scan Localization")
        #Reading the Scan Data
        angle_min = data.angle_min
        angle_max = data.angle_max
        angle_increment = data.angle_increment
        range_min = data.range_min
        range_max = data.range_max
        ranges = data.ranges

        curr_map = []
        angles = np.arange(angle_min,angle_max+angle_increment,angle_increment)
        for dist,angle in zip(ranges,angles):
            if dist > 20:
                continue
            #Absolute Coordinates
            else:
                x_abs = dist*np.cos(angle)
                y_abs = dist*np.sin(angle)
            #Coordinates in the grid
            curr_map.append([x_abs,y_abs])

        if self.first:
            self.pointCloud.extend(curr_map)
            self.first=False
        else:

            #Calculating movement from previous points
            if len(curr_map) >= len(self.prev_map):
                T,dist,i = icp(np.stack(curr_map[:len(self.prev_map)]),np.stack(self.prev_map))
            elif len(curr_map) <= len(self.prev_map):
                T,dist,i = icp(np.stack(curr_map),np.stack(self.prev_map[:len(curr_map)]))
            else:
                T,dist,i = icp(np.stack(curr_map),np.stack(self.prev_map))

            #Updating position
            self.UpdatePosition(T)
            self.get_logger().info("Rotation Matrix: "+ str(T))
            self.get_logger().info("Location: "+ str(self.loc)+"   Orientation:"+str(self.orientation))

            #Mapping points to global coordintates.
            points = self.MapPointsToGlobal(curr_map)

            #Updating the Full
            self.AddPointsToGlobalMap(points)

        #Setting old points for the next iteration.
        self.prev_map = curr_map

        # Distilling points down to a map.
        self.map = np.zeros(self.grid_size)
        for point in self.pointCloud:
            #Coordinates in the grid
            x = round(point[0]/self.grid_dim + self.grid_size[0]/2)
            y = round(point[1]/self.grid_dim + self.grid_size[1]/2)
            #pushing into the grid
            if (x>self.grid_size[0]-1) or (y>self.grid_size[1]-1) or (x<0) or (y<0):
                continue
            self.map[int(x),int(y)] += 1

        #Plotting
        self.get_logger().debug("Updating figure")
        self.ax1.clear()
        self.ax1.imshow(np.clip(self.map,0,20))
        self.fig.canvas.draw()

    def UpdatePosition(self,T):
        x = self.loc[0] + T[0,2]*T[0,0] + T[1,2]*T[0,1]
        y = self.loc[1] + T[0,2]*T[1,0] + T[1,2]*T[1,1]
        self.loc = [x,y]

        if np.abs(np.arcsin(T[0,1])) >= 0.01:
            self.orientation += np.arcsin(T[0,1])

    def MapPointsToGlobal(self,curr_map):
        newPoints = []
        for point in curr_map:
            newPoints.append([  self.loc[0]+point[0]*np.cos(self.orientation)-point[1]*np.sin(self.orientation),
                                self.loc[0]+point[0]*np.sin(self.orientation)-point[1]*np.cos(self.orientation)])
        return newPoints

    def AddPointsToGlobalMap(self,points):
        self.pointCloud.extend(points)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = ContinuousMapping()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


def best_fit_transform(A, B):
    '''
    Calculates the least-squares best-fit transform that maps corresponding points A to B in m spatial dimensions
    Input:
      A: Nxm numpy array of corresponding points
      B: Nxm numpy array of corresponding points
    Returns:
      T: (m+1)x(m+1) homogeneous transformation matrix that maps A on to B
      R: mxm rotation matrix
      t: mx1 translation vector
    '''

    assert A.shape == B.shape

    # get number of dimensions
    m = A.shape[1]

    # translate points to their centroids
    centroid_A = np.mean(A, axis=0)
    centroid_B = np.mean(B, axis=0)
    AA = A - centroid_A
    BB = B - centroid_B

    # rotation matrix
    H = np.dot(AA.T, BB)
    U, S, Vt = np.linalg.svd(H)
    R = np.dot(Vt.T, U.T)

    # special reflection case
    if np.linalg.det(R) < 0:
       Vt[m-1,:] *= -1
       R = np.dot(Vt.T, U.T)

    # translation
    t = centroid_B.T - np.dot(R,centroid_A.T)

    # homogeneous transformation
    T = np.identity(m+1)
    T[:m, :m] = R
    T[:m, m] = t

    return T, R, t


def nearest_neighbor(src, dst):
    '''
    Find the nearest (Euclidean) neighbor in dst for each point in src
    Input:
        src: Nxm array of points
        dst: Nxm array of points
    Output:
        distances: Euclidean distances of the nearest neighbor
        indices: dst indices of the nearest neighbor
    '''

    assert src.shape == dst.shape

    neigh = NearestNeighbors(n_neighbors=1)
    neigh.fit(dst)
    distances, indices = neigh.kneighbors(src, return_distance=True)
    return distances.ravel(), indices.ravel()


def icp(A, B, init_pose=None, max_iterations=20, tolerance=0.001):
    '''
    The Iterative Closest Point method: finds best-fit transform that maps points A on to points B
    Input:
        A: Nxm numpy array of source mD points
        B: Nxm numpy array of destination mD point
        init_pose: (m+1)x(m+1) homogeneous transformation
        max_iterations: exit algorithm after max_iterations
        tolerance: convergence criteria
    Output:
        T: final homogeneous transformation that maps A on to B
        distances: Euclidean distances (errors) of the nearest neighbor
        i: number of iterations to converge
    '''

    assert A.shape == B.shape

    # get number of dimensions
    m = A.shape[1]

    # make points homogeneous, copy them to maintain the originals
    src = np.ones((m+1,A.shape[0]))
    dst = np.ones((m+1,B.shape[0]))
    src[:m,:] = np.copy(A.T)
    dst[:m,:] = np.copy(B.T)

    # apply the initial pose estimation
    if init_pose is not None:
        src = np.dot(init_pose, src)

    prev_error = 0

    for i in range(max_iterations):
        # find the nearest neighbors between the current source and destination points
        distances, indices = nearest_neighbor(src[:m,:].T, dst[:m,:].T)

        # compute the transformation between the current source and nearest destination points
        T,_,_ = best_fit_transform(src[:m,:].T, dst[:m,indices].T)

        # update the current source
        src = np.dot(T, src)

        # check error
        mean_error = np.mean(distances)
        if np.abs(prev_error - mean_error) < tolerance:
            break
        prev_error = mean_error

    # calculate final transformation
    T,_,_ = best_fit_transform(A, src[:m,:].T)

    return T, distances, i

if __name__ == '__main__':
    main()
