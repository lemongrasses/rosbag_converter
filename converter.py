import numpy as np
import open3d as o3d
import rosbag
import cv2
import sensor_msgs
import nav_msgs
import sensor_msgs.point_cloud2 as pc2
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError
from datetime import datetime



class Converter():
    def __init__(self) -> None:
        pass

    def _CompressedImage_convert(self, msg):
        cv_image = None
        try:
            # cv_image = self.bridge.imgmsg_to_cv2(msg.data, "bgr8")
            np_arr = np.fromstring(msg.data, np.ub)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except CvBridgeError as e:
            print(e)

        time =  msg.header.stamp.to_sec()
        timestr = "%.9f" % msg.header.stamp.to_sec()

        image_name = timestr + ".png"
        return [time, image_name, cv_image, 'compressedimage']
    
    def _Image_convert(self, msg):
        bridge = CvBridge()
        cv_image = None
        try:
            cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except CvBridgeError as e:
            print(e)
            
        time =  msg.header.stamp.to_sec()
        timestr = "%.9f" % msg.header.stamp.to_sec()

        image_name = timestr + ".png"
        return [time, image_name, cv_image, 'image']
    
    def _Imu_convert(self, msg):
        time =  msg.header.stamp.to_sec()
        gyro = [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]
        acce = [msg.linear_acceleration.x,  msg.linear_acceleration.y,  msg.linear_acceleration.z]
        
        return [time, gyro, acce, 'imu']

    def _Odom_convert(self, msg):
        time =  msg.header.stamp.to_sec()
        pose = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
        # In order, the parameters are:
        # (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
        orientation = [msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z]
        pos_cov = list(msg.pose.covariance)
        # It is only meant to represent a direction. Therefore, it does not make sense to apply a translation to it
        vel = [msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z]
        ang_vel = [msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z]
        vel_cov = list(msg.twist.covariance)
        return [time, pose, orientation, pos_cov, vel, ang_vel, vel_cov, 'odom']
    
    def _PointCloud_convert(self, msg):
        time = msg.header.stamp.to_sec()
        timestr = "%.9f" % msg.header.stamp.to_sec()
        pcd_name = timestr + ".pcd"
        points = pc2.read_points(msg, skip_nans=True, field_names=("x", "y", "z"))
        pc_list = list()
        for point in points:
            pc_list.append([point[0], point[1], point[2]])

        np.array(pc_list)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pc_list)
        return [time, pcd_name, pcd, 'pcd']
    
    def _Livox_convert(self, msg):
        time = msg.header.stamp.to_sec()

        timestr = "%.9f" % msg.header.stamp.to_sec()
        pcd_name = timestr + ".pcd"
        num_points = msg.point_num
        point_XYZ = np.zeros((num_points,3))
        for idx in range(num_points):
            point_XYZ[idx,0] = msg.points[idx].x
            point_XYZ[idx,1] = msg.points[idx].y
            point_XYZ[idx,2] = msg.points[idx].z
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(point_XYZ)

        return [time, pcd_name, pcd, 'livox']
    
    def convert(self, msg):
        data = tuple()
        if 'CompressedImage' in str(type(msg)):
            data = self._CompressedImage_convert(msg)
        elif 'Image' in str(type(msg)):
            data = self._Image_convert(msg)
        elif 'Imu' in str(type(msg)):
            data = self._Imu_convert(msg)
        elif 'Odometry' in str(type(msg)):
            data = self._Odom_convert(msg)
        elif 'PointCloud2' in str(type(msg)):
            data = self._PointCloud_convert(msg)
        elif 'livox' in str(type(msg)):
            data = self._Livox_convert(msg)
        return data