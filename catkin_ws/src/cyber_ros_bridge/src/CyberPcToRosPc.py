#!/usr/bin/env python3
import numpy as np
import rospy
from pycyber import cyber
import ipdb
from modules.drivers.proto.pointcloud_pb2 import PointCloud 
from modules.drivers.proto.pointcloud_pb2 import PointXYZIT
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
import struct
import faster_for as ff
import math
import time
import threading

class cyber_ros_pc_bridge:
    def __init__(self, ros_publisher):
        self.ros_publisher = ros_publisher
        self.ros_pc = PointCloud2()
        self.ros_pc.fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1),
        PointField('intensity', 12, PointField.FLOAT32, 1), #32
        PointField('t', 16, PointField.UINT32, 1),  #64
        ]
        self.ros_pc.point_step = 20
        self.ros_pc.is_bigendian = False

    def call_back(self, cyber_pc):
        self.ros_pc_update(cyber_pc)
        self.ros_publisher.pub(ros_pc)

    def ros_pc_update(self, cyber_pc):
        cyber_time = cyber_pc.header.lidar_timestamp # nano sec
        self.ros_pc.header.stamp = rospy.rostime.Time.from_sec(cyber_time / 1e9)
        self.ros_pc.header.seq = cyber_pc.header.sequence_num
        self.ros_pc.height = cyber_pc.height
        self.ros_pc.width = cyber_pc.width
        self.ros_pc.row_step = ros_pc.point_step * len(cyber_pc.point)
        self.ros_pc.is_dense = cyber_pc.is_dense
        self.ros_pc.header.frame_id = cyber_pc.frame_id

        self.ros_pc.data = ff.faster_for(cyber_pc.point).tostring()

if __name__ == '__main__':
    rospy.init_node('cyber_ros_pc_bridge', anonymous=True)
    cyber.init()
    cyber_node = cyber.Node("cyber_ros_pc_bridge")

    cyber_ros_pc_bridger = cyber_ros_pc_bridge(rospy.Publisher("/velodyne_points", PointCloud2, queue_size=100))
    cyber_node.create_reader("/velodyne_points", PointCloud, cyber_ros_pc_bridger.call_back)

    cyber_node.spin()
    rospy.spin()


    