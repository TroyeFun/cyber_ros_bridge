#!/usr/bin/env python3

import numpy as np
import rospy
from pycyber import cyber
import ipdb
from modules.drivers.proto.pointcloud_pb2 import PointCloud 
from modules.drivers.proto.pointcloud_pb2 import PointXYZIT
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
# from bitstream import BitStream
import struct
import faster_for as ff


        threads = []
        for i in range(3):
            t = threading.Thread(target=ff.faster_for, args=[p123.point[0:100000]])
            threads.append(t)
            t.start()
        
        results = list()
        for t in threads:
            results.append(t.join())


        # process_num = math.ceil(len(p123.point) / point_num_per_process)
        # input_list = []
        # for pi in range(process_num-1):
        #     input_list.append(p123.point[pi * point_num_per_process : (pi+1) * point_num_per_process])
        # input_list.append(p123.point[(process_num-1) * point_num_per_process ::])

        # # ipdb.set_trace()

        # result = pool.map(ff.faster_for, input_list)


# class cyber_ros_bridge:
#     def __init__(self):
#         self.ros_publisher = None
#     def call_back(self, cyber_pc):
#         ros_pc = PointCloud2()
#         points = cyber_pc.point
#         ros_pc = point_cloud2.create_cloud(ros_pc.header, ['x', 'y', 'z','intensity','timestamp'], points)

#         ros.points
#         # TODO
#         for point in cyber_pc.points:

#             point.x
#             point.y
#             point.z
#             point.i
#             point.t
#             ros_point
#             ros.points.append(ros_point)
#         ros_pc.points = ros.points
#         ros_pc = cyber_pc.points

#         self.ros_publisher.pub(ros_pc)



# def callback(data):
#     rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

# def listener():
#     rospy.init_node('my_listener', anonymous=True)
#     rospy.Subscriber('my_topic', String, callback)
#     rospy.spin()
        
    # for i, p in enumerate(p123.point):
    #     points[i]["x"] = p.x
    #     points[i]["y"] = p.y
    #     points[i]["z"] = p.z
    #     points[i]["intensity"] = p.intensity# uint32 to float
    #     # points[i]["t"] = p.timestamp # uint64 to 32
    
# return a numpy of p2's points 
# def slow_for(p2):
#     for i, p in enumerate(p2):
#         p1[i]["x"] = p.x
#         p1[i]["y"] = p.y
#         p1[i]["z"] = p.z
#         p1[i]["intensity"] = p.intensity# uint32 to float
#         # points[i]["t"] = p.timestamp # uint64 to 32
#     return p1

def pmap(p):
    return (p.x, p.y, p.z, np.float32(p.intensity), np.uint32(p.timestamp))
    



if __name__ == '__main__':
    p123 = PointCloud()

    pc_num = 300000
    
    p = PointXYZIT()
    p.x = 1
    p.y = 1
    p.z = 1
    p.intensity = 300
    p.timestamp = 300
    p123.point.extend(pc_num*[p])
    
    import time
    
    test_round_times = 10
    
    t1 = time.time()
    
    ros_pc = PointCloud2()
    ros_pc.fields = [
    PointField('x', 0, PointField.FLOAT32, 1),
    PointField('y', 4, PointField.FLOAT32, 1),
    PointField('z', 8, PointField.FLOAT32, 1),
    PointField('intensity', 12, PointField.FLOAT32, 1),
    PointField('t', 16, PointField.UINT32, 1), 
    ]
    ros_pc.point_step = 20
    ros_pc.is_bigendian = False
    # plen = len(p123.point)
    for i in range(test_round_times):
        cyber_time = p123.header.lidar_timestamp # nano sec
        ros_pc.header.stamp = rospy.rostime.Time.from_sec(cyber_time / 1e9)
        ros_pc.header.seq = p123.header.sequence_num
        ros_pc.height = p123.height
        ros_pc.width = p123.width
        ros_pc.row_step = ros_pc.point_step * len(p123.point)
        ros_pc.is_dense = p123.is_dense
        ros_pc.header.frame_id = p123.frame_id
        
        # points = np.zeros( len(p123.point), \
        # dtype={ 
        #     "names": ( "x", "y", "z", "intensity", "t"), 
        #     "formats": ( "f4", "f4", "f4", "f4", "u4")} )
        # p_list = p123.point[::]
        # p_np = np.array(p_list)
        # pg = map(pmap, p_list)
        # for i, p in enumerate(pg):
        #     points[i] = p
        # for ii in range(100000):
        # p123.point[plen-1],
        
        points = ff.faster_for(p123.point)
        # for i, p in enumerate(p123.point):      
        #     points[i] = (p.x, p.y, p.z, np.float32(p.intensity), np.uint32(p.timestamp))
        
        
            
        
        ros_pc.data = points.tostring()

    t2 = time.time()
    
    print("Hz = " + str(test_round_times / (t2-t1)))
    
        
    ipdb.set_trace()
    # rospy.init_node('cyber_ros_bridge', anonymous=True)
    # cyber.init()
    # cyber_node = cyber.Node("cyber_ros_bridge")

    # cyber_ros_bridger = cyber_ros_bridge()
    # cyber_ros_bridger.ros_publisher = rospy.Publisher("/********", PointCloud2, queue_size=100)

    # cyber_node.create_reader("/******", PointCloud, cyber_ros_bridger.callback)

    # cyber_node.spin()
    # rospy.spin()