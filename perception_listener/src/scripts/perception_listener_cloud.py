#!/usr/bin/env python
import os
import os.path
import sys
import signal
import rospy
import rosparam

from sensor_msgs.msg import *
from vision_msgs.msg import *
from geometry_msgs.msg import *
from std_msgs.msg import *
from socialrobot_perception_msgs.srv import *
from socialrobot_perception_msgs.msg import *
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class PerceptionListener():
    def __init__(self):
        self.obj_name2id={}

        self.current_obstacles = {}
        self.current_robots = {}
        self.current_joints = {}

        self.robot_count=0

        rospy.Subscriber('/joint_states', JointState, self.callback_joints, queue_size=10)
        rospy.Subscriber('/sim_ros_interface/robots_pose_10', Float64MultiArray, self.callback_robot, queue_size=10)
        rospy.Subscriber('/sim_ros_interface/robots_pose_11', Float64MultiArray, self.callback_robot, queue_size=10)
        rospy.Subscriber('/sim_ros_interface/objects_pose', Objects, self.callback_objects, queue_size=10)
        self.pub_robot = rospy.Publisher("/visual_robot_perceptions", Float32MultiArray, queue_size=10)
        self.pub_objects = rospy.Publisher("/objects_infos", Float32MultiArray, queue_size=10)
        self.pub_joints = rospy.Publisher("/joint_statess", JointState, queue_size=10)

    def prepare(self):
        dir_path = os.path.dirname(os.path.realpath(__file__))+"/"
        with open(dir_path+'object_list.txt','r') as f:
            i=0
            for line in f.readlines():
                tmp=line.lower().strip()
                if not tmp:continue
                self.obj_name2id[tmp]=i;i+=1
        #for k, v in self.obj_name2id.items():
        #    print(k, v)
        

    def publish_objects(self):
        #objs = Float32MultiArray()
        for idx, dat in self.current_obstacles.items():
            #obj = Float32MultiArray()
            obj_info = []


            try:i = self.obj_name2id[dat.name.data]
            except:continue#i = int(dat.name.data.split('_')[-1])
            x = dat.bb3d.center.position.x * 1.0
            y = dat.bb3d.center.position.y * 1.0
            z = dat.bb3d.center.position.z * 1.0
            a = dat.bb3d.center.orientation.x * 1.0
            b = dat.bb3d.center.orientation.y * 1.0
            c = dat.bb3d.center.orientation.z * 1.0
            d = dat.bb3d.center.orientation.w * 1.0
            width = dat.bb3d.size.y * 1.0
            depth = dat.bb3d.size.x * 1.0
            height = dat.bb3d.size.z * 1.0
            
            orientation_list = [a, b, c, d]
            (a, b, c) = euler_from_quaternion(orientation_list)  # object center orientation

            obj_info.append(x)
            obj_info.append(y)
            obj_info.append(z)
            obj_info.append(a)
            obj_info.append(b)
            obj_info.append(c)
            obj_info.append(d)
            obj_info.append(width)
            obj_info.append(depth)
            obj_info.append(height)
            obj_info.append(i)

            #print(obj_info)
                 
            #objs.append(obj)   
            #obj.data = obj_info
            self.pub_objects.publish(data=obj_info)

    def update(self):
        self.publish_objects()
        #self.publish_robot()        

    def callback_objects(self, data):
        self.current_obstacles = {}
        for idx, dat in enumerate(data.detected_objects):
            self.current_obstacles[idx] = dat

    def callback_robot(self, data):
        print(data)
        try:
            robot_info = []
            idx = data.data[6]
            d, w, h = 0, 0, 0

            robot_info.append(data.data[0]) # x
            robot_info.append(data.data[1]) # y
            robot_info.append(data.data[2]) # z
            robot_info.append(data.data[3]) # a
            robot_info.append(data.data[4]) # b
            robot_info.append(data.data[5]) # c
        
            if idx == 10.0:
                d = 5.4177e-01 #0.64001
                w = 5.4177e-01#0.64001
                h = 6.9385e-01#1.20000
            elif idx == 11.0:
                d = 0.05
                w = 0.05
                h = 0.05 # 0.07
            elif idx == 12.0:
                d = 0.05
                w = 0.05
                h = 0.05
            else:
                d = 0
                w = 0
                h = 0

            robot_info.append(d) # d
            robot_info.append(w) # w
            robot_info.append(h) # h

            robot_info.append(data.data[6]) # x

            self.pub_robot.publish(data=robot_info)
        except:pass

    def callback_joints(self, data):
        self.pub_joints.publish(data)
        # LFinger_1,LFinger_2,LFinger_3,RFinger_1,RFinger_2,RFinger_3,fridge_top_joint,fridge_bottom_joint
        #print(data)


##############################
# Main function
##############################
if __name__ == '__main__':
    # ros initialize
    rospy.init_node('perception_listener')        

    # perception manager
    pm = PerceptionListener()
    pm.prepare()

    # Start
    rospy.loginfo('[PerceptionListener] Service Started!')


    loop_freq = 100 # 10hz
    r = rospy.Rate(loop_freq)
    while not rospy.is_shutdown():
        pm.update()
        r.sleep()
