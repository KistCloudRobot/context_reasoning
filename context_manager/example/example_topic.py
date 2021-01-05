import rospy
from rosjava_custom_srv.msg import *

pub = rospy.Publisher('/context_manager/monitor/reception', MonitorServiceRequest, queue_size=10)
rospy.init_node('context_manager_publisher')
r = rospy.Rate(10) # 10hz
while not rospy.is_shutdown():
   req = MonitorServiceRequest()
   
   req.predicate = 'locatedAt'
   req.param1 = 'Object'
   req.param2 = 'Location'
   req.param3 = '0'
   req.param4 = '0'

   req.status = 100
   req.manager = 'TaskManager'



   pub.publish(req)
   r.sleep()
