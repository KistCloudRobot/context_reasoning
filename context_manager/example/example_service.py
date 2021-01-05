
import rospy
from rosjava_custom_srv.msg import *
from rosjava_custom_srv.srv import *
import time
 
rospy.init_node('context_manager_service')
context_srv = rospy.ServiceProxy("/context_manager/monitor/service", MonitorSimilarService)


def sendmessage(req, tri):
    tri=tri.split(' ')
    req.predicate = tri[0]
    req.param1 = tri[1]
    req.param2 = tri[2]
    req.param3 = tri[3]
    req.param4 = tri[4]
    req.status = 100
    req.manager = "TaskManager"

def run():
    req = MonitorSimilarServiceRequest()
    # on_Physical Top Bottom 0 0 100
    # empty_hand Hand 0 0 0 100
    # opened_hand Hand 0 0 0 100
    # graspedBy Object Hand 0 0 100
    # detected_object Object 0 0 0 100
    # locatedAt Object Place 0 0 100
    # in_ContGeneric Object1 Object2 0 0 100
    # aboveOf Object1 Object2 0 0 100
    # belowOf Object1 Object2 0 0 100
    # inFrontOf Object1 Object2 0 0 100
    # behind Object1 Object2 0 0 100
    # near Object1 Object2 0 0 100
    # empty_container Object 0 0 0 100

    sendmessage(req, 'on_Physical H P 0 0')
    print(req)
    res = context_srv(req)
    print res

    time.sleep(0.05)

    sendmessage(req, 'closed_hand H 0 0 0')
    print(req)
    res = context_srv(req)
    print res

    time.sleep(0.05)

    sendmessage(req, 'empty_hand H 0 0 0')
    print(req)
    res = context_srv(req)
    print res

    time.sleep(0.05)
 
    '''
    sendmessage(req, 'nb_current H P 0 0')
    print(req)
    res = context_srv(req)
    print res

    
    sendmessage(req, 'currentObjectPose H P 0 0')
    print(req)
    res = context_srv(req)
    print res


    sendmessage(req, 'currentHandPose H P 0 0')
    print(req)
    res = context_srv(req)
    print res


    sendmessage(req, 'behind H P 0 0')
    print(req)
    res = context_srv(req)
    print res


    sendmessage(req, 'empty_hand H 0 0 0')
    print(req)
    res = context_srv(req)
    print res
    '''
    


run()

