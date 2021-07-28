#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from autoware_perception_msgs.msg.object_recognition import DynamicObjectWithFeatureArray # find documentation autoware_perception_msgs


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "Cluster data %s", data.data)
    
def listener():

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("output/labeled_clusters", DynamicObjectWithFeatureArray, callback) # output/labeled_clusters

    counter = 0;
    stamp_sec_prev = 0;
    stamp_nsec_prev = 0;

    if stamp_sec_prev == stamp_nsec_prev == 0: # No previous object has been detected yet -> keep account of time of 1st object
        stamp_sec_prev = data.header.stamp.sec
        stamp_nsec_prev = data.header.stamp.nsec
        counter = counter + 1
    elif stamp_sec_prev - msg.header.stamp.sec <= 1: # new object detected within 1 second -> need to check if it is the same object
        counter = counter + 1 # change (calculate previous distance and new distance -> check if within the bounds -> if yes = same object) 
    else:stamp_sec_prev - msg.header.stamp.sec <= 1 
        stamp_sec_prev = 0
        stamp_nsec_prev = 0


    rospy.spin()

if __name__ == '__main__':
    listener()