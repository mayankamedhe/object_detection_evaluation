#!/usr/bin/env python
import rospy
from std_msgs.msg import String
# from autoware_perception_msgs.msg.object_recognition import DynamicObjectWithFeatureArray # find documentation autoware_perception_msgs
from visualization_msgs.msg import MarkerArray



def check_similarity(marker, previous_marker):
    ## calcualte absolute pose of markers 
    ## if difference is negligible -> same obj


def callback(marker):

    previous_marker = NULL
    detected_object = {}
    object_ID = 0

    if marker != NULL:
        if previous_marker != NULL:
            if check_similarity(marker, previous_marker) == 0:
                object_ID = object_ID + 1

            ## update detected object info
            detected_object['object_ID'] = object_ID
            detected_object['timestamp'] = marker.stamp
            detected_object['pose'] = marker.pose

            ## move to next marker
            previous_marker = marker

    self.path_publisher.publish(detected_object)



    # rospy.loginfo(rospy.get_caller_id() + "Cluster data %s", data.data)
    # counter = 0;
    # stamp_sec_prev = 0;
    # stamp_nsec_prev = 0;

    # if stamp_sec_prev == stamp_nsec_prev == 0: # No previous object has been detected yet -> keep account of time of 1st object
    #     stamp_sec_prev = data.header.stamp.sec
    #     stamp_nsec_prev = data.header.stamp.nsec
    #     counter = counter + 1
    # elif stamp_sec_prev - msg.header.stamp.sec <= 1: # new object detected within 1 second -> need to check if it is the same object
    #     counter = counter + 1 # change (calculate previous distance and new distance -> check if within the bounds -> if yes = same object) 
    # else:
    #     stamp_sec_prev = 0
    #     stamp_nsec_prev = 0

    ## publish only certain fields- object pos, color, timestamp etc
    self.path_publisher.publish(marker.data)
    
def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/perception/object_recognition/detection/objects/visualization", MarkerArray, callback) # output/labeled_clusters

    self.path_publisher = rospy.Publisher("object_detection/counter", MarkerArray, queue_size=1)

    rospy.spin()


if __name__ == '__main__':
    listener()