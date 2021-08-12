#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from autoware_perception_msgs.msg import DynamicObjectWithFeatureArray
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2

path_publisher = None
previous_object_list = None
object_ID = 0
error = 0.1
vehicle_velocity = None

counter = 0
previous_clusters_list = None
global_object_list = {}


### Function to check similarity between 2 objects based on their pose and shape

def check_similarity(current_object, previous_object, time_elapsed):

    detected_object_info = [0, [0,0,0],[0,0,0]] ## [flag, pose, shape] where flag = 1 if objects are same

    pose_current = [current_object.object.state.pose_covariance.pose.position.x, \
                    current_object.object.state.pose_covariance.pose.position.y, \
                    current_object.object.state.pose_covariance.pose.position.z]
    pose_previous = [previous_object.object.state.pose_covariance.pose.position.x, \
                    previous_object.object.state.pose_covariance.pose.position.y, \
                    previous_object.object.state.pose_covariance.pose.position.z]

    shape_current = [current_object.object.shape.dimensions.x, \
                    current_object.object.shape.dimensions.y, \
                    current_object.object.shape.dimensions.z]
    shape_previous = [previous_object.object.shape.dimensions.x, \
                     previous_object.object.shape.dimensions.y, \
                     previous_object.object.shape.dimensions.z]

    if abs(pose_current[0] - pose_previous[0]) <= error and \
       abs(pose_current[2] - pose_previous[2]) <= error and \
       abs(pose_current[1] - (pose_previous[1] - (vehicle_velocity*time_elapsed))) <= error and \
       abs(shape_current[0] - shape_previous[0]) <= error and \
       abs(shape_current[1] - shape_previous[1]) <= error and \
       abs(shape_current[2] - shape_previous[2]) <= error:
        
        print("Same objects with pose = " + str(pose_current))
        return [1, pose_current, shape_current]
    
    print("Different object pairs")
    return [0, pose_current, shape_current]


### Callback function of the subscriber which parses the current object list and the previous one to check if the same object was detected consecutively 
### and publishes the object ID, timestamp, pose, and shape. If the object is detected twice, it publishes the info only once


def callback(current_object_list):

    detected_object = {}
    global previous_object_list
    global object_ID


    if current_object_list != None:
        if previous_object_list != None:
            for i in range(len(current_object_list.feature_objects)):
                for j in range(len(previous_object_list.feature_objects)):
                    time_elapsed = ((current_object_list.header.stamp.secs + (current_object_list.header.stamp.nsecs/1000000000)) - (previous_object_list.header.stamp.secs + (previous_object_list.header.stamp.nsecs/1000000000)))
                    if check_similarity(current_object_list.feature_objects[i], previous_object_list.feature_objects[j], time_elapsed)[0] == 0:
                        object_ID = object_ID + 1

                        ## update detected object info
                        detected_object['object_ID'] = object_ID
                        detected_object['timestamp'] = current_object_list.header.stamp
                        detected_object['pose'] = current_object_list.feature_objects[i].object.state.pose_covariance.pose.position
                        detected_object['shape'] = current_object_list.feature_objects[i].object.shape.dimensions

                        # print(detected_object)
                        detected_object_string = str(detected_object)
                        path_publisher.publish(detected_object_string)

        ## move to next object list
        previous_object_list = current_object_list

 
def callback2(velocity_data):
    # print(velocity_data)
    global vehicle_velocity
    vehicle_velocity = velocity_data.twist.linear.x

def check_similarity2(current_object, previous_object, time_elapsed):
    # calcualte absolute pose of markers 
    # if difference is negligible -> same obj

    detected_object_info = [0, [0,0,0],[0,0,0]] ## [flag, pose, shape] where flag = 1 if objects are same

    pose_current = [current_object.object.state.pose_covariance.pose.position.x, \
                    current_object.object.state.pose_covariance.pose.position.y, \
                    current_object.object.state.pose_covariance.pose.position.z]
    pose_previous = [previous_object['pose'].x, \
                     previous_object['pose'].y, \
                     previous_object['pose'].z]

    shape_current = [current_object.object.shape.dimensions.x, \
                     current_object.object.shape.dimensions.y, \
                     current_object.object.shape.dimensions.z]
    shape_previous = [previous_object['shape'].x, \
                      previous_object['shape'].y, \
                      previous_object['shape'].z]

    # print(vehicle_velocity)
    if abs(pose_current[1] - pose_previous[1]) <= error and \
       abs(pose_current[2] - pose_previous[2]) <= error and \
       abs(pose_current[0] - (pose_previous[0] - (vehicle_velocity*time_elapsed))) <= error and \
       abs(shape_current[0] - shape_previous[0]) <= error and \
       abs(shape_current[1] - shape_previous[1]) <= error and \
       abs(shape_current[2] - shape_previous[2]) <= error:
       
        # print("Same objects with pose = " + str(pose_current))
        return 1
    
    # print("Different object pairs")
    return 0


def callback3(current_clusters_list):
    
    if counter == 0:
        for i in range(len(current_clusters_list.feature_objects)):
            
            global_object_list[object_ID]['object_ID'] = object_ID
            global_object_list[object_ID]['timestamp'] = current_clusters_list.header.stamp
            global_object_list[object_ID]['pose'] = current_clusters_list.feature_objects[i].object.state.pose_covariance.pose.position
            global_object_list[object_ID]['shape'] = current_clusters_list.feature_objects[i].object.shape.dimensions
            global_object_list[object_ID]['num of detections'] = 1
            object_ID = object_ID + 1
        counter = counter + 1
        # previous_clusters_list = current_clusters_list
        print("Number of objects detected = " + str(len(current_clusters_list.feature_objects)))

    else:
        for i in range(len(current_clusters_list.feature_objects)):
            for j in range(len(global_object_list)):
                time_elapsed = ((current_clusters_list.header.stamp.secs + (current_object_list.header.stamp.nsecs/1000000000)) - (global_object_list[j]['timestamp'].secs + (global_object_list[j]['timestamp'].nsecs/1000000000)))
                if check_similarity2(current_clusters_list.feature_objects[i], global_object_list[j], time_elapsed): 
                    global_object_list[j]['num of detections'] = global_object_list[j]['num of detections'] + 1
                else:
                    global_object_list[object_ID]['object_ID'] = object_ID
                    global_object_list[object_ID]['timestamp'] = current_clusters_list.header.stamp
                    global_object_list[object_ID]['pose'] = current_clusters_list.feature_objects[i].object.state.pose_covariance.pose.position
                    global_object_list[object_ID]['shape'] = current_clusters_list.feature_objects[i].object.shape.dimensions
                    global_object_list[object_ID]['num of detections'] = 1
                    object_ID = object_ID + 1

        counter = counter + 1

        if counter % 10 == 0:
            global_object_str = str(global_object_list)
            path_publisher.publish(global_object_str)


def listener():
    global path_publisher
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/vehicle/status/twist", TwistStamped, callback2)
    rospy.Subscriber("/labeled_clusters", DynamicObjectWithFeatureArray, callback3)

    # rospy.Subscriber("/objects", DynamicObjectWithFeatureArray, callback) # output/labeled_clusters, /perception/object_recognition/detection/objects/visualization -> MarkerArray,  /lidar_apollo_instance_segmentation/debug/instance_pointcloud -> PointCloud2
    path_publisher = rospy.Publisher("object_detection/counter", String, queue_size=1)
    rospy.spin()

if __name__ == '__main__':
    listener()