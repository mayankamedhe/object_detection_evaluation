#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from autoware_perception_msgs.msg import DynamicObjectWithFeatureArray
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from geometry_msgs.msg import TwistStamped

from detection_evaluation.msg import Message1


path_publisher = None
previous_object_list = None
object_ID = 0
error = 0.1
vehicle_velocity = None

counter = 0
# previous_clusters_list = None
global_object_list = []


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
    
    global counter
    global object_ID
    global global_object_list
    


    if len(current_clusters_list.feature_objects) > 0:
    
        # if counter == 0:
        #     for i in range(len(current_clusters_list.feature_objects)):
        #         temp_object_dict = {'object_ID': object_ID, 'timestamp': current_clusters_list.header.stamp, \
        #         'pose': current_clusters_list.feature_objects[i].object.state.pose_covariance.pose.position, \
        #         'shape': current_clusters_list.feature_objects[i].object.shape.dimensions, 'num of detections': 1}
                
        #         global_object_list.append(temp_object_dict)
        #         object_ID = object_ID + 1
        #     # counter = counter + 1
        #     # previous_clusters_list = current_clusters_list
        #     print(str(counter) + "  Number of objects detected = " + str(len(current_clusters_list.feature_objects)))
        #     # print(global_object_list)

        # else:
        for i in range(len(current_clusters_list.feature_objects)):
            # if len(current_clusters_list.feature_objects) != 0:
            # print(len(global_object_list))
            for j in range(len(global_object_list)):
                time_elapsed = ((current_clusters_list.header.stamp.secs + (current_clusters_list.header.stamp.nsecs/1000000000)) - (global_object_list[j]['timestamp'].secs + (global_object_list[j]['timestamp'].nsecs/1000000000)))
                if check_similarity2(current_clusters_list.feature_objects[i], global_object_list[j], time_elapsed) == 1: ## same object
                    global_object_list[j]['num of detections'] = global_object_list[j]['num of detections'] + 1
                # else:

            temp_object_dict = {'object_ID': object_ID, 'timestamp_s': current_clusters_list.header.stamp.secs, 'timestamp_ns': current_clusters_list.header.stamp.nsecs, \
                                'pose': current_clusters_list.feature_objects[i].object.state.pose_covariance.pose.position, \
                                'num_of_detections': 1}

            global_object_list.append(temp_object_dict)
            object_ID = object_ID + 1
                    # print(object_ID)
                    # print(global_object_list)
            # else:

            #     temp_object_dict = {'object_ID': object_ID, 'timestamp': current_clusters_list.header.stamp, \
            #                         'pose': current_clusters_list.feature_objects[i].object.state.pose_covariance.pose.position, \
            #                         'shape': current_clusters_list.feature_objects[i].object.shape.dimensions, 'num of detections': 1}
            
            #     global_object_list.append(temp_object_dict)
            #     object_ID = object_ID + 1

                # print(global_object_list)
            counter = counter + 1
            
            pub1.object_ID = temp_object_dict['object_ID']
            pub1.timestamp_s = temp_object_dict['timestamp_s']
            pub1.timestamp_ns = temp_object_dict['timestamp_ns']
            pub1.pose = current_clusters_list.feature_objects[i].object.state.pose_covariance
            pub1.num_of_detections = temp_object_dict['num_of_detections']

            path_publisher.publish(pub1)


        # print("In else - " + str(len(current_clusters_list.feature_objects)) + " and " + str(len(global_object_list)))
        # print("COUNTER: " + str(counter))


        # for x in range(len(global_object_list)):
        #     print(global_object_list[x]['object_ID'])


        # print("#####################################################")
        if counter % 10 == 0:
            global_object_str = str(global_object_list)
#             path_publisher.publish(global_object_str)


def listener():
    global path_publisher
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/vehicle/status/twist", TwistStamped, callback2)
    rospy.Subscriber("/labeled_clusters", DynamicObjectWithFeatureArray, callback3)

    # rospy.Subscriber("/objects", DynamicObjectWithFeatureArray, callback) # output/labeled_clusters, /perception/object_recognition/detection/objects/visualization -> MarkerArray,  /lidar_apollo_instance_segmentation/debug/instance_pointcloud -> PointCloud2
    path_publisher = rospy.Publisher("object_detection/counter", Message1, queue_size=1)
    rospy.spin()

if __name__ == '__main__':
    listener()
