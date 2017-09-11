#!/usr/bin/env python

# Import modules
import numpy as np
import sklearn
from sklearn.preprocessing import LabelEncoder
import pickle
from sensor_stick.srv import GetNormals
from sensor_stick.features import compute_color_histograms
from sensor_stick.features import compute_normal_histograms
from visualization_msgs.msg import Marker
from sensor_stick.marker_tools import *
from sensor_stick.msg import DetectedObjectsArray
from sensor_stick.msg import DetectedObject
from sensor_stick.pcl_helper import *

import rospy
import tf
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import String
from pr2_robot.srv import *
from rospy_message_converter import message_converter
import yaml


# Helper function to get surface normals
def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster

# Helper function to create a yaml friendly dictionary from ROS messages
def make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose):
    yaml_dict = {}
    yaml_dict["test_scene_num"] = test_scene_num.data
    yaml_dict["arm_name"]  = arm_name.data
    yaml_dict["object_name"] = object_name.data
    yaml_dict["pick_pose"] = message_converter.convert_ros_message_to_dictionary(pick_pose)
    yaml_dict["place_pose"] = message_converter.convert_ros_message_to_dictionary(place_pose)
    return yaml_dict

# Helper function to output to yaml file
def send_to_yaml(yaml_filename, dict_list):
    data_dict = {"object_list": dict_list}
    with open(yaml_filename, 'w') as outfile:
        yaml.dump(data_dict, outfile, default_flow_style=False)

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

# Exercise-2 TODOs:

    # Convert ROS msg to PCL data
    cloud = ros_to_pcl(pcl_msg)
    
    # Statistical Outlier Filtering
    outlier_filter = cloud.make_statistical_outlier_filter()
    outlier_filter.set_mean_k(50) #30
    outlier_filter.set_std_dev_mul_thresh(0.5) # 0.3
    cloud_filtered = outlier_filter.filter()

    #########################################################################
    # TODO: Voxel Grid Downsampling
    vox = cloud.make_voxel_grid_filter()
    # TODO: PassThrough Filter
    LEAF_SIZE = 0.005  # 0.005
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
    cloud_filtered = vox.filter()
    
    
    ##############################################################################
    #TWO passthrough filters one over Z and one over X
    passthrough = cloud_filtered.make_passthrough_filter()
    filter_axis = 'z' 
    passthrough.set_filter_field_name(filter_axis)
    axis_min = 0.3 # 0.65
    axis_max = 5.0 # 1.35
    passthrough.set_filter_limits(axis_min, axis_max)
    cloud_filtered = passthrough.filter()
    
 
    passthrough = cloud_filtered.make_passthrough_filter()
    filter_axis = 'x' # y
    passthrough.set_filter_field_name(filter_axis)
    axis_min = 0.34 # -0.55
    axis_max = 1.0 # + 0.55
    passthrough.set_filter_limits(axis_min, axis_max)
    cloud_filtered = passthrough.filter()
    
    ##############################################################################
    # RANSAC Plane Segmentation
    seg = cloud_filtered.make_segmenter()
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)
    max_distance = 0.015
    seg.set_distance_threshold(max_distance)
    inliers, coefficients = seg.segment()    
    # Extract inliers and outliers
    extracted_inliers = cloud_filtered.extract(inliers, negative=False)
    extracted_outliers = cloud_filtered.extract(inliers, negative=True)
    
    ##################################################################################
    # Euclidean Clustering
    white_cloud = XYZRGB_to_XYZ(extracted_outliers)
    kd_tree = white_cloud.make_kdtree()

    #Created a cluster extraction object
    ec = white_cloud.make_EuclideanClusterExtraction()
    #SetTolerances
    ec.set_ClusterTolerance(0.01) # 0.015
    ec.set_MinClusterSize(50) # 100
    ec.set_MaxClusterSize(15000) # 5000
    
    #Search the k-d tree for clusters
    ec.set_SearchMethod(kd_tree)
    #Extract indices for each discovered clusters
    cluster_indices = ec.Extract()
    
    # TODO: Create Cluster-Mask Point Cloud to visualize each cluster separately
    #Assign a cloror corresponding to each segmented object
    cluster_color = get_color_list(len(cluster_indices))
    
    color_cluster_point_list = []
    
    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([
                    white_cloud[indice][0],
                    white_cloud[indice][1],
                    white_cloud[indice][2],
                    rgb_to_float(cluster_color[j])           
            ])

    #CreateNew Cloud Contaning all clusters, with unique colors
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)
    
    # TODO: Convert PCL data to ROS messages
    table_pcl_msg = pcl_to_ros(extracted_inliers)
    objects_pcl_msg = pcl_to_ros(extracted_outliers)
    ros_cluster_cloud = pcl_to_ros(cluster_cloud)
    

    # TODO: Publish ROS messages
    pcl_objects_pub.publish(objects_pcl_msg)
    pcl_table_pub.publish(table_pcl_msg)
    pcl_clusters_pub.publish(ros_cluster_cloud)
# Exercise-3 TODOs:

    # Classify the clusters! (loop through each detected cluster one at a time)

    detected_objects_labels = []
    detected_objects_list = []
        
    for index, pts_list in enumerate(cluster_indices):
        # Grab the points for the cluster
        pcl_cluster = extracted_outliers.extract(pts_list)
        ros_cluster = pcl_to_ros(pcl_cluster)
            
        # Compute the associated feature vector
        chists = compute_color_histograms(ros_cluster, using_hsv=True)
        normals = get_normals(ros_cluster)
        nhists = compute_color_histograms(normals)
        feature = np.concatenate((chists,nhists))
            
        # Make the prediction
        prediction = clf.predict(scaler.transform(feature.reshape(1,-1)))
        label = encoder.inverse_transform(prediction)[0]
        detected_objects_labels.append(label)
            
        # Publish a label into RViz
        label_pos = list(white_cloud[pts_list[0]])
        label_pos[2] += 0.25
            
        object_markers_pub.publish(make_label(label, label_pos, index))

        # Add the detected object to the list of detected objects.
        do = DetectedObject()
        do.label = label
        do.cloud = ros_cluster
        detected_objects_list.append(do)

    # Publish the list of detected objects
    #This is the output for the upcoming project
    detected_objects_pub.publish(detected_objects_list)

    # Suggested location for where to invoke your pr2_mover() function within pcl_callback()
    # Could add some logic to determine whether or not your object detections are robust
    # before calling pr2_mover()
    try:
        pr2_mover(detected_objects_list)
    except rospy.ROSInterruptException:
        pass

def reset_pose_position(pose):
    pose.position.x = 0
    pose.position.y = 0
    pose.position.z = 0
    return pose
def reset_pose_orientation(pose):
    pose.orientation.x = 0
    pose.orientation.y = 0
    pose.orientation.z = 0
    pose.orientation.w = 0
    return pose
# function to load parameters and request PickPlace service
def pr2_mover(object_list):

    # TODO: Initialize variables
    TEST_SCENE_NUM = std_msgs.msg.Int32()
    TEST_SCENE_NUM.data = 1
    OBJECT_NAME = std_msgs.msg.String()
    WHICH_ARM = std_msgs.msg.String() # green = right, red = left
    PICK_POSE = geometry_msgs.msg.Pose()
    PLACE_POSE = geometry_msgs.msg.Pose()
    dict_list = []
    centroids = []
    counter = 0
    output_yaml = []
    

    # TODO: Get/Read parameters
    object_list_param = rospy.get_param('/object_list')
    dropbox_param = rospy.get_param('/dropbox')
    rospy.loginfo('Starting pr2_mover with {} objects'.format(len(object_list_param)))
    
    # TODO: Parse parameters into individual variables
    dict_dropbox = {}
    for param in dropbox_param:
        dict_dropbox[param['name']] = param['position']
    print "Object List Len", len(object_list)
    print "Dict Dropbox",dict_dropbox
    print "Object Param List", len(object_list_param)
    # TODO: Rotate PR2 in place to capture side tables for the collision map
 
    # TODO: Loop through the pick list
    for obj in object_list_param:
        print "Object Name:", obj['name']
        OBJECT_NAME.data = obj['name']
        
        WHICH_ARM.data = ''
        reset_pose_position(PICK_POSE)
        reset_pose_orientation(PICK_POSE)
        reset_pose_position(PLACE_POSE)
        reset_pose_orientation(PLACE_POSE)
        
        # TODO: Get the PointCloud for a given object and obtain it's centroid
        for detected in object_list:
            if OBJECT_NAME.data == detected.label:
                print "Detected Label:",detected.label
                points_arr = ros_to_pcl(detected.cloud).to_array()
                pick_pose_centroids = np.mean(points_arr, axis=0)[:3]
                # TODO: Create 'place_pose' for the object
                PICK_POSE.position.x = np.asscalar(pick_pose_centroids[0])
                PICK_POSE.position.y = np.asscalar(pick_pose_centroids[1])
                PICK_POSE.position.z = np.asscalar(pick_pose_centroids[2])
                #break

                # TODO: Assign the arm to be used for pick_place            
                if obj['group'] == 'red':
                    WHICH_ARM.data = 'left'
                else:
                    WHICH_ARM.data = 'right'
            
            
                PLACE_POSE.position.x = dict_dropbox[WHICH_ARM.data][0]
                PLACE_POSE.position.y = dict_dropbox[WHICH_ARM.data][1]
                PLACE_POSE.position.z = dict_dropbox[WHICH_ARM.data][2]
                # TODO: Create a list of dictionaries (made with make_yaml_dict()) for later output to yaml format
                yaml_dict = make_yaml_dict(TEST_SCENE_NUM, WHICH_ARM, OBJECT_NAME, PICK_POSE, PLACE_POSE)
                output_yaml.append(yaml_dict)
            
                # Wait for 'pick_place_routine' service to come up
        rospy.wait_for_service('pick_place_routine')

            #try:
            #    pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)

                # TODO: Insert your message variables to be sent as a service request
            #    resp = pick_place_routine(TEST_SCENE_NUM, OBJECT_NAME, WHICH_ARM, PICK_POSE, PLACE_POSE)

            #    print ("Response: ",resp.success)

            #except rospy.ServiceException, e:
            #    print "Service call failed: %s"%e
        #else:
        #    rospy.loginfo('Cant find object: {}'.format(object_list_param[counter]['name']))
    
    # TODO: Output your request parameters into output yaml file
        send_to_yaml("output_"+ str(TEST_SCENE_NUM.data) + ".yaml", output_yaml)


if __name__ == '__main__':

    # TODO: ROS node initialization
    rospy.init_node('clustering', anonymous=False)
    
    # TODO: Create Subscribers
    pcl_sub = rospy.Subscriber('/pr2/world/points', pc2.PointCloud2, pcl_callback, queue_size=1)

    # TODO: Create Publishers
    object_markers_pub = rospy.Publisher("/object_markers", Marker, queue_size=1)
    detected_objects_pub = rospy.Publisher("/detected_objects", DetectedObjectsArray, queue_size=1)
    pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size=1)
    pcl_table_pub = rospy.Publisher("/pcl_table", PointCloud2, queue_size=1)
    pcl_clusters_pub = rospy.Publisher("/pcl_clusters", PointCloud2, queue_size=1)

    # TODO: Load Model From disk
    model = pickle.load(open('model.sav','rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ =  model['classes']
    scaler = model['scaler']
    # Initialize color_list
    get_color_list.color_list = []

    # TODO: Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()
