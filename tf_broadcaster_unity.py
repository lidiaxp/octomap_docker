#!/usr/bin/env python  
import roslib
import rospy
from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import PointCloud2
from rosgraph_msgs.msg import Clock
 
import tf
import turtlesim.msg

global cur_time 
 
def mainTfCallback(data):
    br = tf.TransformBroadcaster()
    # current_time = rospy.Time.now()
    current_time = rospy.get_rostime()
    
    transform = TransformStamped()
    transform.header.stamp = current_time
    transform.header.frame_id = "world"
    transform.child_frame_id = "uav"
    transform.transform.translation.x = 0.0  
    transform.transform.translation.y = 0.0
    transform.transform.translation.z = 0.0
    quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)  # Modify rotation values as needed
    transform.transform.rotation.x = quaternion[0]
    transform.transform.rotation.y = quaternion[1]
    transform.transform.rotation.z = quaternion[2]
    transform.transform.rotation.w = quaternion[3]

    br.sendTransformMessage(transform)


def velodyneTfCallback(data):
    broadcaster = tf.TransformBroadcaster()
    # current_time = rospy.Time.now()
    current_time = rospy.get_rostime()

    transform = TransformStamped()
    transform.header.stamp = current_time
    transform.header.frame_id = "uav"  
    transform.child_frame_id = "point_cloud_from_unity"  
    transform.transform.translation.x = 0.0  
    transform.transform.translation.y = 0.0
    transform.transform.translation.z = 0.0
    transform.transform.rotation.x = 0.0  
    transform.transform.rotation.y = 0.0
    transform.transform.rotation.z = 0.0
    transform.transform.rotation.w = 1.0
    broadcaster.sendTransformMessage(transform)

    broadcaster = tf.TransformBroadcaster()
    transform = TransformStamped()
    transform.header.stamp = rospy.get_rostime()
    transform.header.frame_id = "point_cloud_from_unity"  
    transform.child_frame_id = "global_map_octomap"  
    transform.transform.translation.x = 0.0  
    transform.transform.translation.y = 0.0
    transform.transform.translation.z = 0.0
    transform.transform.rotation.x = 0.0  
    transform.transform.rotation.y = 0.0
    transform.transform.rotation.z = 0.0
    transform.transform.rotation.w = 1.0
    broadcaster.sendTransformMessage(transform)


def clock_callback(data):
    global cur_time
    cur_time = data.clock


if __name__ == '__main__':
    rospy.init_node('tf_broadcaster')

    rospy.Timer(rospy.Duration(0.1), mainTfCallback)
    rospy.Timer(rospy.Duration(0.1), velodyneTfCallback)

    # rospy.Subscriber('/unity/imu', PoseStamped, imu_callback)
    # rospy.Subscriber('/velodyne_points', PointCloud2, velodyne_callback)
    rospy.Subscriber('/clock', Clock, clock_callback)
    rospy.spin()