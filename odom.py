#!/usr/bin/env python

import rospy
import tf
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from gazebo_msgs.srv import GetModelState, GetModelStateRequest

rospy.init_node('odom_pub')

odom_pub = rospy.Publisher('/my_odom', Odometry, queue_size=10)

rospy.wait_for_service('/gazebo/get_model_state')
get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

odom = Odometry()
header = Header()
header.frame_id = rospy.get_param("~ground_frame_id")
model = GetModelStateRequest()
# model.model_name = rospy.get_param("~model_name")
model.model_name = '/'
rospy.loginfo("frame: %s, model: %s", header.frame_id, model.model_name)




while not rospy.is_shutdown():
    result = get_model_srv(model)

    odom.pose.pose = result.pose
    odom.twist.twist = result.twist

    header.stamp = rospy.Time.now()
    odom.header = header

    odom_pub.publish (odom)
    br = tf.TransformBroadcaster()
    br.sendTransform((result.pose.position.x, result.pose.position.y, 0.2),
                      (result.pose.orientation.x, result.pose.orientation.y, result.pose.orientation.z, result.pose.orientation.w),
                      rospy.Time.now(),
                      "/base_link",
                      "/odom")

    r = rospy.Rate(20)