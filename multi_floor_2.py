#!/usr/bin/env python

import rospy
import sys
import time
import roslaunch
import csv
import math
import tf
import numpy as np
from nav_msgs.srv import LoadMap, LoadMapRequest
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseActionResult
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from threading import Thread
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import *


FRAME_ID = "map"
elevator_pos = [[13.35, -5.48, 0.5, 0, 0, 0, 3.14, 10], [-5.37, -5.118, 0.5, 0, 0, 0, 10], [-5.37, -3.58, 0.5, 0, 0, 0, 10]]
exit_pos = [[14.5, -5.48, 0.5, 0, 0, 0, 0, 0.1], [-6.37, -5.28, 0.5, 0, 0, 3.14, 0.1], [-6.37, -3.58, 0.5, 0, 0, 3.14, 1]]
corridor_pos = [[0, 0, 0, 0, 0, 0, 0.2], [0, 0, 0, 0, 0, 3.14, 0.2], [0, 0, 0, 0, 0, 3.14, 0.2]]
room_encode = [['101', [-13.45, 0.4, 0.5, 0, 0, 1.57, 1]], 
              ['102', [-9.56, 0.4, 0.5, 0, 0, 1.57, 1]],
              ['103', [-5.57, 0.4, 0.5, 0, 0, 1.57, 1]],
              ['104', [14.45, 0.4, 0.5, 0, 0, 1.57, 1]],
              ['105', [-14.84, 0.4, 0.5, 0, 0, 4.71, 1]],
              ['201', [-13.45, 0.4, 3, 0, 0, 1.57, 1]],
              ['202', [-9.55, 0.4, 3, 0, 0, 1.57, 1]],
              ['203', [-5.85, 0.4, 3, 0, 0, 1.57, 1]],
              ['204', [-1.69, 0.4, 3, 0, 0, 1.57, 1]],
              ['205', [2.12, 0.4, 3, 0, 0, 1.57, 1]],
              ['206', [6.14, 0.4, 3, 0, 0, 1.57, 1]],
              ['207', [10.07, 0.4, 3, 0, 0, 1.57, 1]],
              ['208', [14.17, 0.4, 3, 0, 0, 1.57, 1]],
              ['209', [-14.42, 0.3, 3, 0, 0, 4.71, 1]],
              ['210', [-2.49, 0.3, 3, 0, 0, 4.71, 1]],
              ['211', [1.49, 0.3, 3, 0, 0, 4.71, 1]],
              ['212', [5.51, 0.3, 3, 0, 0, 4.71, 1]],
              ['213', [9.47, 0.3, 3, 0, 0, 4.71, 1]],
              ['301', [-13.45, 0.4, 5.5, 0, 0, 1.57, 1]],
              ['302', [-9.55, 0.4, 5.5, 0, 0, 1.57, 1]],
              ['303', [-5.85, 0.4, 5.5, 0, 0, 1.57, 1]],
              ['304', [-1.69, 0.4, 5.5, 0, 0, 1.57, 1]],
              ['305', [2.12, 0.4, 5.5, 0, 0, 1.57, 1]],
              ['306', [6.14, 0.4, 5.5, 0, 0, 1.57, 1]],
              ['307', [10.07, 0.4, 5.5, 0, 0, 1.57, 1]],
              ['308', [14.17, 0.4, 5.5, 0, 0, 1.57, 1]],
              ['309', [-14.42, 0.3, 5.5, 0, 0, 4.71, 1]],
              ['310', [-2.45, 0.3, 5.5, 0, 0, 4.71, 1]],
              ['311', [5.50, 0.3, 5.5, 0, 0, 4.71, 1]],
              ['401', [-9.37, 0.4, 8, 0, 0, 1.57, 1]],
              ['402', [-1.6, 0.4, 8, 0, 0, 1.57, 1]],
              ['403', [6.37, 0.4, 8, 0, 0, 1.57, 1]],
              ['404', [14.38, 0.4, 8, 0, 0, 1.57, 1]],
              ['405', [-14.42, 0.3, 8, 0, 0, 4.71, 1]],
              ['406', [-2.45, 0.3, 8, 0, 0, 4.71, 1]],
              ['407', [5.5, 0.3, 8, 0, 0, 4.71, 1]]]
goal = []


class Waypoint_Publish:

  def __init__(self):
    rospy.init_node('jackal_waypoint_publisher', anonymous=True)
    self.result_dict = False
    self.level1_map = '/home/yuhan/catkin_ws/src/boxer/boxer_navigation/maps/level1_map.yaml'
    self.level2_map = '/home/yuhan/catkin_ws/src/boxer/boxer_navigation/maps/level2_map.yaml'
    self.level3_map = '/home/yuhan/catkin_ws/src/boxer/boxer_navigation/maps/level3_map.yaml'
    self.level4_map = '/home/yuhan/catkin_ws/src/boxer/boxer_navigation/maps/level4_map.yaml'
#iterate waypoint list to create a matrix for each jackal
#create publishers for topics

    topic_name = '/move_base_simple/goal'
    self.publishers = rospy.Publisher(topic_name, PoseStamped, queue_size=10, latch=True)
    result_name = '/move_base/result'
    self.results = rospy.Subscriber(result_name, MoveBaseActionResult, self.result_handler)

  def movebase_goal(self, x, y, z, roll, pitch, yaw, frame):
    pose = PoseStamped()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    pose.pose.orientation.x = quaternion[0]
    pose.pose.orientation.y = quaternion[1]
    pose.pose.orientation.z = quaternion[2]
    pose.pose.orientation.w = quaternion[3]
    pose.header.stamp = rospy.Time(0)
    pose.header.frame_id = frame
    return pose

  def handle_waypoints(self, publisher):
    print("handle waypoints")
    global i 
    for i in range(len(goal)):
      print(i)
      pose = self.movebase_goal(goal[i][0], goal[i][1], goal[i][2], goal[i][3], goal[i][4], goal[i][5], FRAME_ID)
      print "publishing goal to " 
      print pose
      publisher.publish(pose)
      while (self.result_dict == False):
        # print("result loop")
        pass
      print("finish one loop")
      self.result_dict = False
      rospy.sleep(goal[i][6])

  def result_handler(self, data):
    print("which map to change", i)
    self.result_dict = True
    print(self.result_dict)
    print " result handle detected"
    if i == 1:
      if goal[3][2] == 3:
        self.request.map_url = self.level2_map
      elif goal[3][2] == 5.5:
        self.request.map_url = self.level3_map
      elif goal[3][2] == 8:
        self.request.map_url = self.level4_map
      change_map_result = self.change_map(self.request)
      model = GetModelStateRequest()
      model.model_name = '/'
      result = self.get_state(model)
      state_msg = ModelState()
      state_msg.model_name = '/'
      state_msg.pose = result.pose
      state_msg.pose.position.z = goal[3][2]
      resp = self.set_state(state_msg)
    print("finished result handler", self.result_dict)
    return


  def execute(self):
    print("execute")
    rospy.wait_for_service('/change_map')
    rospy.wait_for_service('/gazebo/set_model_state')
    rospy.wait_for_service('/gazebo/get_model_state')
    self.change_map = rospy.ServiceProxy('/change_map', LoadMap)
    self.set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    self.get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    self.request = LoadMapRequest()    
    self.handle_waypoints(self.publishers)
    rospy.loginfo("waypoint publishing finished successfully")
    rospy.signal_shutdown("waypoint publishing finished")

def get_goal_points(room):
  global room_encode
  loc = [x[1] for x in room_encode if x[0]==room][0]
  if loc[2] == 0.5:
    goal_points = [loc]
  else:
    # calculate the euclidean distance of the goal to elevator and the start location
    start_location = [0, 0]
    min_dist = 1000
    elevator_idx = 0
    for k, e in enumerate(elevator_pos):
      dist = np.sqrt((e[0]-start_location[0])**2 + (e[1]-start_location[1])**2) + np.sqrt((e[0]-loc[0])**2 + (e[1]-loc[1])**2)
      if dist < min_dist:
        elevator_idx = k
        min_dist = dist
    #12 13 0 0 0 3.14 1
    exit_elevator = exit_pos[elevator_idx]
    exit_elevator[2] = loc[2]
    goal_points = [corridor_pos[elevator_idx], elevator_pos[elevator_idx], exit_elevator, loc]
    print(goal_points)
  return goal_points

def main(args):
  global goal
  # node = rospy.init_node('waypoint_pub', anonymous=True)
  # room = rospy.get_param('~room')
  # room = '404'
  try:
    print("Please enter the room number: ")
    room = str(input())
  except:
    print("key In Error")
  goal_points = get_goal_points(room)
  goal = goal_points
  try:
    # while not rospy.is_shutdown():
    mywaypoint = Waypoint_Publish()
    mywaypoint.execute()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)





