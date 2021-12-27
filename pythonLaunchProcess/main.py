#!/usr/bin/python3

import threading
import time
import socket
import signal
import sys
import subprocess
import os
from typing import Tuple
import roslaunch
import rospy
import signal

HOST = '127.0.0.1'  # The server's hostname or IP address
#HOST = '172.16.120.83'
PORT = 2004         # The port used by the server
ISHOST = False

#Status Running
ROS_IDLE = 0
ROS_START_MAPPING = 1
ROS_START_LOCALIZATION = 2
ROS_LAUNCH_MAPPING = 3
ROS_LAUNCH_LOCALIZATION = 4
ROS_STOP_MAPPING = 5
ROS_STOP_LOCALIZATION = 6

def signal_handler(sig, frame):
  #os.system('kill -9 `sudo lsof -t -i:2007')
  print('You pressed Ctrl+C!')
  socketRunning.close()
  try:
    launchSimulate.shutdown()
  except:
    pass
  try:
    launchLocation.shutdown()
  except:
    pass
  try:
    launchGmaping.shutdown()
  except:
    pass
  #os.system('killall -9 rosmaster')
  sys.exit(0)
def RunLaunch(link):
  print("Start1")
  rospy.init_node('en_Mapping', anonymous=True)
  print("Start2")
  uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
  print("Start3")
  #roslaunch.configure_logging(uuid)
  print("Start4")
  launch = roslaunch.parent.ROSLaunchParent(uuid, [link])
  print("Start5")
  launch.start()
  print("Start6")
  rospy.loginfo("started")
  print("Start7")
  return launch
def RunLaunchWithParameter(link, parameterName, parameterValue):
  rospy.init_node('en_Mapping', anonymous=True)
  rospy.set_param(parameterName, parameterValue)
  uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
  roslaunch.configure_logging(uuid)
  launch = roslaunch.parent.ROSLaunchParent(uuid, [link])
  launch.start()
  rospy.loginfo("started")
  return launch


# Define a function for the thread
statusRun = ROS_IDLE
statusRunPre = ROS_IDLE
isSend = False
launchSimulate = None
launchGmaping = None
launchSaveMap = None
launchLocation = None
socketRunning = None
def TcpConnect():
  global isSend
  global statusRun
  global socketRunning

  isConnected = False
  while(True):
    if not isConnected:
      try:
        socketRunning = socket.socket()  
        socketRunning.connect( ( HOST, PORT ) )
        isConnected = True
        print("Reconnect to Main process Ok")
      except socket.error:
        print(socket.error)
        time.sleep(1)
    else:
      try:
        data = socketRunning.recv(1024)
      except socket.error:
        isConnected = False
        print(socket.error)
        socketRunning.close()
        print("Disconnect Main process Ok")
      if isConnected:
        if data == b'':
          isConnected = False
          socketRunning.close()
          print("Disconnect Main process Ok")
        elif data == b'RosLaunchStartMapping':
          print(data)
          if statusRun == ROS_IDLE:
            statusRun = ROS_START_MAPPING
        elif data == b'RosLaunchStopMapping':
          print(data)
          if statusRun == ROS_LAUNCH_MAPPING:
            statusRun = ROS_STOP_MAPPING
        elif data == b'RosLaunchStartLocation':
          print(data)
          if statusRun == ROS_IDLE:
            statusRun = ROS_START_LOCALIZATION
          else:
            try:
              launchSimulate.shutdown()
            except:
              pass
            try:
              launchLocation.shutdown()
            except:
              pass
            print("ROS_STOP_LOCALIZATION")
            statusRun = ROS_START_LOCALIZATION
        elif data == b'RosLaunchStopLocation':
          print(data)
          if statusRun == ROS_LAUNCH_LOCALIZATION:
            statusRun = ROS_STOP_LOCALIZATION
        else:
          print(data)
    

def test():
  global isSend
  global launchSimulate
  global launchGmaping
  global launchSaveMap
  global launchLocation
  global statusRun
  while True:
    time.sleep(1)

signal.signal(signal.SIGINT, signal_handler)

#os.system('killall -9 rosmaster')
#time.sleep(2)
#roscore = subprocess.Popen('roscore')

if __name__ == "__main__":
  try:
    threadRun = threading.Thread(target=TcpConnect, args=())
    threadRun.start()
    threadRun = threading.Thread(target=test, args=())
    threadRun.start()
  except:
    print("Error: unable to start thread")
  
  isRunMapping = False
  while True:
    if statusRun == ROS_START_MAPPING:
      #rospy.sleep(3)
      print("Start.....")
      os.environ["TURTLEBOT3_MODEL"] = "burger"
      launchSimulate = RunLaunch("/home/idea/turtlebot3/catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/launch/turtlebot3_world.launch")
      if ISHOST:
        launchGmaping = RunLaunch("/opt/ros/melodic/share/turtlebot3_slam/launch/turtlebot3_slam.launch")
      else:
        launchGmaping = RunLaunch("/home/idea/turtle/src/turtlebot3/turtlebot3_slam/launch/turtlebot3_slam.launch")
      statusRun = ROS_LAUNCH_MAPPING
    elif statusRun == ROS_LAUNCH_MAPPING:
      pass
    elif statusRun == ROS_STOP_MAPPING:
      try:
        launchSimulate.shutdown()
      except:
        pass
      try:
        launchGmaping.shutdown()
      except:
        pass
      statusRun = ROS_IDLE
      print("ROS_STOP_MAPPING")
    elif statusRun == ROS_START_LOCALIZATION:
      os.environ["TURTLEBOT3_MODEL"] = "burger"
      launchSimulate = RunLaunch("/home/idea/turtlebot3/catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/launch/turtlebot3_world.launch")
      if ISHOST:
        launchLocation = RunLaunchWithParameter("/opt/ros/melodic/share/turtlebot3_navigation/launch/turtlebot3_navigation.launch", "map_file", "$HOME/idea/mapAgv/map.yaml")
      else:
        launchLocation = RunLaunchWithParameter("/home/idea/turtle/src/turtlebot3/turtlebot3_navigation/launch/turtlebot3_navigation.launch", "map_file", "$HOME/idea/mapAgv/map.yaml")
      statusRun = ROS_LAUNCH_LOCALIZATION
    elif statusRun == ROS_LAUNCH_LOCALIZATION:
      pass
    elif statusRun == ROS_STOP_LOCALIZATION:
      try:
        launchSimulate.shutdown()
      except:
        pass
      try:
        launchLocation.shutdown()
      except:
        pass
      statusRun = ROS_IDLE
      print("ROS_STOP_LOCALIZATION")

    time.sleep(0.3)