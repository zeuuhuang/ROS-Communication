#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import serial
import math
import time
from std_msgs.msg import String

security = 'I am Autonomous Prime, Autobots, roll out!' # Security code to confirm hacker is not$
CAMERA_FREQUENCY = 10 # in Hz, set sensing rate
accelerometerThreshold = -10.0 # set deceleration threshold in g where the stop command is sent
history = 'running' # Initialize history string
ser = serial.Serial(
  port='/dev/ttyUSB1',\
  baudrate=57600,\
  parity=serial.PARITY_NONE,\
  stopbits=serial.STOPBITS_ONE,\
  bytesize=serial.EIGHTBITS,\
  timeout=0) # Open Serial Port to the OLA
time.sleep(5) # Wait 5 seconds for serial port to initialize

# Read Data from IMU
def callback(ser):
  line = ser.readline()
  line = str(line, 'utf-8', errors='ignore')
  splitLine = line.split(',')
  if len(splitLine) < 2:
    a = 0
  else:
    a = float(splitLine[1])
  rospy.loginfo(f"/Acceleration: {a}")
  return a
  #b = a[0].decode('utf-8')
  #c = int.from_bytes(a[0], byteorder='little', signed=False) 
  #a = line.split(comma
  #ser.close()
  #return c
  
# Publish security and stop commands
def talker(history, ser):
  rospy.init_node('talker_node', anonymous=True)
  pubSecurity = rospy.Publisher('SecurityCode', String, queue_size=10)
  pubCommand = rospy.Publisher('CrashAlert', String, queue_size=10)
  rate = rospy.Rate(CAMERA_FREQUENCY)
    
  while not rospy.is_shutdown():
    # Publish Security Code to confirm correct correction
    pubSecurity.publish(security)
    acceleration = callback(ser)

    # Detect if there is a crash or slowdown, publish command 
    if acceleration < accelerometerThreshold or history == 'stopped':
      pubCommand.publish('stop')
      history = 'stopped'
      rospy.loginfo(f"/Command sent: {history}")
      rospy.loginfo(f"/Acceleration: {acceleration}")
      rospy.spin()
    else:
      pubCommand.publish('continue')
      rospy.loginfo(f"/Command sent: {history}")
      rospy.loginfo(f"/Acceleration: {acceleration}")
      rospy.spin()
      
    rate.sleep()
         
    
if __name__ == '__main__':
    try:
        talker(history, ser)
    except rospy.ROSInterruptException:
        pass
