#!/usr/bin/env python
	
import rospy
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionResult
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler
import csv
import time
import numpy as np

global navFinished
navFinished = 0

#Callback function to check if the navigation process has finished
def navCallback(msg):
    global navFinished
    if msg.status.status == 3:
        navFinished = 1
	rospy.loginfo('Goal Reached ')

#Quaternion to RPY (by Divakar - StackOverflow)
def quat2RPY(w, x, y, z):
    ysqr = y * y

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + ysqr)
    X = np.degrees(np.arctan2(t0, t1))

    t2 = +2.0 * (w * y - z * x)

    t2 = np.clip(t2, a_min=-1.0, a_max=1.0)
    Y = np.degrees(np.arcsin(t2))

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (ysqr + z * z)
    Z = np.degrees(np.arctan2(t3, t4))

    return X, Y, Z

def writeOdomCSV(fname,goalPose,finalPose):
    #Convert Quartenion to RPY before writing to CSV
    goalRPY = quat2RPY(goalPose.pose.orientation.x,goalPose.pose.orientation.y,goalPose.pose.orientation.z,goalPose.pose.orientation.w)
    finalRPY = quat2RPY(finalPose.pose.pose.orientation.x,finalPose.pose.pose.orientation.y,finalPose.pose.pose.orientation.z,finalPose.pose.pose.orientation.w)
    #Write the result to CSV
    with open(fname, 'w') as file:
        writer = csv.writer(file)
        header = ['xGoal','yGoal','zGoal','rollGoal','pitchGoal','yawGoal','xFinal','yFinal','zFinal','rollFinal','pitchFinal','yawFinal']
        data = [goalPose.pose.position.x,goalPose.pose.position.y,goalPose.pose.position.z,goalRPY[0],goalRPY[1],goalRPY[2],finalPose.pose.pose.position.x,finalPose.pose.pose.position.y,finalPose.pose.pose.position.z,finalRPY[0],finalRPY[1],finalRPY[2]]
        writer.writerow(header)
        writer.writerow(data)
        


        
#init ros and the required variables
rospy.init_node('husky_auto_nav_py_node')
pub = rospy.Publisher('/move_base_simple/goal',PoseStamped,queue_size=10)
sub = rospy.Subscriber('/move_base/result',MoveBaseActionResult,navCallback)

goalPose = PoseStamped()

#Read CSV
with open('../navigation_io.csv', 'r') as file:
    reader = list(csv.reader(file, delimiter=','))
    csvData=reader[1]
    
#Set Position
goalPose.pose.position.x = float(csvData[0])
goalPose.pose.position.y = float(csvData[1])
goalPose.pose.position.z = float(csvData[2])

#Convert Roll,Pitch,Yaw to Quaternion
roll = float(csvData[3])
pitch = float(csvData[4])
yaw = float(csvData[5])
q = quaternion_from_euler(roll,pitch,yaw)

#Assign to Quaternion to goalPose
goalPose.pose.orientation.x = q[0]
goalPose.pose.orientation.y = q[1]
goalPose.pose.orientation.z = q[2]
goalPose.pose.orientation.w = q[3]

#Set frame id
goalPose.header.frame_id ='odom';

while not rospy.is_shutdown():

    #Wait for connection to publisher to publish the goal pose
    while pub.get_num_connections() < 1:
        pass
    pub.publish(goalPose)

    rospy.loginfo("publishing Pose: {},{},{} ".format(goalPose.pose.position.x,goalPose.pose.position.y,goalPose.pose.position.z))
    rospy.loginfo("Orientation: {},{},{}".format(roll,pitch,yaw))

    #Wait for navigation to finish
    while navFinished == 0:
        pass
        
    rospy.loginfo("Navigation Finished - Writing result")

    #Get robot position from odom
    finalPose = rospy.wait_for_message('/odometry/filtered',Odometry)
    
    #Write to CSV
    writeOdomCSV('../navigation_io.csv',goalPose,finalPose)
    
    rospy.loginfo("Done Writing. Exiting")
    
    break

    





    
