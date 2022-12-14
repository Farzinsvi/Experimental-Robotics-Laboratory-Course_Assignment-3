#! /usr/bin/env python
"""
Module:
	AnnounceHypotesis
Author:
	Alice Nardelli alice.nardelli98@gmail.com
ROS nodes used for simulating the robot announcement. Given an hypotesis it announces it simply printing on terminal.
Service :
	/announce_service to get the hypotesis to announce
Service Client:
        /reaching_goal client call to reach the centre of the arena

"""
import sys
import rospy
import actionlib
import erl2.msg
import math
import time
from erl_assignment3.srv import ArmorInterface, ArmorInterfaceRequest, Marker
from std_srvs.srv import Empty, Trigger, TriggerResponse
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalID
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Int64
import random
import time


def move_base(x, y):
    global client_move_base
    client_move_base.wait_for_server()
    msg = MoveBaseGoal()
    msg.target_pose.header.frame_id = "map";
    msg.target_pose.pose.orientation.w = 1;
    msg.target_pose.pose.position.x = x
    msg.target_pose.pose.position.y = y
    client_move_base.send_goal(msg)
    print("sent goal")
    client_move_base.wait_for_result(rospy.Duration(20.0))
    #rospy.Duration(30.0)


def cbk_auco(data):
    global client_oracle_hint
    global perceived_id
    global new_id
    global check
    if (data.data in perceived_id) == False:
            perceived_id.append(data.data)
            new_id = data.data
            check = True


def armor_client(new_id):
    global client_oracle_hint, client_armor_interface
    global game_ended
    rospy.wait_for_service('oracle_hint')
    resp1 = client_oracle_hint(new_id)
    rospy.wait_for_service('armor_interface')
    msg=ArmorInterfaceRequest()
    msg.mode=3
    msg.ID=new_id
    msg.key=resp1.key
    msg.value=resp1.value
    resp2=client_armor_interface(msg)
    if resp2.success==True:
       rospy.wait_for_service('armor_interface')
       
       msg.mode=2
       msg.ID=new_id
       resp3=client_armor_interface(msg)
       if resp3.success==True:
            rospy.wait_for_service('armor_interface')
            msg.mode=1
            msg.ID=new_id
            resp4=client_armor_interface(msg)
            if resp4.success==True:
                print("GAME ENDED")
                game_ended=True
             
       
def rotate(w):
       cmd=Twist()
       cmd.angular.z = w
       vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
       vel_publisher.publish(cmd)


def change_room():

     global Room_id, Room_x, Room_y
     global actual_room
     rospy.set_param('/actual_room', actual_room)
     move_base(Room_x[actual_room - 1], Room_y[actual_room - 1])


def rnd_move():
     global check
     global start
     global Room_id, Room_x, Room_y
     global actual_room, new_id
     start_time = time.perf_counter()
     for i in range(1, 10):
             
             if start == False:
               rotate(0)
               x = random.uniform(Room_x[actual_room - 1] - 1.5, Room_x[actual_room - 1] + 1.5)
               y = random.uniform(Room_y[actual_room - 1] - 1.5, Room_y[actual_room - 1] + 1.5)
               print(x)
               print(y)
               move_base(x,y)
               start_time = time.perf_counter()
             start=False
             actual_time = time.perf_counter()
             rotate(10)
             elapsed_time = actual_time - start_time
             while elapsed_time < 10:

                  if check == True:
                     rotate(0)
                     print("perceived_id")
                     print(new_id)
                     armor_client(new_id)
                     rotate(10)
                     check=False
                  actual_time = time.perf_counter()
                  elapsed_time = actual_time - start_time
             print("OUT")



def main():

    # Centre: (0,-1)

    global client_move_base, client_motion_plan
    global perceived_id
    global Room_id, Room_x, Room_y
    global actual_room, new_id
    global check, game_ended, start

    check = False
    # init node
    rospy.init_node('fsm')
    # load the ontology
    client_armor_interface = rospy.ServiceProxy('/armor_interface', ArmorInterface)
    client_oracle_hint = rospy.ServiceProxy('oracle_hint', Marker)
    client_move_base = actionlib.SimpleActionClient(
        '/move_base', MoveBaseAction)

    rospy.Subscriber("/id_aruco", Int64, cbk_auco)
    actual_room = 0
    rospy.wait_for_service('/armor_interface')
    msg=ArmorInterfaceRequest()
    msg.mode=0
    resp=client_armor_interface(msg)
    perceived_id = list()
    Room_id = [1, 2,3,4,5,6]
    Room_x = [-4,-4,-4,5,5,5]
    Room_y = [-3,2,7,-7,-3,1]
    
    game_ended=False
    
    rate = rospy.Rate(10)
    while game_ended == False:
            if actual_room == 6:
                 actual_room = 1
            else:
                 actual_room = actual_room + 1
            change_room()
            print("room_changed")
            start = True
            rnd_move()
            start=True



            rate.sleep()


if __name__ == '__main__':
    main()
