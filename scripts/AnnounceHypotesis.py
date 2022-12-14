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
import rospy
import random
from erl2.srv import Announcement, AnnouncementResponse
import actionlib

from move_base_msgs.msg import MoveBaseActionGoal,MoveBaseAction, MoveBaseGoal

def announce_clbk(req):
    '''
    Description of the callback:
    This function retrieves the request field of the Announcement message. Inside the custom message is present the istances of classes PERSON, PLACE, WEAPON
    corresponding to the hypothesis to announce. The robot firstly reach the centre of the arena, the announce the hypothesis finally returns to the initial location.
    Args:
      srv(Announcement): data retrieved by */announce_service* topic
    Returns:
      srv(Announcement):True
    '''
    Room_x = [-4,-4,-4,5,5,5]
    Room_y = [-3,2,7,-7,-3,1]
    rospy.loginfo('moving at the centr of the arena')
    #reach the centre of the arena 
    client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()
    msg=MoveBaseGoal()
    msg.target_pose.header.frame_id="map";
    msg.target_pose.pose.orientation.w=1;
        #take from parameters the position that must be reached
    msg.target_pose.pose.position.x=0
    msg.target_pose.pose.position.y=-1

    
   
    # Sends the goal to the action server.
    client.send_goal(msg)
    

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    #announce the hypothesis
    rospy.loginfo('Announce to Oracle: ')
    rospy.loginfo(req.who + ' with the ' + req.what + ' in the ' + req.where)
    #return to the starting location

    actual_loc = rospy.get_param('/actual_location')

    goal.target_pose.pose.position.x = Room_x[actual_location-1]
    goal.target_pose.pose.position.y = Room_y[actual_location-1]
    client.send_goal(goal)
    client.wait_for_result()

    

    return True


def main():
    # init node
    rospy.init_node('announce_service')
    # init service
    srv = rospy.Service('announce_service', Announcement, announce_clbk)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():

        rate.sleep()


if __name__ == '__main__':
    main()
