#!/usr/bin/env python
# license removed for brevity

import rospy

# Brings in the SimpleActionClient
import actionlib
# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import nav_msgs.srv 
import nav_msgs
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped

def movebase_client():

   # Create an action client called "move_base" with action definition file "MoveBaseAction"
    client = actionlib.SimpleActionClient('/tb3_1/move_base',MoveBaseAction)
 
   # Waits until the action server has started up and started listening for goals.
    client.wait_for_server()

   # Creates a new goal with the MoveBaseGoal constructor
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "/map"
    goal.target_pose.header.stamp = rospy.Time.now()
   # Move 0.5 meters forward along the x axis of the "map" coordinate frame 
    goal.target_pose.pose.position.x = 1.0
    goal.target_pose.pose.position.y = 0.5
   # No rotation of the mobile base frame w.r.t. map frame
    goal.target_pose.pose.orientation.w = 1.0

   # Sends the goal to the action server.
    client.send_goal(goal)
   # Waits for the server to finish performing the action.
    wait = client.wait_for_result()

    #GetPlan - example of functionality

    # sub = rospy.Subscriber("/move_base/GlobalPlanner/plan", Path)
    
    start = PoseStamped()
    start.header.seq = 0
    start.header.frame_id = "/tb3_1/map"
    start.header.stamp = rospy.Time(0)
    start.pose.position.x = goal.target_pose.pose.position.x 
    start.pose.position.y = goal.target_pose.pose.position.y

    Goal = PoseStamped()
    Goal.header.seq = 0
    Goal.header.frame_id = "/tb3_1/map"
    Goal.header.stamp = rospy.Time(0)
    Goal.pose.position.x = 1.0 
    Goal.pose.position.y = 1.0  

    get_plan = rospy.ServiceProxy('/tb3_1/move_base/make_plan', nav_msgs.srv.GetPlan)
    req = nav_msgs.srv.GetPlan()
    req.start = start
    req.goal = Goal
    req.tolerance = .5
    resp = get_plan(req.start, req.goal, req.tolerance)
    rospy.loginfo(len(resp.plan.poses))
    # rospy.loginfo()

   # If the result doesn't arrive, assume the Server is not available
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
    # Result of executing the action
        return client.get_result()   

    

# If the python node is executed as main process (sourced directly)
if __name__ == '__main__':
    try:
       # Initializes a rospy node to let the SimpleActionClient publish and subscribe
        rospy.init_node('movebase_client_py')
        result = movebase_client()
        if result:
            rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")