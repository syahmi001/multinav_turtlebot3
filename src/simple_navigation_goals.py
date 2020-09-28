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
    client1 = actionlib.SimpleActionClient('/tb3_0/move_base',MoveBaseAction)
    client2 = actionlib.SimpleActionClient('/tb3_1/move_base',MoveBaseAction)
    client3 = actionlib.SimpleActionClient('/tb3_2/move_base',MoveBaseAction)
    client4 = actionlib.SimpleActionClient('/tb3_3/move_base',MoveBaseAction)
 
   # Waits until the action server has started up and started listening for goals.
    client1.wait_for_server()
    client2.wait_for_server()
    client3.wait_for_server()
    client4.wait_for_server()

   # Creates a new goal with the MoveBaseGoal constructor
    goal1 = MoveBaseGoal()
    goal1.target_pose.header.frame_id = "/map"
    goal1.target_pose.header.stamp = rospy.Time.now()

    goal2 = MoveBaseGoal()
    goal2.target_pose.header.frame_id = "/map"
    goal2.target_pose.header.stamp = rospy.Time.now()

    goal3 = MoveBaseGoal()
    goal3.target_pose.header.frame_id = "/map"
    goal3.target_pose.header.stamp = rospy.Time.now()

    goal4 = MoveBaseGoal()
    goal4.target_pose.header.frame_id = "/map"
    goal4.target_pose.header.stamp = rospy.Time.now()

   # Move 0.5 meters forward along the x axis of the "map" coordinate frame 
    goal1.target_pose.pose.position.x = 1.0
    goal1.target_pose.pose.position.y = -1.0

    goal2.target_pose.pose.position.x = 1.0
    goal2.target_pose.pose.position.y = 1.0

    goal3.target_pose.pose.position.x = -1.0
    goal3.target_pose.pose.position.y = -1.0

    goal4.target_pose.pose.position.x = -1.0
    goal4.target_pose.pose.position.y = 1.0

   # No rotation of the mobile base frame w.r.t. map frame
    goal1.target_pose.pose.orientation.w = 1.0
    goal2.target_pose.pose.orientation.w = 1.0
    goal3.target_pose.pose.orientation.w = 1.0
    goal4.target_pose.pose.orientation.w = 1.0

   # Sends the goal to the action server.
    client1.send_goal(goal1)
    client2.send_goal(goal2)
    client3.send_goal(goal3)
    client4.send_goal(goal4)
   # Waits for the server to finish performing the action.
    wait1 = client1.wait_for_result()
    wait2 = client2.wait_for_result()
    wait3 = client3.wait_for_result()
    wait4 = client4.wait_for_result()

    #GetPlan - example of functionality

    # sub = rospy.Subscriber("/move_base/GlobalPlanner/plan", Path)
    
    start1 = PoseStamped()
    start1.header.seq = 0
    start1.header.frame_id = "/tb3_0/map"
    start1.header.stamp = rospy.Time(0)
    start1.pose.position.x = goal1.target_pose.pose.position.x 
    start1.pose.position.y = goal1.target_pose.pose.position.y

    start2 = PoseStamped()
    start2.header.seq = 0
    start2.header.frame_id = "/tb3_1/map"
    start2.header.stamp = rospy.Time(0)
    start2.pose.position.x = goal2.target_pose.pose.position.x 
    start2.pose.position.y = goal2.target_pose.pose.position.y

    start3 = PoseStamped()
    start3.header.seq = 0
    start3.header.frame_id = "/tb3_2/map"
    start3.header.stamp = rospy.Time(0)
    start3.pose.position.x = goal3.target_pose.pose.position.x 
    start3.pose.position.y = goal3.target_pose.pose.position.y

    start4 = PoseStamped()
    start4.header.seq = 0
    start4.header.frame_id = "/tb3_3/map"
    start4.header.stamp = rospy.Time(0)
    start4.pose.position.x = goal4.target_pose.pose.position.x 
    start4.pose.position.y = goal4.target_pose.pose.position.y

    Goal1 = PoseStamped()
    Goal1.header.seq = 0
    Goal1.header.frame_id = "/tb3_0/map"
    Goal1.header.stamp = rospy.Time(0)
    Goal1.pose.position.x = 1.0 
    Goal1.pose.position.y = -1.0

    Goal2 = PoseStamped()
    Goal2.header.seq = 0
    Goal2.header.frame_id = "/tb3_1/map"
    Goal2.header.stamp = rospy.Time(0)
    Goal2.pose.position.x = 1.0 
    Goal2.pose.position.y = 1.0 

    Goal3 = PoseStamped()
    Goal3.header.seq = 0
    Goal3.header.frame_id = "/tb3_2/map"
    Goal3.header.stamp = rospy.Time(0)
    Goal3.pose.position.x = -1.0 
    Goal3.pose.position.y = -1.0 

    Goal4 = PoseStamped()
    Goal4.header.seq = 0
    Goal4.header.frame_id = "/tb3_3/map"
    Goal4.header.stamp = rospy.Time(0)
    Goal4.pose.position.x = -1.0 
    Goal4.pose.position.y = 1.0   

    get_plan1 = rospy.ServiceProxy('/tb3_0/move_base/make_plan', nav_msgs.srv.GetPlan)
    req1 = nav_msgs.srv.GetPlan()
    req1.start = start1
    req1.goal = Goal1
    req1.tolerance = .5
    resp1 = get_plan1(req1.start, req1.goal, req1.tolerance)
    rospy.loginfo(len(resp1.plan.poses))

    get_plan2 = rospy.ServiceProxy('/tb3_1/move_base/make_plan', nav_msgs.srv.GetPlan)
    req2 = nav_msgs.srv.GetPlan()
    req2.start = start2
    req2.goal = Goal2
    resp2 = get_plan2(req2.start, req2.goal, req1.tolerance)
    rospy.loginfo(len(resp2.plan.poses))

    get_plan3 = rospy.ServiceProxy('/tb3_2/move_base/make_plan', nav_msgs.srv.GetPlan)
    req3 = nav_msgs.srv.GetPlan()
    req3.start = start3
    req3.goal = Goal3
    resp3 = get_plan3(req3.start, req3.goal, req1.tolerance)
    rospy.loginfo(len(resp3.plan.poses))

    get_plan4 = rospy.ServiceProxy('/tb3_3/move_base/make_plan', nav_msgs.srv.GetPlan)
    req4 = nav_msgs.srv.GetPlan()
    req4.start = start4
    req4.goal = Goal4
    resp4 = get_plan4(req4.start, req4.goal, req1.tolerance)
    rospy.loginfo(len(resp4.plan.poses))
    # rospy.loginfo()

   # If the result doesn't arrive, assume the Server is not available
    if not wait1:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
    # Result of executing the action
        return client1.get_result()

    if not wait2:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
    # Result of executing the action
        return client2.get_result()

    if not wait3:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
    # Result of executing the action
        return client3.get_result()

    if not wait4:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
    # Result of executing the action
        return client4.get_result()   

    

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