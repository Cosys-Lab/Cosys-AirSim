#!/usr/bin/env python
# license removed for brevity

import rospy

# Brings in the SimpleActionClient
import actionlib
# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionResult
from geometry_msgs.msg import Pose, Point, Quaternion
from actionlib_msgs.msg import GoalStatusArray
from posecsv import PoseCSV

finished = False
goalnr = 0
def _result_cb(result):
    global finished
    try:
        if len(result.status_list) == 1:
            finished = result.status_list[0].status == 3
        else:
            finished = False
    except Exception as e:
        pass

def movebase_client():
    rospy.loginfo("Starting...")
   # Create an action client called "move_base" with action definition file "MoveBaseAction"
    for i in range(5):
        client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    
    # Waits until the action server has started up and started listening for goals.
        if client.wait_for_server(timeout=rospy.Duration(5.0)):
            rospy.loginfo("Server up and running...")
            break
        else:
            rospy.logerr("Server timeout")
            if i == 4:
                raise Exception("Timeout occured")

    #['/move_base/result', 'move_base_msgs/MoveBaseActionResult']
    rospy.Subscriber('move_base/status', GoalStatusArray, callback=_result_cb) # cb's of action client not returning...
    global goalnr
    global finished

    poseReader = PoseCSV("/home/jori/projects/hyslam/airsim/ros/python_ws/src/airsim/scripts/poses_example_benchmark_env_one_round.csv", False)
    poses = poseReader.readPoses()
    lengthPoses = len(poses)

    while not rospy.is_shutdown():
    # Creates a new goal with the MoveBaseGoal constructor
        goal = MoveBaseGoal()
        #goal.target_pose.header.frame_id = "base_link"
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
    # Move 0.5 meters forward along the x axis of the "map" coordinate frame 
    # No rotation of the mobile base frame w.r.t. map frame
        #goal.target_pose.pose = Pose(Point(10, 0.0, 0.000), Quaternion(0, 0, 0, 1))
        #goal.target_pose.pose = Pose(Point(-0.06, -11.97, 0.25), Quaternion(0, 0, 0.71, 0.71))
        pose = poses[goalnr]
        goal.target_pose.pose = pose
        rospy.loginfo("Intermediate goal set: {}".format(goal))
    # Sends the goal to the action server.
        client.send_goal(goal) #cb not returning: , active_cb=activeCallback, done_cb=doneCallback, feedback_cb=feedbackCallback)
        rospy.loginfo("Send goal...: "+ rospy.remap_name('move_base'))
        finished = False

        while not finished and not rospy.is_shutdown():
            pass
        
        goalnr = goalnr + 1
        
        if goalnr == lengthPoses:
            poseReader.close()
            return True
        """
    # Waits for the server to finish performing the action.
        wait = client.wait_for_result()
    # If the result doesn't arrive, assume the Server is not available
        if not wait:
            rospy.logerr("Action server not available!")
            #rospy.signal_shutdown("Action server not available!")
        else:
        # Result of executing the action
            return client.get_result()  
        """ 
    poseReader.close()
    return False

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