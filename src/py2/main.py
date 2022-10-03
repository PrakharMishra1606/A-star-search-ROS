#!/usr/bin/env python

import rospy
import numpy as np
from nav_msgs.msg import Path
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Pose
from scipy.spatial.transform import Rotation


from A_star_2 import problem, A_star_search

def talker():
    # Initialize node and publisher
    pub = rospy.Publisher('myPath', Path, queue_size=1)
    rospy.init_node('AStarPublisher', anonymous=True)
    rate = rospy.Rate(10) #10Hz

    # Get A star path
    graph_csv_file = "/home/prakhar/ros_workspaces/prakhar_ws/src/prakhar_accio_task/params/graph.csv"
    prob = problem(filepath = graph_csv_file, 
                    heuristic = "euclidean",
                    source = 10,
                    goal = 100)
    solution = A_star_search(prob)

    # create message
    msg = Path()
    
    # message header
    msg.header = Header()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = 'map'
    msg.poses = [] 

    for i in range(0, len(solution)):
        msg.poses.append(PoseStamped())
    
    #poses header
    for i, node in enumerate(solution):
        #header
        msg.poses[i].header = Header()
        msg.poses[i].header.seq = i+1
        msg.poses[i].header.stamp = rospy.Time.now()
        msg.poses[i].header.frame_id = 'map'

        #pose
        msg.poses[i].pose.position.x = node.coordinates[0]
        msg.poses[i].pose.position.y = node.coordinates[1]
        msg.poses[i].pose.position.z = node.coordinates[2]

        if (i<len(solution) - 1):
            next = solution[i+1].coordinates
            vector = np.array(next) - np.array(node.coordinates)
            vector = vector/np.linalg.norm(vector)


            rotation = Rotation.from_matrix(np.array([  [vector[0], -vector[1] , 0],
                                                        [vector[1], vector[0],   0],
                                                        [0,          0,          1]
                                                    ]))
            rotation2 = rotation.as_quat()
            print(rotation.as_euler('XYZ'))

            msg.poses[i].pose.orientation.x = rotation2[0]
            msg.poses[i].pose.orientation.y = rotation2[1]
            msg.poses[i].pose.orientation.z = rotation2[2]
            msg.poses[i].pose.orientation.w = rotation2[3]

        if i == len(solution) - 1:
            msg.poses[i].pose.orientation = msg.poses[i-1].pose.orientation




    # while not rospy.is_shutdown():
        #publish message
    pub.publish(msg)
    rospy.loginfo(msg)
        # rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

