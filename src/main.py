#!/usr/bin/env python

import rospy
import argparse
import numpy as np
from nav_msgs.msg import Path
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation
from A_star import problem, A_star_search

def shortest_path_publisher(args):
    # Initialize node and publisher
    pub = rospy.Publisher('myPath', Path, queue_size=1)
    rospy.init_node('AStarPublisher', anonymous=True)

    # Get A star path
    prob = problem(filepath = args.filepath, 
                    heuristic = args.heuristic,
                    source = args.source,
                    goal = args.goal)
    solution = A_star_search(prob)

    print ("Shortest path is - ")
    if not solution:
        print ("No path exists")
    for node in solution:
        print (node.state)

    # create message
    msg = Path()
    
    # create message header
    msg.header = Header()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = 'map'
    msg.poses = [] 

    for i in range(0, len(solution)):
        msg.poses.append(PoseStamped())
    
    #create poses
    for i, node in enumerate(solution):
        #create header part of Path message
        msg.poses[i].header = Header()
        msg.poses[i].header.seq = i+1
        msg.poses[i].header.stamp = rospy.Time.now()
        msg.poses[i].header.frame_id = 'map'

        #create position part of Path message
        msg.poses[i].pose.position.x = node.coordinates[0]
        msg.poses[i].pose.position.y = node.coordinates[1]
        msg.poses[i].pose.position.z = node.coordinates[2]

        #calculating orientation
        if (i<len(solution) - 1):
            next = solution[i+1].coordinates
            vector = np.array(next) - np.array(node.coordinates)
            vector = vector/np.linalg.norm(vector)
            vector[2] = 0 #since our bot will move only vertically up or down, we only need the yaw.
                          # Hence we project the direction vector to xy plane to calculate orientation

            angle = np.arctan2(vector[1], vector[0])
            
            #rotate "angle" radians around Z axis
            rotation = Rotation.from_rotvec(angle*np.array([0, 0, 1]))
            rotation = rotation.as_quat()

            #create orientation part of Path message
            msg.poses[i].pose.orientation.x = rotation[0]
            msg.poses[i].pose.orientation.y = rotation[1]
            msg.poses[i].pose.orientation.z = rotation[2]
            msg.poses[i].pose.orientation.w = rotation[3]

        #for the last state, keep the same orientation
        if i == len(solution) - 1:
            msg.poses[i].pose.orientation = msg.poses[i-1].pose.orientation

    pub.publish(msg)
    rospy.loginfo(msg)
    rospy.spin()

def parse_args():
    parser = argparse.ArgumentParser(description="A star")
    parser.add_argument(
        '--filepath',
        metavar='path to graph.csv',
        type=str,
        required=True,
        help='path to graph.csv'
    )

    parser.add_argument(
        '--heuristic',
        metavar='heuristic to use',
        type=str,
        required=True,
        help='heuristic for A star'
    )
    
    parser.add_argument(
        '--source',
        metavar='source state',
        type=int,
        required=True,
        help='source state'
    )
    
    parser.add_argument(
        '--goal',
        metavar='goal state',
        type=int,
        required=True,
        help='goal state'
    )

    return parser.parse_args()



if __name__ == '__main__':
    args = parse_args()
    try:
        shortest_path_publisher(args)
    except rospy.ROSInterruptException:
        pass

