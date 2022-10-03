#!/usr/bin/env python2

import csv
import heapq
from sklearn.neighbors import DistanceMetric


class node():
    def __init__(self, state, location, parent = None, action= None, path_cost = 0.0):
        self.state = state
        self.coordinates = location
        self.parent = parent
        self.action = action # an integer from 1 to 6, which represents which node to go to
        self.path_cost = path_cost


# graph is encoded in the problem
class problem():
     
    def __init__(self, filepath, heuristic, source, goal = 1062):
        '''
        Initializes the problem with graph, goal and source
        '''
        self.graph = dict()

        with open(filepath, mode='r') as file:
            csvFile = csv.reader(file)

            for line in csvFile:
                line = line[:-1]
                
                uuid = int( line[0])
                x = float( line[1] )
                y = float( line[2] )
                z = float( line[3] ) 
                node1 = int( line[4] ) 
                node2 = int( line[5] )
                node3 = int( line[6] ) 
                node4 = int( line[7] ) 
                node5 = int( line[8] ) 
                node6 = int( line[9] )
                weight1 = int( line[10] )
                weight2 = int( line[11] )
                weight3 = int( line[12] )
                weight4 = int( line[13] )
                weight5 = int( line[14] ) 
                weight6 = int( line[15] )

                data = {
                        'x': x, 
                        'y': y, 
                        'z': z, 
                        'coordinates' : [x,y,z],
                        'node1' : { "child": node1, "weight": weight1}, 
                        'node2' : { "child": node2, "weight": weight2},
                        'node3' : { "child": node3, "weight": weight3}, 
                        'node4' : { "child": node4, "weight": weight4},
                        'node5' : { "child": node5, "weight": weight5},
                        'node6' : { "child": node6, "weight": weight6}
                    }

                self.graph[uuid] = data

        # source node
        self.initial_node = node(source, location = self.graph[source]['coordinates'] ,parent=None, action=None, path_cost=0.0)
        
        # don't use goal node for any other purpose than storing state and coordinates
        self.goal_node = node(goal, location = self.graph[goal]['coordinates'])
        self.heuristic = heuristic
    
    def goal_test(self, state):
        '''
        Returns True if input state is the goal state
        '''
        return state == self.goal_node.state

    def actions(self, state):
        '''
        Returns the list of actions available from the input state

        Output:
        ['node1','node2']
        '''
        action_list = []

        node_dict = self.graph[state]

        for key, value in node_dict.items():
            if isinstance(value, dict):
                if value["child"]:
                    action_list.append(key)

        return action_list

    def result(self, parent_state, action):
        '''
        Returns the state reached when action is taken at parent_state
        '''
        return self.graph[parent_state][action]["child"]

    def step_cost(self, parent_state, action):
        '''
        Returns the step cost - the weight associated with action at parent_state
        '''
        return self.graph[parent_state][action]['weight']

    def child_node(self, parent_node, action):
        '''
        Creates and returns a child node of parent_node when action is taken
        '''
        state = self.result(parent_node.state, action)
        location = self.graph[state]['coordinates']
        path_cost = parent_node.path_cost + self.step_cost(parent_node.state, action)
        
        return node(state, location, parent_node, action, path_cost)

    def heuristic_value(self, n):
        '''
        Returns h(n) for node n
        '''
        dist = DistanceMetric.get_metric(self.heuristic)
        a = n.coordinates
        b = self.goal_node.coordinates
        X = [a, b]
        return dist.pairwise(X)[0][1]
        
    def f(self, n):
        '''
        returns the total cost for a node n
        '''
        g = n.path_cost
        h = self.heuristic_value(n)
        return float(g+h)

    # should return sequence of actions of sequence of nodes?
    def solution(self, node):
        '''
        returns list of nodes odered source to goal node (bactracks)
        '''  
        path = [node]
        while(node.parent is not None):
            path.append(node.parent)
            node = node.parent

        path.reverse()
        return path


class priority_queue():
    def __init__(self):
        self.list = []
        heapq.heapify(self.list)
    
    def isEmpty(self):
        '''
        Returns True if frontier is empty
        '''
        return not self.list

    def doesNotContain(self, state):
        '''
        Returns True if child not in frontier,
                False if child is in frontier
        '''
        for data in self.list:
            _, s, _ = data
            if s == state:
                return False
        return True

    def f_cost(self, n):
        '''
        Returns the f_cost of the node n. f_cost is the estimated total cost of the path through node n to goal 
        '''

        for data in self.list:
            f_cost, state, _ = data
            if state == n.state:
                return f_cost
        print("INPUT NODE NOT IN FRONTIER!!!")

    def replace(self, n, child_cost):
        '''
        Replaces node n in frontier with a lower f_cost
        '''
        for i, data in enumerate(self.list):
            _, s, _ = data
            if s == n.state:
                self.list.pop(i)
                break
        self.insert(child_cost, n.state, n)


    def pop(self):
        '''
        pop and return node with lowest cost
        '''
        return heapq.heappop(self.list)

    def insert(self, f_cost, state, node):
        '''
        Insert node with state and f_cost into frontier
        '''
        heapq.heappush(self.list, (f_cost, node.state, node) )


def A_star_search(prob):
    node = prob.initial_node
    frontier = priority_queue()
    frontier.insert(prob.f(node), node.state, node)
    
    explored = set()
    
    while(True):
        # If frontier is empty, then no path exists
        if frontier.isEmpty():
            return [] #failure
        
        # POP(frontier)
        _, state, node = frontier.pop()

        # GOAL TEST
        if prob.goal_test(node.state):
            return prob.solution(node)

        # Explore a node's children. Mark node as explored
        explored.add(node.state)

        for action in prob.actions(node.state):
            child = prob.child_node(node, action)
            
            if( (child.state not in explored) and frontier.doesNotContain(child.state) ):
                frontier.insert(prob.f(child), child.state, child)

            
            # replace child node if it exists in frontier with higher cost
            elif frontier.doesNotContain(child.state) == False:
                # retrieve child f cost
                x = frontier.f_cost(child)
                # if f cost is higher than child.f cost
                child_cost = prob.f(child)
                if x > prob.f(child):
                    #replace()
                    frontier.replace(child, child_cost)
            

if __name__ == "__main__":
    graph_csv_file = "/home/prakhar/ws/Accio-Task/params/graph.csv"
    
    #INITIALIZE THE PROBLEM
    prob = problem(filepath = graph_csv_file,
                    heuristic = "euclidean",
                    source = 1,
                    goal = 10)
    
    solution = A_star_search(prob)

    for node in solution:
        print(node.state)
    
    
    


