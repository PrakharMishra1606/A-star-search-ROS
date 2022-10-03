#!/usr/bin/env python3

import csv
import heapq
from typing import List, Tuple
from sklearn.metrics import DistanceMetric


class node():
    def __init__(self, state:int, location: List[int], parent = None, action:int = None, path_cost: float = 0.0):
        self.state :int = state
        self.coordinates = location
        self.parent :node = parent
        self.action = action # an integer from 1 to 6, which represents which node to go to
        self.path_cost :float = path_cost


# graph is encoded in the problem
class problem():
     
    def __init__(self, filepath:str, heuristic, source:int = 1, goal:int = 1062) -> None:
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
    
    def goal_test(self, state: int) -> bool:
        '''
        Returns True if input state is the goal state
        '''
        return state == self.goal_node.state

    def actions(self, state: int) -> List[str]:
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

    def result(self, parent_state:int, action:str) -> int:
        '''
        Returns the state reached when action is taken at parent_state
        '''
        return self.graph[parent_state][action]["child"]

    def step_cost(self, parent_state:int, action:str) -> float:
        '''
        Returns the step cost - the weight associated with action at parent_state
        '''
        return self.graph[parent_state][action]['weight']

    def child_node(self, parent_node:node, action:str) -> node:
        '''
        Creates and returns a child node of parent_node when action is taken
        '''
        state = self.result(parent_node.state, action)
        location = self.graph[state]['coordinates']
        path_cost = parent_node.path_cost + self.step_cost(parent_node.state, action)
        
        return node(state, location, parent_node, action, path_cost)

    def heuristic_value(self, n: node) -> float:
        dist = DistanceMetric.get_metric(self.heuristic)
        a = n.coordinates
        b = self.goal_node.coordinates
        X = [a, b]
        return dist.pairwise(X)[0][1]
        
    def f(self, n: node) -> float:
        '''
        returns the total cost for a node n
        '''
        g = n.path_cost
        h = self.heuristic_value(n)
        return float(g+h)

    # should return sequence of actions of sequence of nodes?
    def solution(self, node: node) -> List[node]:
        '''
        returns list of nodes odered source to goal node
        '''  
        solution = [node]
        while(node.parent is not None):
            solution.append(node.parent)
            node = node.parent

        solution.reverse()
        return solution


class priority_queue():
    def __init__(self):
        self.list = []
        heapq.heapify(self.list)
    
    def isEmpty(self) -> bool:
        return not self.list

    # is this being called?
    def length(self) -> int:
        return len(self.list)

    def doesNotContain(self, state:int) -> bool:
        '''
        Returns True if child not in frontier,
                False if child is in frontier
        '''
        for data in self.list:
            _, s, _ = data
            if s == state:
                return False
        return True

    def f_cost(self, n:node) -> float:
        '''
        Returns the f_cost of the node n
        '''

        for data in self.list:
            f_cost, state, _ = data
            if state == n.state:
                return f_cost
        print("INPUT NODE NOT IN FRONTIER!!!")

    def replace(self, n:node, child_cost:float) -> None:
        '''
        Replaces n in frontier
        '''
        for i, data in enumerate(self.list):
            _, s, _ = data
            if s == n.state:
                self.list.pop(i)
                print(f"removed element {i}")
                break
        self.insert(child_cost, n.state, n)
        print("succesfully replaced")


    def pop(self) -> Tuple[float, int, node]:
        return heapq.heappop(self.list)

    def insert(self, f_cost:float, state:int, node:node):
        heapq.heappush(self.list, (f_cost, node.state, node) )


def A_star_search(prob: problem) -> List[node]:
    node = prob.initial_node
    frontier = priority_queue()
    frontier.insert(prob.f(node), node.state, node)
    #print( f"Initially frontier list is {frontier.list}")
    
    # set or dictionary?
    explored = set()
    
    while(True):
        # EMPTY?(FRONTIER)
        if frontier.isEmpty():
            return [] #failure
        
        # POP(frontier)
        _, state, node = frontier.pop()
        #print( f"After popping frontier list is {frontier.list}")

        # GOAL TEST
        if prob.goal_test(node.state):
            return prob.solution(node)

        explored.add(node.state)
        print( f"Explored is {explored}")

        for action in prob.actions(node.state):
            child = prob.child_node(node, action)
            
            if( (child.state not in explored) and frontier.doesNotContain(child.state) ):
                #print(f" frontier doesnotcontain? {child.state}  - {frontier.doesNotContain(child.state)}")
                frontier.insert(prob.f(child), child.state, child)
                #print(f" ABSENT - after inserting {frontier.list} ")
                #print( f"After adding child frontier list is {frontier.list}")

            
            # remaining
            elif frontier.doesNotContain(child.state) == False:
                # retrieve child f cost
                #print("Retrieving")
                x = frontier.f_cost(child)
                #print("retreived")
                # if f cost is higher than child.f cost
                child_cost = prob.f(child)
                if x > prob.f(child):
                    #replace()
                    #print("calling replace")
                    frontier.replace(child, child_cost)
                    #print("replaced")
            

if __name__ == "__main__":
    graph_csv_file = "/home/prakhar/ws/Accio-Task/params/graph.csv"
    
    #INITIALIZE THE PROBLEM
    prob = problem(filepath = graph_csv_file,
                    heuristic = "chebyshev",
                    source = 1,
                    goal = 10)
    
    solution = A_star_search(prob)

    for node in solution:
        print(node.state)
    
    
    

    # # TESTING
    # source = node(state = 1, location=prob.graph[1]['coordinates'], parent = None, action = None, path_cost=0.0)
    # dest1 = prob.child_node(source, action="node1")
    # dest2 = prob.child_node(dest1, action="node1")
    # dest3 = prob.child_node(dest2, action="node1")
    # dest4 = prob.child_node(dest3, action="node1")

    # s = prob.solution(dest4)

    
    # for node in s:
    #     print(node.state)



