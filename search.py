# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).

import searchAgents
"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem):
	"""
	Search the deepest nodes in the search tree first [p 74].
  
	Your search algorithm needs to return a list of actions that reaches
	the goal.  Make sure to implement a graph search algorithm [Fig. 3.18].
  
	To get started, you might want to try some of these simple commands to
	understand the search problem that is being passed in:
	"""
	print ''
	print "Start:", problem.getStartState()
	print "Is the start a goal?", problem.isGoalState(problem.getStartState())
	print "Start's successors:", problem.getSuccessors(problem.getStartState())
  
	"*** YOUR CODE HERE ***"  
	visited = dict()
	state = problem.getStartState()

	frontier = util.Stack()
	visited_node=1

	explored_node = {}
	explored_node["parent"] = None
	explored_node["action"] = None
	explored_node["state"] = state
	frontier.push(explored_node)

	while not frontier.isEmpty():
		visited_node=visited_node+1
		explored_node = frontier.pop()
		state = explored_node["state"]
		if visited.has_key(hash(state)):
			continue
		visited[hash(state)] = True

		if problem.isGoalState(state) == True:
			break

		for child in problem.getSuccessors(state):
			if not visited.has_key(hash(child[0])):
				son_node = {}
				son_node["parent"] = explored_node
				son_node["action"] = child[1]
				son_node["state"] = child[0]
				frontier.push(son_node)
	
	actions = []
	while explored_node["action"] != None:
		actions.insert(0, explored_node["action"])
		explored_node = explored_node["parent"]
	print ''
	print 'Search agent arrives to the initial position'
	print 'Final position:', state[0]
	print 'Visited nodes:',visited_node
	return actions
def breadthFirstSearch(problem):
	"Search the shallowest nodes in the search tree first. [p 74]" 
	"*** YOUR CODE HERE ***"
	print ''
	print "Start state:", problem.getStartState()
	print "Is the start a goal?", problem.isGoalState(problem.getStartState())
	print "Start's successors:", problem.getSuccessors(problem.getStartState())
	
	frontier = util.Queue()
	visited = dict()
	state = problem.getStartState()
	explored_node = {}
	explored_node["parent"] = None
	explored_node["action"] = None
	explored_node["state"] = state
	frontier.push(explored_node)
	visited_node=1
	while not frontier.isEmpty():
		visited_node=visited_node+1
		explored_node = frontier.pop()
		state = explored_node["state"]
		if visited.has_key(state):
			continue
		visited[state] = True
		if problem.isGoalState(state) == True:

			break
		for child in problem.getSuccessors(state):
			if child[0] not in visited:
				son_node = {}
				son_node["parent"] = explored_node
				son_node["state"] = child[0]
				son_node["action"] = child[1]
				frontier.push(son_node)
	actions = []
	while explored_node["action"] != None:
		actions.insert(0, explored_node["action"])
		explored_node = explored_node["parent"]
	print ''
	print 'Search agent arrives to the initial position'
	print 'Final position:', state[0]
	print 'Visited nodes:',visited_node
	return actions
def iDeepeningSearch(problem):
	print ''
	print "Start:", problem.getStartState()
	print "Is the start a goal?", problem.isGoalState(problem.getStartState())
	print "Start's successors:", problem.getSuccessors(problem.getStartState())
	
	visited_node=0
	for i in range(1000):
		actions,visited=DFS_it(problem,i)
		visited_node=visited_node + visited
		if  actions== 0:
			continue
		else:
			print 'Visited nodes:',visited_node
			print "Iterations: " , i
			return actions
			break
		
def DFS_it(problem,i):
	visited = dict()
	state = problem.getStartState()
	frontier = util.Stack()
	visited_node=1
	explored_node = {}
	explored_node["parent"] = None
	explored_node["action"] = None
	explored_node["state"] = state
	frontier.push(explored_node)

	while not frontier.isEmpty():
		visited_node=visited_node+1
		explored_node = frontier.pop()
		state = explored_node["state"]
		if visited.has_key(hash(state)):
			continue
		visited[hash(state)] = True

		if problem.isGoalState(state) == True:
			break
		if i==0:
			break
		i=i-1
		for child in problem.getSuccessors(state):

			if not visited.has_key(hash(child[0])):
				son_node = {}
				son_node["parent"] = explored_node
				son_node["action"] = child[1]
				son_node["state"] = child[0]
				frontier.push(son_node)
		
	actions = []
	if problem.isGoalState(state) == True:
		while explored_node["action"] != None:
			actions.insert(0, explored_node["action"])
			explored_node = explored_node["parent"]
		print ''
		print 'Search agent arrives to the initial position'
		print 'Final position:', state[0]
		return actions,visited_node
	else:
		return 0,visited_node
def nullHeuristic(state, problem=None):
	"""
	A heuristic function estimates the cost from the current state to the nearest
	goal in the provided SearchProblem.  This heuristic is trivial.
	"""
	return 0
def cornersHeuristic(state, problem):

	corners = problem.corners # These are the corner coordinates
	walls = problem.walls # These are the walls of the maze, as a Grid (game.py)
  
	position = state[0]
	corner_state = list(state[1])

	if problem.isGoalState(state):
		return 0

	unvisited_corners = []
	for u in range(len(corners)):
		if corner_state[u] == 0:
			unvisited_corners.append(corners[u])

	current_pos = position
	total_cost = 0
	while len(unvisited_corners) != 0:
		i, dist = lower_Distance(current_pos, unvisited_corners)
		total_cost += dist
		current_pos = unvisited_corners[i]
		unvisited_corners.remove(unvisited_corners[i])
	return total_cost
def lower_Distance(actual_pos, corners):
	index = -1
	min = None
	for i in range(len(corners)):
		dist = util.manhattanDistance(actual_pos, corners[i])
		if min == None or min > dist:
			index = i
			min = dist
	return index, min

def aStarSearch(problem, heuristic=nullHeuristic):
	"""Search the node that has the lowest combined cost and heuristic first."""
	"*** YOUR CODE HERE ***"
	print ''
	print "Start:", problem.getStartState()
	print "Is the start a goal?", problem.isGoalState(problem.getStartState())
	print "Start's successors:", problem.getSuccessors(problem.getStartState())
	
	frontier = util.PriorityQueue()
	visited = dict()
	visited_node=1
	state = problem.getStartState()
	explored_node = {}
	explored_node["parent"] = None
	explored_node["action"] = None
	explored_node["state"] = state
	explored_node["cost"] = 0
	explored_node["eval"] = heuristic(state, problem)
	frontier.push(explored_node, explored_node["cost"] + explored_node["eval"])

	while not frontier.isEmpty():
		visited_node=visited_node+1
		explored_node = frontier.pop()
		state = explored_node["state"]
		cost = explored_node["cost"]
		v = explored_node["eval"]
		if visited.has_key(state):
			continue	
		visited[state] = True
		if problem.isGoalState(state) == True:
			break
		for child in problem.getSuccessors(state):
			if not visited.has_key(child[0]):
				son_node = {}
				son_node["parent"] = explored_node
				son_node["state"] = child[0]
				son_node["action"] = child[1]
				son_node["cost"] = child[2] + cost
				son_node["eval"] = heuristic(son_node["state"], problem)
				frontier.push(son_node, son_node["cost"] + explored_node["eval"])

	actions = []
	while explored_node["action"] != None:
		actions.insert(0, explored_node["action"])
		explored_node = explored_node["parent"]
	print ''
	print 'Search agent arrives to the initial position'
	print 'Final position:', state[0]
	print 'Visited nodes:',visited_node
	return actions
def bidirectionalSearch(problem):
	print ''
	print "Start:", problem.getStartState()
	print "Is the start a goal?", problem.isGoalState(problem.getStartState())
	print "Start's successors:", problem.getSuccessors(problem.getStartState())
	frontier1 = util.Queue()
	frontier2 = util.Queue()
	exp_n1 = list()
	exp_n2 = list()
	frontier1.push((problem.getStartState(), list()))
	frontier2.push((problem.getGoalState(), list()))
	visited_node=1
	goal2 = problem.getStartState()
	while not frontier1.isEmpty():
		node1 = frontier1.pop()
		for data in problem.getSuccessors(node1[0]):
			if not data[0] in exp_n1:
				if problem.isGoalState(data[0]):
					return node1[1] + [data[1]]
				frontier1.push((data[0], node1[1] + [data[1]]))
				exp_n1.append(data[0])
		if not frontier2.isEmpty():	
			node2 = frontier2.pop()
			for data in problem.getSuccessors(node2[0]):
				if not data[0] in exp_n2:
					if data[0] == goal2:
						return [data[1]] + node2[1][::-1]
					frontier2.push((data[0], node2[1] + [data[1]]))
					exp_n2.append(data[0])
		for node in frontier1.list:
			for elem in frontier2.list:
				if node[0] == elem[0]:
					print 'Solution find in state', node[0]
					actions1 = node[1]
					actions2 = elem[1]
					actions2 = actions2[::-1]
					for i in range(len(actions2)):
						if actions2[i] == "North": actions2[i] = "South"
						elif actions2[i] == "South": actions2[i] = "North"
						elif actions2[i] == "West": actions2[i] = "East"
						elif actions2[i] == "East": actions2[i] = "West"
					return actions1 + actions2
	return []	
# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
ids = iDeepeningSearch
astar = aStarSearch
bs= bidirectionalSearch
