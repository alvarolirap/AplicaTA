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

def inFrontier(node, frontier):
	
	for nodeInFrontier in frontier:
		if node["state"] == nodeInFrontier["state"]:
			return nodeInFrontier,True
	return None,False
		

def bidirectionalSearch(problem):
	print ''
	print "Start:", problem.getStartState()
	print "Is the start a goal?", problem.isGoalState(problem.getStartState())
	print "Start's successors:", problem.getSuccessors(problem.getStartState())

	visited1 = dict()
	state1 = problem.getStartState()
	frontier1 = util.Queue()
	explored_node1 = {}
	explored_node1["parent"] = None
	explored_node1["action"] = None
	explored_node1["state"] = state1
	frontier1.push(explored_node1)

	visited2 = dict()
	state2 = problem.getGoalState()
	frontier2 = util.Queue()
	explored_node2 = {}
	explored_node2["parent"] = None
	explored_node2["action"] = None
	explored_node2["state"] = state2
	frontier2.push(explored_node2)

	visited_node = 2

	while not frontier1.isEmpty() and not frontier2.isEmpty():
		if not frontier1.isEmpty():
			explored_node1 = frontier1.pop()
			visited_node = visited_node + 1
			state1 = explored_node1["state"]
			if not visited1.has_key(hash(state1)):
				visited1[hash(state1)] = True
				if problem.isGoalState(state1) == True:
					break
				nodoTemp, res = inFrontier(explored_node1,frontier2.list)
				if res:
					explored_node2 = nodoTemp 
					break
				for child in problem.getSuccessors(state1):
					if not visited1.has_key(hash(child[0])):
						son_node = {}
						son_node["parent"] = explored_node1
						son_node["action"] = child[1]
						son_node["state"] = child[0]
						frontier1.push(son_node)

		if not frontier2.isEmpty():
			explored_node2 = frontier2.pop()
			visited_node = visited_node + 1
			state2 = explored_node2["state"]
			if not visited2.has_key(hash(state2)):
				visited2[hash(state2)] = True
				if problem.getStartState() == state2:
					break
				nodoTemp, res = inFrontier(explored_node2,frontier1.list)
				if res:
					explored_node1 = nodoTemp 
					break
				for child in problem.getInvertedSuccessors(state2):
					if not visited2.has_key(hash(child[0])):
						son_node = {}
						son_node["parent"] = explored_node2
						son_node["action"] = child[1]
						son_node["state"] = child[0]
						frontier2.push(son_node)
	
	actions1 = []
	actions2temp = []
	print ''
	print 'Search algorithms meet at the position:', explored_node1["state"][0]
	print 'Visited nodes:',visited_node
	
	while explored_node1["action"] != None:
		actions1.insert(0, explored_node1["action"])
		explored_node1 = explored_node1["parent"]

	while explored_node2["action"] != None:
		actions2temp.insert(0, explored_node2["action"])
		explored_node2 = explored_node2["parent"]

	actions2 = []
	for i in range(0,len(actions2temp)):
		if actions2temp[len(actions2temp)-1-i] == "North": actions2.append("South")
		elif actions2temp[len(actions2temp)-1-i] == "South": actions2.append("North")
		elif actions2temp[len(actions2temp)-1-i] == "West": actions2.append("East")
		elif actions2temp[len(actions2temp)-1-i] == "East": actions2.append("West")
	
	return actions1 + actions2


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
ids = iDeepeningSearch
astar = aStarSearch
bs= bidirectionalSearch
