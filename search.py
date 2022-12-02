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
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    n = Directions.NORTH
    e = Directions.EAST
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:
    """
    from util import Stack
    
    stiva = Stack(); 
    visited = [];
    path = []
    father = {} #dictionary that will help me make the output list of directions    
    stiva.push(problem.getStartState()); 
    current = problem.getStartState();
    father[current] = 'Nimic';
    while (not stiva.isEmpty() ):
    	current= stiva.pop();
    	visited.append(current);
    	if(problem.isGoalState(current)):
    		path = calculatePath(father,current,problem.getStartState()); 
    		break;	
    	for node, direc, cost in problem.getSuccessors(current) :
    		f = 0;
    		if node not in visited:
   			f=1;
   		if f==1:
   			stiva.push(node);
   			father[node] = (current,direc);
    
    return path
    util.raiseNotDefined()

def calculatePath(father, goal, start):
    path = []
    while(goal != start):
    	path.append(father[goal][1])
    	goal = father[goal][0];
    path.reverse();
    return path;	    	

def breadthFirstSearch(problem):

    from util import Queue

    queue = Queue(); 
    visited_grey = [];
    visited = [];
    father = {} #dictionary that will help me make the output list of directions
    path = []
    
    queue.push(problem.getStartState()); 
    current = problem.getStartState();
    visited_grey.append(current);
    #father[current] = 0;
    
    
    while (not queue.isEmpty() ):
    	current = queue.pop();
    	if(problem.isGoalState(current)):
    		path= calculatePath(father,current,problem.getStartState());
    		break;
    	for node, direc, cost in problem.getSuccessors(current) :
    		f = 0;
    		if node not in visited and node not in visited_grey:
   			f=1;
   		if f==1:
   			visited_grey.append(node);
   			father[node] = (current,direc);
   			queue.push(node);
    	visited.append(current);
   
    return path
    util.raiseNotDefined()
    
def breadthFirstSearch2(problem):

    from util import Queue

    queue = Queue(); 
    visited = [];
    path = []

    current = problem.getStartState()
    if(problem.isGoalState(current)):
        return path

    queue.push((current, []))
    while not queue.isEmpty():
        current, path = queue.pop()

        if (not current in visited) :
            visited.append(current)

            if problem.isGoalState(current) :
                return path
	    succ = problem.getSuccessors(current)
            for node, direc, cost in succ:
                queue.push((node, path + [direc]))

    return path
    util.raiseNotDefined()
def uniformCostSearch(problem): 
    
    #pe asta tre sa l facem amu cu PriorityQueue
    #punem un element in coada si facem suma costului pana acolo
    
    
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    from util import PriorityQueue
    pqueue = PriorityQueue(); 
    visited = [];
    visited_grey = [];
    father = {} #dictionary that will help me make the output list of directions
    co = {}   
    pqueue.push(problem.getStartState(), 0);  #incepem cu cost zero (cost zero sa ajungem la start, de acolo plecam)
    current = problem.getStartState();
    father[current] = 'Nimic';
    co[current] = 0;
    
    while (not pqueue.isEmpty() ):
    	current = pqueue.pop();
    	if(problem.isGoalState(current)):
    		path= calculatePath(father,current,problem.getStartState());
    		break;
    	for node, direc, cost in problem.getSuccessors(current) :
    		f = 0;
    		if node not in visited: 
    			if node in visited_grey:
    				if co[node] > (co[father[node][0]] + cost):
   					father[node] = (current,direc,cost);
   					co[node] = co[father[node][0]] + cost
   					pqueue.update(node,co[node]);
    			elif node not in visited:
   				f=1;
   			if f==1:
   				visited_grey.append(node);
   				father[node] = (current,direc,cost);
   				co[node] = co[father[node][0]] + cost
   				pqueue.update(node,co[node]);
   	visited.append(current);
   			
    return path;
    util.raiseNotDefined()

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0			

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    
    from util import PriorityQueue
    
    start = problem.getStartState()
    print start,
    path = [] #drumul care ma duce la goal
    pqueue = PriorityQueue()
    visited = [] #nodurile deja vizitate
    
    if(problem.isGoalState(start)):
    	return path;
    
    pqueue.push((start,path,0), 0) #primul zero este costul curent, al doilea zero este prioritatea
    
    while(not pqueue.isEmpty()):
    	current,path,cost_acum = pqueue.pop()
    	if not(current in visited):
    		visited.append(current)
    
    		if(problem.isGoalState(current)):
    			break;	
    			
    		for node, direc, cost in problem.getSuccessors(current) :
    			cost_nou = cost_acum + cost
    			heuristicCost = cost_nou + heuristic(node, problem)
    			pqueue.push((node,path + [direc],cost_nou), heuristicCost)
    			
    return path
    util.raiseNotDefined()

def aStarWeighted(problem,heuristic=nullHeuristic):
     w = 2 #ponderea 
     from util import PriorityQueue

     pqueue = PriorityQueue()
     start = problem.getStartState()
     path = []
     visited = []
     heuristicCost = w * heuristic(start,problem)
     data = (start,[],0)
     pqueue.push(data,0);
     while( not pqueue.isEmpty() ):
        current,path,g = pqueue.pop();
        if not(current in visited):
    		visited.append(current)
        	if(problem.isGoalState(current)):
            		break;
        	for node,direc,cost in problem.getSuccessors(current):
            		heuristicCost = w * heuristic(node,problem)
            		data = (node,path+[direc],g+cost)
            		pqueue.push(data, heuristicCost)
            
     return path;
     util.raiseNotDefined()
     
def aStarWeighted_forGreedy(problem,w,heuristic=nullHeuristic):
     from util import PriorityQueue

     pqueue = PriorityQueue()
     start = problem.getStartState()
     path = []
     visited = []
     heuristicCost = w * heuristic(start,problem)
     data = (start,[],0)
     pqueue.push(data,0);
     while( not pqueue.isEmpty() ):
        current,path,g = pqueue.pop();
        if not(current in visited):
    		visited.append(current)
        	if(problem.isGoalState(current)):
            		break;
        	for node,direc,cost in problem.getSuccessors(current):
            		heuristicCost = w * heuristic(node,problem)
            		data = (node,path+[direc],(1-w)*(g+cost))
            		pqueue.push(data, heuristicCost)
            
     return path;
     util.raiseNotDefined()
# Abbreviations
bfs2 = breadthFirstSearch2
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
asw = aStarWeighted
