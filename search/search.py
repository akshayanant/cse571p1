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
from util import PriorityQueue


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
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())

    """
    res = []
    visit = []
    dfsRec(problem,problem.getStartState(),res,visit)
    return res
    
#Helper function to find depth frst search soultion recursively
def dfsRec(problem,curState,res,visit):
    if(problem.isGoalState(curState)):
        return True
    if(curState in visit):
        return False
    visit.append(curState)
    for suc in problem.getSuccessors(curState):
        nextState = suc[0]
        action = suc[1]
        res.append(action)
        if(dfsRec(problem,nextState,res,visit)):
            return True
        res.pop()
    return False

    


def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    res = []
    visit =set()
    curState = problem.getStartState()
    q = util.Queue()
    loc = Location(curState,[],0)
    q.push(loc)
    while(not q.isEmpty()):
        poll = q.pop()
        curState = poll.state
        if(problem.isGoalState(curState)):
            return poll.prev
        visit.add(curState)
        successors = problem.getSuccessors(curState)
        for suc in successors:
            nextState = suc[0]
            if(nextState in visit):
                continue
            copy = poll.prev[:]
            copy.append(suc[1])
            nextLoc = Location(nextState,copy,0)
            if(nextLoc in q.list):
                continue
            q.push(nextLoc)
    return []



def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    curState = problem.getStartState()
    q = PriorityQueue()
    start = Location(curState,[],0)
    q.push(start,0)
    visit = []
    exists = set()
    exists.add(start)
    while(not q.isEmpty()):
        next = q.pop()
        visit.append(next.state)
        exists.remove(next)
        if(problem.isGoalState(next.state)):
            return next.prev
        for suc in problem.getSuccessors(next.state):
            if(suc[0] in visit):
                continue
            copy = next.prev[:]
            copy.append(suc[1])
            newState = Location(suc[0],copy,next.cost+suc[2])
            q.update(newState,newState.cost)
            exists.add(newState)
    return []
    
def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    curState = problem.getStartState()
    q = PriorityQueue()
    start = Location(curState,[],0)
    q.push(start,0)
    visit = set()
    exists = set()
    exists.add(start)
    while(not q.isEmpty()):
        next = q.pop()
        visit.add(next.state)
        exists.remove(next)
        if(problem.isGoalState(next.state)):
            return next.prev
        for suc in problem.getSuccessors(next.state):
            if(suc[0] in visit):
                continue
            copy = next.prev[:]
            copy.append(suc[1])
            newState = Location(suc[0],copy,next.cost+suc[2])
            q.update(newState,newState.cost+heuristic(suc[0],problem))
            exists.add(newState)
    return []

class Location:
    """
    This class is used to store 
                1. the location in the grid with:
                2. the list of actions to reach the location 
                3. the cost of reaching this location from start location
    State is usedto uniquely identify the location, hence, __eq__() and __hash__ use this information

    """

    def __init__(self,state,prev,cost):
        self.state = state
        self.prev = prev
        self.cost = cost

    def __eq__(self, other):
        return self.state == other.state

    def __hash__(self):
        return hash(self.state)


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
