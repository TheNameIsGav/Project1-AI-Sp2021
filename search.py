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

from game import Directions
import util
from util import Queue as Q

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
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem):
    closedNodes = set() #always keeps seen nodes
    openNodes = util.Stack() #Stack bc stack
    startState = (problem.getStartState(), "", 1)
    startNode = (startState, []) 
    openNodes.push(startNode)
    while not openNodes.isEmpty(): #Go until we can see no more nodes
        ((pos, direction, cost), path) = openNodes.pop() #read in the state and the previous path
        currNode = ((pos, direction, cost), path) #Set this for funs sake
        if problem.isGoalState(pos): #see if we're at the goal
            return path
        if pos not in closedNodes: #if we've not seen this node before
            closedNodes.add(pos) #Add to closed node
            for (succPos, succDirect, succCost) in problem.getSuccessors(pos): #for each successor make a new node and add to list
                newNode = (succPos, succDirect, succCost)
                openNodes.push((newNode, currNode[1] + [succDirect])) #This is gross but it doesn't work if I just use succDirect????

def breadthFirstSearch(problem):
    closedNodes = set() #always keeps seen nodes
    openNodes = util.Queue() #Queue bc Queue <- only change in the code from DFS
    startState = (problem.getStartState(), "", 1)
    startNode = (startState, []) 
    openNodes.push(startNode)
    while not openNodes.isEmpty(): #Go until we can see no more nodes
        ((pos, direction, cost), path) = openNodes.pop() #read in the state and the previous path
        currNode = ((pos, direction, cost), path) #Set this for funs sake
        if problem.isGoalState(pos): #see if we're at the goal
            return path
        if pos not in closedNodes: #if we've not seen this node before
            closedNodes.add(pos) #Add to closed node
            for (succPos, succDirect, succCost) in problem.getSuccessors(pos): #for each successor make a new node and add to list
                newNode = (succPos, succDirect, succCost)
                openNodes.push((newNode, currNode[1] + [succDirect])) #This is gross but it doesn't work if I just use succDirect????

def uniformCostSearch(problem):
    closedNodes = set() #Same logic as before
    openNodes = util.PriorityQueue() #use a PQueue to devise the highest value thing
    startState = (problem.getStartState(), "", 1)
    startNode = (startState, [], 0)
    openNodes.push(startNode, 0) #Some extra shtuffs with cost
    while not openNodes.isEmpty():
        currNodes = openNodes.pop() #"simplified" this, just didnt work withou
        if problem.isGoalState(currNodes[0][0]): #"simplified"
            return currNodes[1] #"simplified"
        if currNodes[0][0] not in closedNodes: #"simplified"
            closedNodes.add(currNodes[0][0]) #"simplified"
            for state in problem.getSuccessors(currNodes[0][0]): #"simplified"
                openNodes.push((state, currNodes[1] + [state[1]], currNodes[2] + state[2]), currNodes[2] + state[2]) #"Oh its not a string waaaah im python"

def nullHeuristic(state, problem=None):
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    path = []
    closedNodes = set()
    hitNodes = util.Queue()
    openNodes = util.PriorityQueue()
    firstState = (problem.getStartState(), path, 0)
    openNodes.push(firstState, 0) #Enters new state and its prio
    hitNodes.push((problem.getStartState(), path))
    while openNodes:
        (currPos, currPath, currCost) = openNodes.pop() #gets highest value node
        hitNodes.pop()
        if currPos not in closedNodes: #close the node if we haven't been here before, handles cheaper paths bc pQueue
            closedNodes.add(currPos)
            path = currPath
        if problem.isGoalState(currPos): #Rebuild the path if goal found
            c = [i[1] for i in path] #i[1] is direction in (pos, direction)
            return c

        for (pos, direction, thisCost) in problem.getSuccessors(currPos): #For every neighbor
            if pos not in closedNodes: #Check to see that we haven't closed it already
                notInClosed = False
                for x in hitNodes.list: #Check to see that we don't double expand nodes, can't be open nodes bc it won't be a neighbor of itself
                    if pos == x[0] and thisCost > x[2]:
                        notInClosed = True
                if not notInClosed:
                    d = currPath[:] #Copy the array so we don't overwrite in the future
                    d.append((pos, direction)) #appends to path
                    cost = currCost + thisCost + heuristic(pos, problem) #Estimated distance + cost to get to this array + previous cost
                    newState = (pos, d, currCost + thisCost)
                    openNodes.push(newState, cost)
                    hitNodes.push(newState)


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
