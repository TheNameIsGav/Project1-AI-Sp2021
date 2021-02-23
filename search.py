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

    class DFSNode: 
        costSoFar = 1
        direct = Directions.NORTH
        pos = (0, 0)
        state = ((0,0), Directions.NORTH, 1)

        def __init__(self, c, d, p, s, pN):
            costSoFar = c + 1
            direct = d
            pos = p
            state = s
            self.previousNode = pN

    #Sets up the Open and Closed queues
    openNodes = Q()
    closedNodes = Q()

    #sets up the first node and adds it to the list of open nodes
    firstState = problem.getStartState()
    firstNode = DFSNode(1, None, firstState[0], firstState, None)
    openNodes.push(firstNode)

    while(not openNodes.isEmpty()): #Searches through all the nodes

        #Finds the oldest state and gets the successors from it
        currentNode = openNodes.list[0]
        connections = problem.getSuccessors(currentNode.pos) #Returns a list of states

        #Absolutely disgusting way of determining if the current node has been opened or closed
        isNodeOpen = -1
        for i in range(len(openNodes.list)):
            if currentNode.pos[0] == openNodes.list[i].pos[0] and currentNode.pos[1] == openNodes.list[i].pos[1]:
                isNodeOpen = i

        isNodeClosed = -1
        for i in range(len(closedNodes.list)):
            if currentNode.pos[0] == closedNodes.list[i].pos[0] and currentNode.pos[1] == closedNodes.list[i].pos[1]:
                isNodeClosed = i

        #Goes through the connections to the node and adds them to the appropriate list
        for state in connections:
            stateAsNode = DFSNode(state[0], state[1], state[2], state, currentNode)
            if not (isNodeOpen == -1):
                #if the node is already open check to see if the cost of the node in the list is lower then our current connection
                prevNode = openNodes.list[isNodeOpen]
                if prevNode.costSoFar <= stateAsNode.costSoFar:
                    continue
                else:
                    openNodes.list[isNodeOpen] = stateAsNode

            elif not (isNodeClosed == -1):
                #if the node is closed, check to see if our root is less expensive, and if so then reopen it
                prevNode = closedNodes.list[isNodeClosed]
                if prevNode.costSoFar <= stateAsNode.costSoFar:
                    continue
                else:
                    closedNodes.list.pop(isNodeClosed)
                    openNodes.push(stateAsNode)
            else:
                openNodes.push(stateAsNode)

        #End of For Loop

        openNodes.list.remove(currentNode)
        closedNodes.push(currentNode)

    #End of While Loop

    #Finds the node that is the destination
    current = closedNodes.list[0]
    for node in closedNodes.list:
        if problem.isGoalState(node.state):
            current = node
    
    #Builds an inverted path of directions
    path = []
    while(current.previousNode != None):
        path.append(current.direct)
        current = current.previousNode

    #reverses the path so that we can follow it
    revPath = []
    for i in range(len(path)):
        revPath.append(path.pop())


    return revPath
    """
    #Tester information?
    #print("Start:", problem.getStartState())
    #print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    #print("Start's successors:", problem.getSuccessors(problem.getStartState()))"""


def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    util.raiseNotDefined()

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
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
    util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
