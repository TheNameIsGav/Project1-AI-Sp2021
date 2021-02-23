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

#State is just a position
#Successors return the State, Direction, and cost

    class DFSNode: 
        def __init__(self, c, d, p, pN):
            self.costSoFar = c + 1
            self.direct = d
            self.pos = p
            self.previousNode = pN

    #Sets up the Open and Closed queues
    openNodes = Q()
    closedNodes = Q()

    #sets up the first node and adds it to the list of open nodes
    firstNode = DFSNode(0, None, problem.getStartState(), None)
    openNodes.push(firstNode)

    #Needs to be modified to search for the food
    while(not openNodes.isEmpty()): #Searches through all the nodes

        #Finds the oldest state and gets the successors from it
        currentNode = openNodes.list[0]
        connections = problem.getSuccessors(currentNode.pos) #Returns a list of positions that we can move to

        #Goes through the connections to the node and adds them to the appropriate list
        for state in connections:
            stateAsNode = DFSNode(currentNode.costSoFar+1, state[1], state[0], currentNode)

            #Figures out if the node is in the open or closed lists
            isNodeOpen = -1
            for i in range(len(openNodes.list)):
                if stateAsNode.pos[0] == openNodes.list[i].pos[0] and stateAsNode.pos[1] == openNodes.list[i].pos[1]:
                    isNodeOpen = i

            isNodeClosed = -1
            for i in range(len(closedNodes.list)):
                if stateAsNode.pos[0] == closedNodes.list[i].pos[0] and stateAsNode.pos[1] == closedNodes.list[i].pos[1]:
                    isNodeClosed = i
            ######
            
            #Checks to see what the condition is, the node is either open, closed, or neither. If open or closed, check to see if we're cheaper
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
        #Processing for "currentNode" node
        openNodes.list.remove(currentNode)
        closedNodes.push(currentNode)

    #End of While Loop

    #Finds the node that is the destination
    current = None
    for node in closedNodes.list:
        if problem.isGoalState(node.pos):
            current = node

    #Checks to see if we could find a path
    if current == None:
        return None

    #Builds an inverted path of directions
    path = []
    while(not current == None):
        path.append(current.direct)
        current = current.previousNode

    #reverses the path so that we can follow it (and removes None from the end)
    revPath = []
    for i in range(len(path)):
        revPath.append(path.pop())
    return revPath[1:]
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
