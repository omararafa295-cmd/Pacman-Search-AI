"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""
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

def depthFirstSearch(problem: SearchProblem):
    """
    Search the deepest nodes in the search tree first.
    """
 
    fringe = util.Stack()
    return genericGraphSearch(problem, fringe)

def breadthFirstSearch(problem: SearchProblem):
    """Search the shallowest nodes in the search tree first."""
 
    fringe = util.Queue()
    return genericGraphSearch(problem, fringe)

def uniformCostSearch(problem: SearchProblem):
    """Search the node of least total cost first."""
  
    fringe = util.PriorityQueue()
    return genericGraphSearch(problem, fringe, usePriority=True)

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem: SearchProblem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    
    fringe = util.PriorityQueue()
    return genericGraphSearch(problem, fringe, usePriority=True, heuristic=heuristic)

def genericGraphSearch(problem, fringe, usePriority=False, heuristic=None):
    """
    خوارزمية بحث عامة تطبق Graph Search لتجنب تكرار زيارة الحالات. 
    """
    visited = [] 

    startNode = (problem.getStartState(), [], 0)
    
    if usePriority:
        fringe.push(startNode, 0)
    else:
        fringe.push(startNode)

    while not fringe.isEmpty():
        state, actions, cost = fringe.pop()

       
        if problem.isGoalState(state):
            return actions

        if state not in visited:
            visited.append(state)
            
            for successor, action, stepCost in problem.getSuccessors(state):
                newActions = actions + [action]
                newCost = cost + stepCost
                
                priority = newCost
                if heuristic:
                    priority += heuristic(successor, problem)
                
                newNode = (successor, newActions, newCost)
                
                if usePriority:
                    fringe.push(newNode, priority)
                else:
                    fringe.push(newNode)
    return []

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch