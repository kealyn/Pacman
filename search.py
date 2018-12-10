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

class Node:
    """
    Class of node which is pushed and popped from the fringe for search exploration
    """
    def __init__(self, state, moves, cost = 0):
        self.state = state
        self.moves = moves
        self.cost  = cost

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    """
    return search_algorithm(problem,
                            util.Stack)

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    return search_algorithm(problem,
                            util.Queue)


def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    return search_algorithm(problem,
                            util.PriorityQueue,
                            True)

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    return search_algorithm(problem,
                            util.PriorityQueue,
                            True,
                            heuristic)


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch


def search_algorithm(problem, fringeContainer, considerPriority = False, heuristic = nullHeuristic):
    """
    A generic search algorithm that searches from start state until goal state is reached.

    :param problem: the problem
    :param fringeContainer: the container type of the fringe
    :param considerPriority: flag that indicates whether the priority is considered
    :param heuristic: heuristic that is used in computing priority (astar)
    :return: Sequence of moves
    """
    startState = problem.getStartState()
    if problem.isGoalState(startState):
        # start state is goal state
        return []

    fringe  = fringeContainer()  # fringe that holds states to be visited
    moves   = []                 # sequence of moves
    visited = set()              # set of visited states

    if not considerPriority:
        fringe.push(Node(startState, moves))
    else:
        fringe.push(Node(startState, moves, 0), 0)
    while not fringe.isEmpty():
        curr = fringe.pop()
        cur_state = curr.state
        cur_moves = curr.moves
        cur_cost  = curr.cost

        # check for goal state
        if problem.isGoalState(cur_state):
            return cur_moves

        # skip or add to visited
        if cur_state in visited:
            continue
        visited.add(cur_state)

        # get successors
        successors = problem.getSuccessors(cur_state)
        for triplet in successors:
            state = triplet[0]
            action = triplet[1]
            cost = triplet[2]
            if not considerPriority:
                fringe.push(Node(state, cur_moves + [action], cost))
            else:
                cummulative_cost = cur_cost + cost + heuristic(state, problem)
                fringe.update(
                    Node(state, cur_moves + [action], cur_cost + cost),
                    cummulative_cost
                )

    return []
