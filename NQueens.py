from queue import Queue
from collections import deque

# PriorityQueue will be used in A* search
class PriorityQueue(object):
    def __init__(self):
        self.queue = []

    def pop(self):
        top = 0
        for i in range(self.size()):
            if self.queue[i][0] < self.queue[top][0]:
                top = i
        node = self.queue[top]
        del self.queue[top]
        return node

    def remove(self, node):
        heapq.heapify(self.queue)
        self.queue.remove(node)
        heapq.heapify(self.queue)

    def _iter_(self):
        return iter(sorted(self.queue))

    def append(self, node):
        self.queue.append(node)

    def _contains_(self, key):
        return key in [n[-1] for n in self.queue]

    def size(self):
        return len(self.queue)

    def top(self):
        return self.queue[0]

class NQueenProblemSolver:

    def __init__(self, N = 8):

        self.boardSize = N
        self.initialStates = []
        self.solutionCost = 0
        self.searchCost = 0
        self.maxFrontier = 0
        self.solution = [[0 for i in range(N)] for j in range(N)]

    def get_initialStates(self):
        self.initialStates = [(queen_pos,) for queen_pos in range(self.boardSize)]
        return self.initialStates

    def get_solutionCost(self):
        return self.solutionCost

    def get_searchCost(self):
        return self.searchCost

    def get_maxFrontier(self):
        return self.maxFrontier

    def BFS_Strategy(self, initialStates):

        frontier = deque(initialStates)

        while frontier:

            # update max size of the frontier
            if self.maxFrontier < len(frontier):
                self.maxFrontier = len(frontier)

            state = frontier.popleft()

            # steps to reach a solution
            self.solutionCost += 1

            if len(state) == self.boardSize:
                return state
            for queen in range(self.boardSize):

                # nodes generated before reaching solution
                self.searchCost += 1

                # check if valid before inserting to frontier
                if not self.isValid(state, queen):
                    continue
                next_state = state + (queen,)
                frontier.append(next_state)

    def Iterative_Deepening(self, state, maxDepth, depth=0):

        # steps to reach a solution
        self.solutionCost += 1

        # update max size of the frontier
        if self.maxFrontier < len(state):
            self.maxFrontier = len(state)

        if len(state) == self.boardSize:
            return state, True

        if depth == maxDepth:
            if len(state) > 0:
                return None, False
            else:
                return None, True

        bottom = True
        for queen in range(self.boardSize):

            # nodes generated before reaching solution
            self.searchCost += 1

            if not self.isValid(state, queen):
                continue
            next_state = state + (queen,)
            solution, bottom_reached = self.Iterative_Deepening(next_state, maxDepth, depth + 1)

            if solution is not None:
                return solution, True
            bottom = bottom and bottom_reached

        return None, bottom

    def IDS_Strategy(self, initialStates):

        depth = 1
        bottom = False
        start = (0,)

        while not bottom:
            solution, bottom = self.Iterative_Deepening(state=start, maxDepth= depth)
            if solution is not None:
                return solution
            depth *= 2

        return None

    # simple heuristic more queens placed is better
    def heuristic(self, state):
        return len(state)

    def A_Star_Strategy(self, state=(0,)):

        frontier = PriorityQueue()
        h = self.heuristic(state)
        frontier.append((h, 0, state))

        while frontier.size():

            node = frontier.pop()
            h = node[0]
            g = node[1]
            state = node[2]

            # steps to reach a solution
            self.solutionCost += 1

            # update max size of the frontier
            if self.maxFrontier < len(state):
                self.maxFrontier = len(state)

            if len(state) == self.boardSize:
                return state

            neighbors = PriorityQueue()

            # h heuristic to goal, and g actual cost to neighbors
            for queen in range(self.boardSize):

                # nodes generated before reaching solution
                self.searchCost += 1

                if not self.isValid(state, queen):
                    continue
                next_state = state + (queen,)
                h = self.heuristic(next_state)
                g = node[1] + 1
                neighbors.append((h+g, g, next_state))

            for _ in neighbors._iter_():
                child = neighbors.pop()

                if (not frontier._contains_(child[-1])):
                    frontier.append(child)

                elif frontier._contains_(child[-1]):
                    for n in frontier._iter_():
                        # check for states and heuristics
                        if (n[-1] == child[-1]) and (n[0] > child[0]): 
                            frontier.remove(n)
                            frontier.append(child)
                            break

    def isValid(self, state, new_queen):
        for col, row in enumerate(state):
            if abs(row - new_queen) == abs(col - len(state)) or\
            new_queen in state:
              return False
        return True

    def assign_queens(self, state):
        for i in range(self.boardSize):
            for j in range(self.boardSize):
                if state[i] == j:
                    self.solution[i][j] = 1
                else: 
                    self.solution[i][j] = 0

    def printSolution(self):
        for i in range(self.boardSize):
            line = ""
            for j in range(self.boardSize):
                if self.solution[i][j] == 1:
                    line += "Q\t"
                else:
                    line += "_\t"
            print("\n")        
            print(line)
        print("\n")               
                    
def main():

    print('N-Queens Problem')
    # default number of n-queens is 8
    n_queens = 8
    # user input number of n-queens
    n_queens = int(input('Please enter the size of board: '))

    #BFS Strategy
    print('\n\n1. BFS strategy')

    # intialize n-queens class object
    BFS = NQueenProblemSolver(N = n_queens)
    # putting queens in first row
    state = BFS.get_initialStates()
    # applying strategy
    solution = BFS.BFS_Strategy(initialStates = state)
    # assign queens to the board
    BFS.assign_queens(solution)
    # print solution board
    BFS.printSolution()
    # number of steps to reach solution
    print('Solution cost is %d' %BFS.get_solutionCost())
    # number of nodes generated before reaching a solution
    print('Search cost is %d' %BFS.get_searchCost())
    # maximum size of the frontier
    print('Maximum frontier size is %d' %BFS.get_maxFrontier())

    #IDS Strategy
    print('\n\n2. IDS strategy')

    # intialize n-queens class object
    IDS = NQueenProblemSolver(N = n_queens)
    # putting queens in first row
    state = IDS.get_initialStates()
    # applying strategy
    solution = IDS.IDS_Strategy(initialStates = state)
    # assign queens
    IDS.assign_queens(solution)
    # print solution board
    IDS.printSolution()
    # number of steps to reach solution
    print('Solution cost is %d' %IDS.get_solutionCost())
    # search cost: number of nodes generated before reaching a solution
    print('Search cost is %d' %IDS.get_searchCost())
    # maximum size of the frontier
    print('Maximum frontier size is %d' %IDS.get_maxFrontier())

    #A* strategy
    print('\n\n3. A* strategy')

    # intialize n-queens class object
    ASTAR = NQueenProblemSolver(N = n_queens)
    # putting queens in first row
    state = ASTAR.get_initialStates()
    # applying strategy
    solution = ASTAR.A_Star_Strategy()
    # assign queens
    ASTAR.assign_queens(solution)
    # print solution board
    ASTAR.printSolution()
    # number of steps to reach solution
    print('Solution cost is %d' %ASTAR.get_solutionCost())
    # number of nodes generated before reaching a solution
    print('Search cost is %d' %ASTAR.get_searchCost())
    # maximum size of the frontier
    print('Maximum frontier size is %d' %ASTAR.get_maxFrontier())
    
if __name__ == '__main__':
    main()