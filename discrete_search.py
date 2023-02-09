ALG_BFS = "bfs"
ALG_DFS = "dfs"
ALG_ASTAR = "astar"
from queue import Queue



class StateSpace:
    """A base class to specify a state space X"""
    
    def __contains__(self, x):
        """Return whether the given state x is in the state space"""
        

        raise NotImplementedError

    def get_distance_lower_bound(self, x1, x2):
        """Return the lower bound on the distance between the given states x1 and x2"""
        return 0


class ActionSpace:
    """A base class to specify an action space"""

    def __call__(self, x):
        """ Return the list of all the possible actions at the given state x"""
        raise NotImplementedError


class StateTransition:
    """A base class to specify a state transition function"""

    def __call__(self, x, u):
        """Return the new state obtained by applying action u at state x"""
        raise NotImplementedError


def fsearch(X, U, f, xI, XG, alg):
    """Return the list of visited nodes and a path from xI to XG based on the given algorithm

    This is the general template for forward search describe in Figure 2.4 in the textbook.

    @type X:   an instance of the StateSpace class (or its derived class) that represent the state space
    @type U:   an instance of the ActionSpace class (or its derived class) such that
               U(x) returns the list of all the possible actions at state x
    @type f:   an instance of the StateTransition class  (or its derived class) such that
               f(x, u) returns the new state obtained by applying action u at cell x
    @type xI:  an initial state such that xI in X
    @type XG:  a list of states that specify the goal set
    @type alg: a string in {"bfs", "dfs", "astar"} that specifies the discrete search algorithm to use

    @return:   a dictionary {"visited": visited_states, "path": path} where visited_states is the list of
               states visited during the search and path is a path from xI to a state in XG
    """
    # TODO: Implement this function

    

    queue = get_queue(alg, X, XG)
    visited = set()
    queue.insert(xI, None)

    while queue.queue:
        x = queue.pop()
        print("start",x)
        if x in visited:
            continue
        visited.add(x)
        
        if x in XG:
            path = []
            while x is not None:
                path.insert(0, x)
                x = queue.parent[x]
            return {'visited': list(visited), 'path': path}
        for u in U(x):
            print("the actions",u)
            x_new = u
            print("new actions",x_new)
            
            if x_new not in visited:
                queue.insert(x_new, x)

    return {'visited': list(visited), 'path': []}

    raise NotImplementedError

class Queue:
    def __init__(self):
        self.queue = []
        self.parent = {}

    def insert(self, x, parent):
        pass

    def pop(self):
        pass


class QueueBFS(Queue):
    def insert(self, x, parent):
        self.queue.append(x)
        self.parent[x] = parent

    def pop(self):
        return self.queue.pop(0)


class QueueDFS(Queue):
    def insert(self, x, parent):
        self.queue.insert(0, x)
        self.parent[x] = parent

    def pop(self):
        return self.queue.pop(0)


class QueueAstar(Queue):
    def __init__(self, X,XI XG):
        self.X = X
        self.XG = XG
        self.queue = []
        self.parent = {}
        self.cost = {}
        self.cost[XI]=0

    def insert(self, x, parent):
        self.queue.append(x)
        self.parent[x] = parent
        if parent is not None:
            self.cost[x] = self.cost[parent] + 1
        else:
            self.cost[x] = 0
        for goal in self.XG:
            self.cost[x] += self.X.get_distance_lower_bound(x, goal)

    def pop(self):
        index = self.queue.index(min(self.queue, key=lambda x: self.cost[x]))
        x = self.queue.pop(index)
        return x


def get_queue(alg, X,XI, XG):
    if alg == 'bfs':
        return QueueBFS()
    elif alg == 'dfs':
        return QueueDFS()
    elif alg == 'astar':
        return QueueAstar(X,XI, XG)

    