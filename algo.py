import numpy as np
from state import next_state, solved_state
from location import next_location
from collections import OrderedDict
import time



class Node:
    def __init__(self,state,parent,depth, action):
        self.state = state
        self.parent = parent
        self.depth = depth
        self.action = action
    
    def __hash__(self) -> int:
        return hash(str(self.state))

    def copy(self) :
        return Node(self.state,self.parent,self.depth,self.action)



def compare_states(state1,state2):
    """
    check two numpy array are equal or not
    
    Args:
        state1 (numpy.array): the state to check
        state2 (numpy.array): the state to check

    Returns:
        boolean: the result of check
    """
    comparsion = state1 == state2
    return comparsion.all()



def solve(init_state, init_location, method):
    """
    Solves the given Rubik's cube using the selected search algorithm.
 
    Args:
        init_state (numpy.array): Initial state of the Rubik's cube.
        init_location (numpy.array): Initial location of the little cubes.
        method (str): Name of the search algorithm.
 
    Returns:
        list: The sequence of actions needed to solve the Rubik's cube.
    """

    # instructions and hints:
    # 1. use 'solved_state()' to obtain the goal state.
    # 2. use 'next_state()' to obtain the next state when taking an action .
    # 3. use 'next_location()' to obtain the next location of the little cubes when taking an action.
    # 4. you can use 'Set', 'Dictionary', 'OrderedDict', and 'heapq' as efficient data structures.

    if method == 'Random':
        return list(np.random.randint(1, 12+1, 10))
    
    elif method == 'IDS-DFS':

        def appendToFrontier(frontier,explored,new_state, action , parent_path):
            if(not hash(str(new_state)) in explored and not hash(str(new_state)) in frontier):
                _path = parent_path.copy()
                if(action > 0):
                    _path.append(action)
                node = (new_state , _path)
                frontier[hash(str(node[0]))] = node
            elif(hash(str(new_state)) in explored):
                _path_length = explored[hash(str(new_state))]
                if(len(parent_path)+1 < _path_length ):
                    _path = parent_path.copy()
                    if(action > 0):
                        _path.append(action)
                    node = (new_state , _path)
                    frontier[hash(str(node[0]))] = node
                    explored[hash(str(new_state))] = len(_path)
                                    

        def ld_dfs(limited_depth):
            frontier = OrderedDict()
            explored = {}
            goal = None
            appendToFrontier(frontier,explored,init_state,0,[])
            while(len(frontier) > 0):
                _hash , _node = frontier.popitem(True)
                state , path = _node
                if(compare_states(state,solved_state())):
                    goal = path
                    print(path)
                    break
                elif (limited_depth > len(path)):
                    for action in range(1,13):
                        _state = next_state(state,action)
                        appendToFrontier(frontier=frontier,action=action,explored=explored,
                                        new_state=_state,parent_path=path.copy())  
                explored[_hash] = len(path)                        
            return goal

        def id_dfs():
            path = None
            _goal = None
            _limited_depth = 0
            while(path is None):
                _limited_depth = _limited_depth + 1
                print(_limited_depth)
                path = ld_dfs(_limited_depth)
            return path 

        path = id_dfs()
        print(str(path),type(path[1]))
        return path
        

    elif method == 'A*':
        ...

    elif method == 'BiBFS':
        ...
    
    else:
        return []