import numpy as np
from state import next_state, solved_state
from location import next_location,solved_location
from collections import OrderedDict
import heapq



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


        def appendToFrontier(frontier,new_state, action , parent_path):
            if(not hash(str(new_state)) in frontier):
                _path = parent_path.copy()
                if(action > 0):
                    _path.append(action)
                node = (new_state , _path)
                frontier[hash(str(node[0]))] = node
                                    

        def ld_dfs(limited_depth):
            frontier = OrderedDict()
            explored = {}
            numberOfExploration = 0
            numberOfExpansion = 0
            goal = None
            appendToFrontier(frontier,init_state,0,[])
            while(len(frontier) > 0):
                _hash , _node = frontier.popitem(True)
                state , path = _node
                if(not _hash in explored or explored[_hash] > len(path)):    
                    if(compare_states(state,solved_state())):
                        goal = path
                        print(path)
                        break
                    elif (limited_depth > len(path)):
                        for action in range(1,13):
                            _state = next_state(state,action)
                            numberOfExpansion+=1
                            appendToFrontier(frontier=frontier,action=action,
                                            new_state=_state,parent_path=path.copy())  
                    explored[_hash] = len(path)                        
                    numberOfExploration+=1
            return goal,numberOfExpansion,numberOfExploration

        def id_dfs():
            path = None
            _goal = None
            _limited_depth = 0
            numberOfExploration = 0
            numberOfExpansion = 0
            while(path is None):
                _limited_depth = _limited_depth + 1
                path,_numberOfExpansion,_numberOfExploration = ld_dfs(_limited_depth)
                numberOfExploration+=_numberOfExploration
                numberOfExpansion+=_numberOfExpansion
                print("depth"+str(_limited_depth))
                print("numberOfExploration"+str(numberOfExploration))
                print("numberOfExpansion"+str(numberOfExpansion))
            return path,numberOfExpansion,numberOfExploration 

        path,numberOfExpansion,numberOfExploration = id_dfs()
        print("numberOfExploration",numberOfExploration)
        print("numberOfExpansion",numberOfExpansion)
        return path
        

    elif method == 'A*':
        def heuristic(location):
            distances = []
            _solved_location = solved_location()
            for i in range (1,9):
                x_solved , y_solved,z_solved = np.where(_solved_location == i)
                x_loc , y_loc, z_loc = np.where(location==i)
                distances.append(abs(x_solved[0]-x_loc[0])+abs(y_solved[0]-y_loc[0])+abs(z_solved[0]-z_loc[0]))
            return sum(distances)/4
    
        def a_star():
            explored = {}
            frontier = []
            states = {}
            locations = {}
            path = []
            numberOfExpansion = 0
            numberOfExploration = 0
            _init_state_estimate = heuristic(init_location)
            states[hash(str(init_state))] = init_state
            locations[hash(str(init_location))] = init_location
            heapq.heappush(frontier,(_init_state_estimate,hash(str(init_state)),hash(str(init_location)),[]))
            while(len(frontier)>0):
                numberOfExploration +=1
                _estimate, _current_state_hash,_current_location_hash,_path = heapq.heappop(frontier)
                if(not _current_state_hash in explored or explored[_current_state_hash] > _estimate):
                    if(compare_states(states[_current_state_hash],solved_state())):
                        path = _path
                        break
                    else:
                        for action in range(1,13):
                            numberOfExpansion += 1
                            _next_state = next_state(states[_current_state_hash],action)
                            _next_location = next_location(locations[_current_location_hash],action)
                            _next_path = _path.copy()
                            _next_path.append(action)
                            _next_state_estimate = heuristic(_next_location) + len(_next_path)
                            states[hash(str(_next_state))] = _next_state
                            locations[hash(str(_next_location))] = _next_location
                            heapq.heappush(frontier,(_next_state_estimate,hash(str(_next_state)),hash(str(_next_location)),_next_path))
                    explored[_current_state_hash] = len(_path)
            return path,numberOfExpansion,numberOfExploration
        
        path,numberOfExpansion,numberOfExploration = a_star()
        print(str(path))
        print("numberOfExploration",str(numberOfExploration))
        print("numberOfExpansion",str(numberOfExpansion))
        return path

    elif method == 'BiBFS':
        def find_reverse_of_action(action):
            result = action
            if(action > 6):
                result = action - 6
            else:
                result = action + 6

            return result
        
        def bi_bfs():
            src_frontier = OrderedDict()
            dist_frontier= OrderedDict()
            number_of_explores = 0
            number_of_expansion = 0
            src_explored = {}
            dist_explored = {}
            path = []
            src_frontier[hash(str(init_state))] = (init_state,[])
            dist_frontier[hash(str(dist_frontier))] = (solved_state(),[])
            while(len(src_frontier) > 0 and len(dist_frontier) >0):
                _explored_commons_keys = set(src_frontier).intersection(set(dist_frontier))
                if(len(_explored_commons_keys) > 0):
                    _common_state_hash = _explored_commons_keys.pop()
                    _,from_src_path = src_frontier[_common_state_hash]
                    _,from_dist_path = dist_frontier[_common_state_hash]
                    from_dist_path.reverse()
                    for i in range(len(from_dist_path)):
                        from_dist_path[i] = find_reverse_of_action(from_dist_path[i])
                    path = from_src_path + from_dist_path
                    break
                else:
                    _src_current_hash,(_src_current_state,_src_path) = src_frontier.popitem(False)
                    _dist_current_hash,(_dist_current_state,_dist_path) = dist_frontier.popitem(False)
                    number_of_explores += 2
                    if(not _src_current_hash in src_explored):
                        if(number_of_explores % 10000 == 0):
                            print(number_of_explores)
                        for action in range (1,13):
                            _new_state = next_state(_src_current_state,action)
                            _new_path = _src_path.copy()
                            _new_path.append(action)
                            src_frontier[hash(str(_new_state))] = (_new_state,_new_path)
                            number_of_expansion += 1
                    if(not _dist_current_hash in dist_explored):
                        for action in range (1,13):
                            _new_state = next_state(_dist_current_state,action)
                            _new_path = _dist_path.copy()
                            _new_path.append(action)
                            dist_frontier[hash(str(_new_state))] = (_new_state,_new_path)
                            number_of_expansion += 1
                    src_explored[_src_current_hash] = _src_path
                    dist_explored[_dist_current_hash] = _dist_path
            return path,number_of_expansion,number_of_explores

        path,numberOfExpansion,numberOfExploration = bi_bfs()
        print("numberOfExpansion",str(numberOfExpansion))
        print("numberOfExploration",str(numberOfExploration))
        return path

    
    else:
        return []