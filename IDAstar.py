"""
1) Uncomment the main function if you need to run any test cases
2 Solving 8 puzzle using IDA* algorithm

"""
import numpy as np
import math
import time
import copy

loop_counter = list()  # used to count the number of moves made


class Node:
    def __init__(self, state):
        self.state = state

    def __repr__(self):
        return np.array_str(self.state.flatten())

    def __hash__(self):  # returns the hash value of an object if it has one
        return hash(self.__repr__())

    def __eq__(self, other):  # used to compare the two object
        return self.__hash__() == other.__hash__()

    def get_position(self):  # get position of the 0 element in the given state
        pos = [i[0] for i in np.where(self.state == 0)]  # Numpy function which returns the 0 position in n*n array
        return pos

    def nextnodes(self):  # check for valid moves (up, down, right, left), return only valid moves

        zero_position = Node.get_position(self)  # get position of the 0 element in the given state

        x, y = zero_position  # co-ordinates values

        x = int(x)
        y = int(y)

        # final states which needs to be evaluated
        up = (x + 1, y)
        down = (y - 1, y)
        right = (x, y + 1)
        left = (x - 1, y)

        zero = np.where(self.state == 0)  # get values zero position using numpy

        temp_array = list()
        for tile_move in (up, down, right, left):
            # making sure value doesnt cross the co-ordinate values. eg: 3 tile problem [0,1,2]
            # create the moves only its its valid within the boundary
            if len(self.state) - 1 >= tile_move[1] >= 0 and len(self.state) - 1 >= tile_move[0] >= 0:
                temp = np.copy(self.state)  # shallow copy
                temp[tile_move[0], tile_move[1]], temp[zero] = temp[zero], temp[
                    tile_move[0], tile_move[1]]  # swapping will take place
                temp_array.append(Node(temp))  # append the value to temporary array
        return temp_array, len(temp_array)


def IDAS(initial_state, goal_state):
    initial_node = Node(initial_state)  # Create object of the Node (initial_node)
    goal_node = Node(goal_state)

    threshold = manh(initial_state,
                     goal_state)  # Calcualte Manhattan heuristic value initially, This variable will decide the number of nodes explored

    while 1:  # Infinite loop untill its found
        path = set([initial_node])  # paths explored

        temp = search(initial_node, goal_state, 0, threshold, path)  # g(n) initailly is set to 0

        # print("temp: {}".format(temp))
        if temp:  # if found exit the function print the values
            return True, threshold
        elif temp == float('inf'):  # if not found exit the function, display errors
            return False, float('inf')
        else:
            threshold = temp


def search(node, goal_state, cost, threshold, path):  # cost = g(n)
    f = cost + manh(node.state, goal_state)  # calcualte f(n) = g(n) + h(n)

    if f > threshold:  # Threshold value exceed, return the values
        return f

    if np.array_equal(node.state, goal_state):  # check for goal state reached or not
        return True

    minimum = math.inf  # set value to infinite
    nextnodearray, length = node.nextnodes()  # search for valid moves

    # loop_counter += length
    loop_counter.append(length)
    for n in nextnodearray:  # itterate the next moves
        # print("n--- > ", n)
        if n not in path:  # if new state is found
            path.add(n)  # add the explaored path
            temp = search(n, goal_state, cost + 1, threshold, path)  # recursive call the function

            if temp:  # if found in child then return truw
                return True
            if temp < minimum:
                minimum = temp

    return minimum


## hvaluefunction is used to get the co-ordinate the of a number in give 8/15 tile problem
def hvalue(item, ray2d):  # Take one of the (initial/goal) array
    hvalue_list = list()
    for r in range(len(ray2d)):
        for c in range(len(ray2d[0])):
            if ray2d[r][c] == item:
                hvalue_list.append((r, c))
    return hvalue_list[0]


def manh(i1, g1):
    h = 0  # initial value to zero

    for i in range(9):
        r1, c1 = hvalue(i, i1)  # get the co-ordinates of initial state
        # for example, Number 3 is in (1,0)
        # print('r1, c1', r1 ,c1)

        r2, c2 = hvalue(i, g1)  # get the co-ordinates of a number corresponding to initial state
        # for example, Number 3 is in (2,0)
        # print('r2, c2', r2 ,c2)

        h += abs(r1 - r2) + abs(c1 - c2)  # calculate absolute difference and sum up
        # heuristic value

    return h


def flatten(raw_input):  #
    flatten_list = list()  # Declare empty list
    for element in raw_input[2:]:  # start from 2nd position
        # print(element)
        if type(element) is list:
            for item in element:
                # print(item)
                flatten_list.append(item)
    return flatten_list  # return in list without 'zero's poistion'


#####################################################################################
#########   USE ME FOR TESTING SINGLE VALUES ########################################
#####################################################################################
# if __name__ == "__main__":
#     initial_state = np.array([1, 2, 3, 8, 6, 4, 7, 5, 0]).reshape(3, 3)
#     goal_state = np.array([1, 2, 3, 8, 0, 4, 7, 6, 5]).reshape(3, 3)
#
#     t = time.process_time() #sum of the system and user CPU time of the current process
#     start = time.time() #start timer
#     result, val = IDAS(initial_state, goal_state) #calling the iterative deepning A* Algo
#     # print('Loop Counters', sum(loop_counter))
#     elapsed = time.process_time() - t
#     end = time.time()
#     clockTime = end - start
#
#     if result:
#         print("Number of moves taken to complete : ", sum(loop_counter))
#         print('the number of yield of your move procedure made during the search for a solution : ', val)
#         print("Time Taken to complete the search : {:8.4f}".format(elapsed, clockTime))
#     else:
#         print("Solution not found!")

#
if __name__ == "__main__":

    # Provide the Initial state as given in the assignment
    input_list = [[0, 0, [[0, 7, 1], [4, 3, 2], [8, 6, 5]]], [0, 2, [[5, 6, 0], [1, 3, 8], [4, 7, 2]]],
                  [2, 0, [[3, 5, 6], [1, 2, 7], [0, 8, 4]]],
                  [1, 1, [[7, 3, 5], [4, 0, 2], [8, 1, 6]]], [2, 0, [[6, 4, 8], [7, 1, 3], [0, 2, 5]]]]
    # Goal state as shown in the Assignment
    goal_list = [0, 2, [[3, 2, 0], [6, 1, 8], [4, 7, 5]]]

    for i in input_list:
        flat_input = flatten(i)  # Trimming the Tile zero position
        flat_goal = flatten(goal_list)  # Trimming the Tile zero position
        dimension = len(i[2])  # Dimension of the list eg: 3*3, 4*4
        try:
            initial_state = np.array(flat_input).reshape(dimension, dimension)  # convert to numpy arrray
            goal_state = np.array(flat_goal).reshape(dimension, dimension)
            t = time.process_time()  # sum of the system and user CPU time of the current process
            start = time.time()  # start timer
            result, val = IDAS(initial_state, goal_state)  # calling the iterative deepening A* Algo
            # print('Loop Counters', sum(loop_counter))
            elapsed = time.process_time() - t
            end = time.time()  # end timmer
            clockTime = end - start
            print("initial state \n", initial_state)
            print("goal state \n", goal_state)

            if result:

                print("Number of moves taken to complete : ", sum(loop_counter))
                print('the number of yield of your move procedure made during the search for a solution : ', val)
                print("Time Taken to complete the search : {:8.2f}".format(elapsed, clockTime))
                print('\n')
                print('----------------------------------------------------------------------------------------------')
            else:
                print("Solution not found!")

        except:
            print("check the input format")

    print("First half of the program is complete")
    ######################
    ## Format of the input is same as above
    #####################

    input_list2 = [0, 0, [[0, 1, 8], [3, 6, 7], [5, 4, 2]]], [2, 0, [[6, 4, 1], [7, 3, 2], [0, 5, 8]]], [0, 0,
                                                                                                         [[0, 7, 1],
                                                                                                          [5, 4, 8],
                                                                                                          [6, 2, 3]]], [
                      0, 2, [[5, 4, 0], [2, 3, 1], [8, 7, 6]]], [2, 1, [[8, 6, 7], [2, 5, 4], [3, 0, 1]]]
    # Goal state as shown in the Assignment
    goal_list2 = [0, 2, [[3, 2, 0], [6, 1, 8], [4, 7, 5]]]

    for i in input_list2:
        flat_input = flatten(i)  # Trimming the Tile zero position
        flat_goal = flatten(goal_list2)  # Trimming the Tile zero position
        dimension = len(i[2])  # Dimension of the list eg: 3*3, 4*4
        try:
            initial_state = np.array(flat_input).reshape(dimension, dimension)  # convert to numpy arrray
            goal_state = np.array(flat_goal).reshape(dimension, dimension)
            t = time.process_time()
            start = time.time()
            result, val = IDAS(initial_state, goal_state)
            # print('Loop Counters', sum(loop_counter))
            elapsed = time.process_time() - t
            end = time.time()
            clockTime = end - start
            print("initial state \n", initial_state)
            print("goal state \n", goal_state)

            if result:

                print("Number of moves taken to complete : ", sum(loop_counter))
                print('the number of yield of your move procedure made during the search for a solution : ', val)
                print("Time Taken to complete the search : {:8.2f}".format(elapsed, clockTime))
                print('\n')
                print('----------------------------------------------------------------------------------------------')
            else:
                print("Solution not found!")

        except:
            print("check the input format")

    print("end")
