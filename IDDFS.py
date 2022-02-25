"""
1) Uncomment the main function if you need to run any test cases
2 Solving 8 puzzle using IDDFS algorithm

"""
import numpy as np
import time
import copy


def flatten(raw_input):
    flatten_list = list()  # Declare empty list
    for element in raw_input[2:]:  # start from 2nd position
        # print(element)
        if type(element) is list:
            for item in element:
                # print(item)
                flatten_list.append(item)
    return flatten_list  # return in list without 'zero's poistion'


class Node:
    def __init__(self, state, parent, action, depth, step_cost, path_cost):
        self.state = state
        self.parent = parent  # parent node
        self.action = action  # move up, left, down, right
        self.depth = depth  # depth of the node in the tree
        self.step_cost = step_cost  # g(n), the cost to take the step
        self.path_cost = path_cost  # accumulated g(n), the cost to reach the current node
        # children node
        self.move_up = None
        self.move_left = None
        self.move_down = None
        self.move_right = None

    def __len__(self):
        # Dunder Method is used to get the number of depth
        return self.depth

    def get_position(self):
        pos = [i[0] for i in np.where(self.state == 0)]  # Numpy function which returns the 0 position in n*n array
        return pos

    def up(self):  # Move move tile to "up" direction
        # print("up")
        zero_position = Node.get_position(self)  # get position of '0' number
        if zero_position[0] == 0:  # Invalid move
            return False
        else:  # call the swap function with old and new tile position
            return Node.swap(self, zero_position[0], zero_position[1], zero_position[0] - 1, zero_position[1])

    def left(self):  # Similar to up function, here are trying to cehck if LEFT move is valid
        # print("left")
        zero_position = Node.get_position(self)
        if zero_position[1] == 0:
            return False
        else:
            return Node.swap(self, zero_position[0], zero_position[1], zero_position[0], zero_position[1] - 1)

    def right(self, dimension):  # Similar to up function, here are trying to cehck if RIGHT move is valid
        # passing dimension parameter because to accomdate even if 15 tile problem is give
        # print("right")
        zero_position = Node.get_position(self)
        if zero_position[1] == dimension - 1:
            return False
        else:
            return Node.swap(self, zero_position[0], zero_position[1], zero_position[0], zero_position[1] + 1)

    def down(self, dimension):  # Similar to up function, here are trying to cehck if DOWN move is valid
        # passing dimension parameter because to accommodate even if 15 tile problem is give
        # print("down")
        zero_position = Node.get_position(self)
        # print(zero_position)
        if zero_position[0] == dimension - 1:
            return False
        else:
            return Node.swap(self, zero_position[0], zero_position[1], zero_position[0] + 1, zero_position[1])

    def swap(self, x, y, x1, y1):
        value = self.state[x1, y1]  # getting new tile value
        new_state = copy.copy(self.state)  # shallow copy from old to new
        new_state[x, y] = value  # assign value to new position
        new_state[x1, y1] = 0  # old position is made 0
        return new_state, value

    def iddfs(self, goal, dimension):
        n = dimension * dimension  # Number of tile problem we are trying to solve
        states_created = 0  # Total number of states created during the implementation

        # Max Depth limit is set to 50
        for depth_limit in range(50):
            # print("in for loop")

            queue = [self]  # Nodes found but unvisited nodes
            # print("queue -> ", queue)
            depth_queue = [0]  # queue of node depth
            # print("depth_queue", depth_queue)
            path_cost_queue = [0]  # queue for path cost
            # print("path_cost_queue", path_cost_queue)
            visited = set([])  # record visited states
            # print("visited", visited)

            while queue:

                current_node = queue.pop(0)  # select and remove the first node in the queue
                states_created += 1
                current_depth = depth_queue.pop(0)  # select and remove the depth for current node
                # print ('Current  depth:',current_depth,'\n')
                current_path_cost = path_cost_queue.pop(
                    0)  # # select and remove the path cost for reaching current node
                visited.add(tuple(current_node.state.reshape(1, n)[
                                      0]))  # Marking and adding state which is visited, which is represented as a tuple

                ###############################################################################

                # when the goal state is found, Return the moves and number of states created
                if np.array_equal(current_node.state, goal_state):
                    # print("God Ganesha")
                    return len(current_node), str(states_created), True

                ###############################################################################

                else:
                    # Making sure the current depth is not more than MAX_DEPTH limt (31)
                    if current_depth < depth_limit:

                        if current_node.down(dimension):  # Can we move tile to down?, if true,  Call down function
                            new_state, down_value = current_node.down(dimension)  # Fetch new node and down value
                            if tuple(new_state.reshape(1, n)[0]) not in visited:  # check whther its already visited
                                # by converting into tuple which doesnt allow duplicates
                                # if new/unvisited , then Create child node
                                # creating a child node (instance crete below line)
                                current_node.move_down = Node(state=new_state, parent=current_node, action='down',
                                                              depth=current_depth + 1,
                                                              step_cost=down_value,
                                                              path_cost=current_path_cost + down_value)
                                queue.insert(0,
                                             current_node.move_down)  ##New child is added to front (STACK IMPLEMENTATION(LIFO))
                                depth_queue.insert(0, current_depth + 1)  # count moves
                                path_cost_queue.insert(0, current_path_cost + down_value)  # Cost

                        # see if right move is valid
                        # ALL THE STEPS ARE SIMILAR TO PREVIOUS IF CONDITION,instead of "down" we are cehcking for "right: move condition
                        if current_node.right(dimension):
                            new_state, right_value = current_node.right(dimension)
                            if tuple(new_state.reshape(1, n)[0]) not in visited:
                                current_node.move_right = Node(state=new_state, parent=current_node, action='right',
                                                               depth=current_depth + 1, step_cost=right_value,
                                                               path_cost=current_path_cost + right_value)
                                queue.insert(0, current_node.move_right)
                                depth_queue.insert(0, current_depth + 1)
                                path_cost_queue.insert(0, current_path_cost + right_value)

                        # see if moving up tile up is a valid move
                        # ALL THE STEPS ARE SIMILAR TO PREVIOUS IF CONDITION,instead of "down" we are cehcking for "up" move condition up_value
                        if current_node.up():
                            new_state, up_value = current_node.up()
                            # check if the resulting node is already visited
                            if tuple(new_state.reshape(1, n)[0]) not in visited:
                                # create a new child node
                                current_node.move_up = Node(state=new_state, parent=current_node, action='up',
                                                            depth=current_depth + 1,
                                                            step_cost=up_value,
                                                            path_cost=current_path_cost + up_value)
                                queue.insert(0, current_node.move_up)
                                depth_queue.insert(0, current_depth + 1)
                                path_cost_queue.insert(0, current_path_cost + up_value)

                        # see if moving left tile to the left is a valid move
                        # ALL THE STEPS ARE SIMILAR TO PREVIOUS IF CONDITION,instead of "down" we are cehcking for "left" move condition
                        if current_node.left():
                            new_state, left_value = current_node.left()
                            # check if the resulting node is already visited
                            if tuple(new_state.reshape(1, n)[0]) not in visited:
                                # create a new child node
                                current_node.move_left = Node(state=new_state, parent=current_node, action='left',
                                                              depth=current_depth + 1,
                                                              step_cost=left_value,
                                                              path_cost=current_path_cost + left_value)
                                queue.insert(0, current_node.move_left)
                                depth_queue.insert(0, current_depth + 1)

                                path_cost_queue.insert(0, current_path_cost + left_value)

        print("Max depth reached ABORT ABORT ABORT")  # If solution is not found within Max depth limit then Abort
        return len(current_node), str(states_created), False


#####################################################################################
#########   USE ME FOR TESTING SINGLE VALUES ########################################
#####################################################################################
# if __name__ == "__main__":
#     raw_input = [0, 0, [[0, 1, 8], [3, 6, 7], [5, 4, 2]]]  # PROVIDE INITIAL STATE HERE
#     goal_state = [2, 2, [[1, 2, 3], [4, 5, 6], [7, 8, 0]]]  # PROVIDE GOAL STATE HERE
#     flat_input = flatten(raw_input)
#     flat_goal = flatten(goal_state)
#
#     dimension = len(raw_input[2])
#
#     initial_state = np.array(flat_input).reshape(dimension, dimension)
#     goal_state = np.array(flat_goal).reshape(dimension, dimension)
#     # print("vinay --> \n", initial_state)
#     # print("sahana --> \n", goal_state)
#     root_node = Node(state=initial_state, parent=None, action=None, depth=0, step_cost=0,
#                      path_cost=0)  # Declaring instance of a class
#     t = time.process_time()  # start timer
#     start = time.time()
#     moves, yield_val, result = root_node.iddfs(goal_state, dimension)  # Call the IDDFS ALGO
#     # print(result)
#     if result:
#         elapsed = time.process_time() - t
#         end = time.time()  # End Timer
#         clockTime = end - start
#         print("Initial state : ", initial_state)
#         print("Goal state : ", goal_state)
#         print("Number of moves taken to complete : ", moves)
#         print('the number of yield of your move procedure made during the search for a solution : ', yield_val)
#         print("Time Taken to complete the search : {:8.2f}".format(elapsed, clockTime))
#         print('\n')
#         print('-------------------------------------------------------------------------------')
#     else:
#         elapsed = time.process_time() - t
#         end = time.time()  # End Timer
#         clockTime = end - start
#         print("Max depth reached")
#         print('the number of yield of your move procedure made during the search for a solution : ', yield_val)
#         print("Time Taken to complete the search : {:8.2f}".format(elapsed, clockTime))
#         print('\n')
#         print('-------------------------------------------------------------------------------')


if __name__ == "__main__":

    # Provide the Initial state as given in the assignment
    input_list = [[0, 0, [[0, 7, 1], [4, 3, 2], [8, 6, 5]]], [0, 2, [[5, 6, 0], [1, 3, 8], [4, 7, 2]]],
                  [2, 0, [[3, 5, 6], [1, 2, 7], [0, 8, 4]]],
                  [1, 1, [[7, 3, 5], [4, 0, 2], [8, 1, 6]]], [2, 0, [[6, 4, 8], [7, 1, 3], [0, 2, 5]]]]
    # Goal state as shown in the Assignment
    goal_list = [0, 2, [[3, 2, 0], [6, 1, 8], [4, 7, 5]]]

    for i in input_list:
        flat_input = flatten(i)  # Trimming the Tile zero poistion
        flat_goal = flatten(goal_list)  # Trimming the Tile zero poistion
        dimension = len(i[2])  # Dimension of the list eg: 3*3, 4*4
        try:
            initial_state = np.array(flat_input).reshape(dimension, dimension)  # convert to numpt arrray
            goal_state = np.array(flat_goal).reshape(dimension, dimension)  # convert to numpt arrray
            root_node = Node(state=initial_state, parent=None, action=None, depth=0, step_cost=0,
                             path_cost=0)  # Declaring instance of a class
            t = time.process_time()  # start timer
            start = time.time()
            moves, yield_val, result = root_node.iddfs(goal_state, dimension)  # Call the IDDFS ALGO

            if result:
                elapsed = time.process_time() - t
                end = time.time()  # End Timer
                clockTime = end - start
                print("Initial state : \n ", initial_state)
                print("Goal state : \n ", goal_state)
                print("Number of moves taken to complete : ", moves)
                print('the number of yield of your move procedure made during the search for a solution : ', yield_val)
                print("Time Taken to complete the search : {:8.2f}".format(elapsed, clockTime))
                print('\n')
                print('-------------------------------------------------------------------------------')
            else:
                elapsed = time.process_time() - t
                end = time.time()  # End Timer
                clockTime = end - start
                print("Max depth reached")
                print('the number of yield of your move procedure made during the search for a solution : ', yield_val)
                print("Time Taken to complete the search : {:8.2f}".format(elapsed, clockTime))
                print('\n')
                print('-------------------------------------------------------------------------------')


        except:
            print("check your input")
    print("FIRST SET OF QUESTION COMPLETE")

    input_list2 = [[0, 0, [[0, 1, 8], [3, 6, 7], [5, 4, 2]]], [2, 0, [[6, 4, 1], [7, 3, 2], [0, 5, 8]]],
                   [0, 0, [[0, 7, 1], [5, 4, 8], [6, 2, 3]]], [0, 2, [[5, 4, 0], [2, 3, 1], [8, 7, 6]]],
                   [2, 1, [[8, 6, 7], [2, 5, 4], [3, 0, 1]]] ]
    # Goal state as shown in the Assignment
    goal_list2 = [2, 2, [[1, 2, 3], [4, 5, 6], [7, 8, 0]]]

for i in input_list2:
    flat_input = flatten(i)  # Trimming the Tile zero poistion
    flat_goal = flatten(goal_list2)  # Trimming the Tile zero poistion
    dimension = len(i[2])  # Dimension of the list eg: 3*3, 4*4
    try:
        initial_state = np.array(flat_input).reshape(dimension, dimension)  # convert to numpt arrray
        goal_state = np.array(flat_goal).reshape(dimension, dimension)  # convert to numpt arrray
        root_node = Node(state=initial_state, parent=None, action=None, depth=0, step_cost=0,
                         path_cost=0)  # Declaring instance of a class
        t = time.process_time()  # start timer
        start = time.time()
        moves, yield_val, result = root_node.iddfs(goal_state, dimension)  # Call the IDDFS ALGO

        if result:
            elapsed = time.process_time() - t
            end = time.time()  # End Timer
            clockTime = end - start
            print("Initial state : \n ", initial_state)
            print("Goal state \n: ", goal_state)
            print("Number of moves taken to complete : ", moves)
            print('the number of yield of your move procedure made during the search for a solution : ', yield_val)
            print("Time Taken to complete the search : {:8.2f}".format(elapsed, clockTime))
            print('\n')
            print('-------------------------------------------------------------------------------')
        else:
            elapsed = time.process_time() - t
            end = time.time()  # End Timer
            clockTime = end - start
            print("Max depth reached")
            print('the number of yield of your move procedure made during the search for a solution : ', yield_val)
            print("Time Taken to complete the search : {:8.2f}".format(elapsed, clockTime))
            print('\n')
            print('-------------------------------------------------------------------------------')

    except:
        print("check your input")
