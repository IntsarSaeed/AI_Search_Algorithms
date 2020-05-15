"""
Submitted by: Intsar Saeed
Dated: 27.04.2020
"""

import os
import sys
import math
import time
import psutil
import queue as Q
import numpy as np


class PuzzleState(object):

    """This class defines the state of the puzzle"""
    def __init__(self, config, n, parent=None, action="Initial", cost=0):

        if n*n != len(config) or n < 2:
            raise Exception("the length of config is not correct!")

        self.n = n
        self.cost = cost
        self.parent = parent
        self.action = action
        self.dimension = n
        self.config = config
        self.children = []
        self.priority = 1

        for i, item in enumerate(self.config):
            if item == 0:
                self.blank_row = i // self.n
                self.blank_col = i % self.n
                break

    def __lt__(self, other):
        if self.priority < other.priority:
            return True

    def display(self):

        for i in range(self.n):
            line = []
            offset = i * self.n
            for j in range(self.n):
                line.append(self.config[offset + j])
            print(line)

    def move_left(self):

        if self.blank_col == 0:
            return None

        else:

            blank_index = self.blank_row * self.n + self.blank_col
            target = blank_index - 1
            new_config = list(self.config)
            new_config[blank_index], new_config[target] = new_config[target], new_config[blank_index]
            return PuzzleState(tuple(new_config), self.n, parent=self, action="Left", cost=self.cost + 1)

    def move_right(self):

        if self.blank_col == self.n - 1:
            return None

        else:

            blank_index = self.blank_row * self.n + self.blank_col
            target = blank_index + 1
            new_config = list(self.config)
            new_config[blank_index], new_config[target] = new_config[target], new_config[blank_index]
            return PuzzleState(tuple(new_config), self.n, parent=self, action="Right", cost=self.cost + 1)

    def move_up(self):

        if self.blank_row == 0:
            return None

        else:
            blank_index = self.blank_row * self.n + self.blank_col
            target = blank_index - self.n
            new_config = list(self.config)
            new_config[blank_index], new_config[target] = new_config[target], new_config[blank_index]
            return PuzzleState(tuple(new_config), self.n, parent=self, action="Up", cost=self.cost + 1)

    def move_down(self):

        if self.blank_row == self.n - 1:
            return None

        else:
            blank_index = self.blank_row * self.n + self.blank_col
            target = blank_index + self.n
            new_config = list(self.config)
            new_config[blank_index], new_config[target] = new_config[target], new_config[blank_index]
            return PuzzleState(tuple(new_config), self.n, parent=self, action="Down", cost=self.cost + 1)

    def expand(self):

        """expands the node"""
        # add child nodes in order of UDLR

        if len(self.children) == 0:
            up_child = self.move_up()
            if up_child is not None:
                self.children.append(up_child)

            down_child = self.move_down()
            if down_child is not None:
                self.children.append(down_child)

            left_child = self.move_left()
            if left_child is not None:
                self.children.append(left_child)

            right_child = self.move_right()
            if right_child is not None:
                self.children.append(right_child)

        return self.children


def write_output(path_to_goal, cost_of_path, nodes_expanded, search_depth, max_search_depth,
                 running_time, max_ram_usage):
    """Generates an output file with the defined parameters"""

    f = open(os.path.join(os.getcwd(), 'output.txt'), 'w')
    write_str = "path_to_goal: %s \ncost_of_path: %i \nnodes_expanded: %s\nsearch_depth:%i " \
                "\nmax_search_depth: %s\nrunning_time: %s\nmax_ram_usage:%s " \
                % (path_to_goal, cost_of_path, nodes_expanded, search_depth, max_search_depth,
                   running_time, max_ram_usage)
    f.write(write_str)
    f.close()
    return


def bfs_search(initial_state):

    """BFS search"""
    start_time = time.time()
    nodes_expanded = 0
    max_depth = 0
    frontier = Q.Queue()
    explored_nodes = set()
    frontier_configs = set()
    moves = []

    current_node = initial_state
    frontier.put(current_node)
    frontier_configs.add(current_node.config)

    while frontier:
        current_node = frontier.get()
        explored_nodes.add(current_node.config)

        if current_node.cost > max_depth:
            max_depth = current_node.cost

        if current_node.config in frontier_configs:
            frontier_configs.remove(current_node.config)

        if test_goal(current_node.config):

            search_depth = cost_of_path = current_node.cost
            while current_node.action != "Initial":
                moves.append(current_node.action)
                current_node = current_node.parent
            moves.reverse()  # reverse the order
            memory_use = psutil.Process().memory_info().rss
            write_output(str(moves), cost_of_path, str(nodes_expanded), search_depth,
                         str(max_depth + 1), str((time.time() - start_time)),  str(memory_use))
            print("Output File was generated")
            return True

        child_nodes = current_node.expand()  # Expand the current Node
        nodes_expanded += 1
        for node in child_nodes:
            if node.config not in explored_nodes:
                if node.config not in frontier_configs:
                    frontier_configs.add(node.config)
                    frontier.put(node)

    return False


def dfs_search(initial_state):

    """DFS search"""
    goal = (0, 1, 2, 3, 4, 5, 6, 7, 8)
    """BFS search"""
    start_time = time.time()
    nodes_expanded = 0
    max_depth = 0
    frontier = []
    explored_nodes = set()
    frontier_configs = set()
    moves = []

    current_node = initial_state
    frontier.append(current_node)
    frontier_configs.add(current_node.config)

    while frontier:
        current_node = frontier.pop()
        explored_nodes.add(current_node.config)

        if current_node.cost > max_depth:
            max_depth = current_node.cost

        if current_node.config in frontier_configs:
            frontier_configs.remove(current_node.config)

        if test_goal(current_node.config):
            search_depth = cost_of_path = current_node.cost
            while current_node.action != "Initial":
                moves.append(current_node.action)
                current_node = current_node.parent
            moves.reverse()  # reverse the order
            memory_use = psutil.Process().memory_info().rss
            write_output(str(moves), cost_of_path, str(nodes_expanded), search_depth,
                         str(max_depth), str((time.time() - start_time)),  str(memory_use))
            print("Output File was generated")
            return True

        child_nodes = current_node.expand()  # Expand the current Node
        child_nodes.reverse()
        nodes_expanded += 1
        for node in child_nodes:
            if node.config not in explored_nodes:
                if node.config not in frontier_configs:
                    frontier_configs.add(node.config)
                    frontier.append(node)

    return False


def a_star_search(initial_state):

    """A * search"""
    pid = os.getpid()
    py = psutil.Process(pid)
    start_time = time.time()
    nodes_expanded = 0
    max_depth = 0

    frontier = []
    explored_nodes = set()
    frontier_configs = []
    moves = []
    current_node = initial_state
    frontier.append([1, current_node])
    frontier_configs.append([1, current_node.config])

    while frontier:
        frontier.sort(key=lambda x: x[0])
        frontier_configs.sort(key=lambda x: x[0])

        current_node = frontier.pop()
        fr = frontier_configs.pop()
        explored_nodes.add(fr[1])

        if current_node[1].cost > max_depth:
            max_depth = current_node[1].cost

        if test_goal(current_node[1].config):
            search_depth = cost_of_path = current_node[1].cost
            while current_node[1].action != "Initial":
                moves.append(current_node[1].action)
                current_node[1] = current_node[1].parent
            moves.reverse()  # reverse the order
            memory_use = psutil.Process().memory_info().rss
            write_output(str(moves), cost_of_path, str(nodes_expanded), search_depth,
                         str(max_depth), str((time.time() - start_time)), str(memory_use))
            print("Output File was generated")
            return True

        child_nodes = current_node[1].expand()  # Expand the current Node
        nodes_expanded += 1

        list_of_configs_in_frontier = [x[1] for x in frontier_configs]
        for node in child_nodes:
            if node.config not in explored_nodes:
                if node.config not in list_of_configs_in_frontier:
                    cost = calculate_total_cost(node)
                    priority = 1/cost
                    frontier.append([priority, node])
                    frontier_configs.append([priority, node.config])

            elif node.config in list_of_configs_in_frontier:
                cost = calculate_total_cost(node)
                priority = 1 / cost
                for i in range(0, (len(list_of_configs_in_frontier)-1)):
                    if len(list_of_configs_in_frontier) > 0:
                        if list_of_configs_in_frontier[i].config == node.config:
                            if frontier[i][0] > priority:
                                frontier[i][0] = priority
    return False


def calculate_total_cost(state) -> float:

    """calculate the total estimated cost of a state"""
    return state.cost + calculate_manhattan_dist(state.config)


def calculate_manhattan_dist(value) -> int:

    """calculate the manhattan distance of a tile"""
    goal = np.reshape([0, 1, 2, 3, 4, 5, 6, 7, 8], (3, 3))
    state = np.reshape(value, (3, 3))
    man_dist = 0
    for i in range(0, goal.size):
        i_at_goal = np.where(goal == i)
        i_at_current = np.where(state == i)
        diff = abs(i_at_goal[0] - i_at_current[0]) + abs(i_at_goal[1] - i_at_current[1])
        man_dist += diff
    return int(man_dist)


def test_goal(puzzle_state) -> bool:

    """checks if the input state is the goal-state"""
    goal = (0, 1, 2, 3, 4, 5, 6, 7, 8)
    if puzzle_state == goal:
        return True
    else:
        return False


# Main Function that reads in Input and Runs corresponding Algorithm
def main():

    sm = sys.argv[1].lower()  # Input The name of the Algorithm
    begin_state = sys.argv[2].split(",")  # Input the initial state
    begin_state = tuple(map(int, begin_state))
    size = int(math.sqrt(len(begin_state)))
    hard_state = PuzzleState(begin_state, size)

    if sm == "bfs":
        bfs_search(hard_state)
    elif sm == "dfs":
        dfs_search(hard_state)
    elif sm == "ast":
        a_star_search(hard_state)
    else:
        print("Enter valid command arguments !")


if __name__ == '__main__':
    main()
