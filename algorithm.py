from collections import OrderedDict


graph = None
fringe = []
visited = OrderedDict()

def DFS_BFS(algorithm):

    pop_index = 0
    goal_state = None
    solution_cost = 0
    solution = []
    expanded_nodes = []
    iteration = -1

    # DFS_BFS
    while goal_state is None and iteration <= graph.maximum_depth:

        iteration += 1
        fringe.clear()
        visited.clear()
        fringe.append(graph.root)

        while len(fringe) > 0:
            if "DFS" in algorithm :
                pop_index = len(fringe) - 1
            current_node = fringe.pop(pop_index)
            visited[current_node] = None
            if is_goal(current_node):
                goal_state = current_node
                break
            if "iterative" in algorithm:
                parent = current_node
                for i in range(iteration):
                    parent = parent if parent is None else parent.parent
                if parent is None:
                    add_to_fringe(current_node, "DFS")
            else:
                add_to_fringe(current_node, algorithm)

        for node in visited:
            expanded_nodes.append(node)
        if "iterative" not in algorithm:
            break
    if goal_state is None:
        print("No goal state found.")
        return

    current = goal_state
    while current is not None:
        solution_cost += current.cost
        solution.insert(0, current)
        current = current.parent
    print_results(algorithm, solution_cost, solution, expanded_nodes)

def DFS():
    graph.clear_parents()
    DFS_BFS("DFS: ")

def BFS():
    graph.clear_parents()
    DFS_BFS("BFS:")

def aStar():
    graph.clear_parents()
    heuristic("A Star Search(A*):", return_cost_and_heuristic)


def heuristic(algorithm, sorting_with):

    goal_state = None
    solution_cost = 0
    solution = []
    fringe.clear()
    visited.clear()
    fringe.append(graph.root)

    while len(fringe) > 0:
        sort_fringe(sorting_with)
        current_node = fringe.pop(0)
        visited[current_node] = None
        if is_goal(current_node):
            goal_state = current_node
            break
        add_to_fringe(current_node, "BFS")

    if goal_state is not None:
        current = goal_state
        while current is not None:
            solution_cost += current.cost
            solution.insert(0, current)
            current = current.parent
        print_results(algorithm, solution_cost, solution, visited)
    else:
        print("No goal state found.")

def set_parent(parent_node, child_node, algorithm):
    if "DFS" in algorithm or child_node.parent is None:
        child_node.parent = parent_node
    return child_node


def in_visited(node):
    if node in visited:
        return True
    return False


def is_goal(node):
    for goal in graph.maze.goals:
        if goal[0] == node.x and goal[1] == node.y:
            return True
    return False

def return_cost(node):
    return node.cost


def return_heuristic(node):
    return node.heuristic


def return_cost_and_heuristic(node):
    return node.heuristic + node.cost


def sort_fringe(sort_by):
    fringe.sort(key=sort_by)

def add_to_fringe(current_node, algorithm):
    nodes_to_add = []

    if current_node.up is not None and not in_visited(current_node.up):
        nodes_to_add.append(set_parent(current_node, current_node.up, algorithm))

    if current_node.right is not None and not in_visited(current_node.right):
        nodes_to_add.append(set_parent(current_node, current_node.right, algorithm))

    if current_node.down is not None and not in_visited(current_node.down):
        nodes_to_add.append(set_parent(current_node, current_node.down, algorithm))

    if current_node.left is not None and not in_visited(current_node.left):
        nodes_to_add.append(set_parent(current_node, current_node.left, algorithm))

    if "DFS" in algorithm:
        nodes_to_add.reverse()
    for node in nodes_to_add:
        fringe.append(node)

def print_results(algorithm, cost, solution, opened_nodes):
    print(algorithm)
    print("Cost:", cost)
    print(" solution  (********" + str(len(solution)) + "****** nodes):", end=" ")
    for node in solution:
        print(node, end=" ")
    print("\nopened (********" + str(len(opened_nodes)) + " ********nodes):", end=" ")
    if "iterative" in algorithm:
        print()
        for i in range(len(opened_nodes) - 1):
            if type(opened_nodes[i + 1]) == str:
                print(opened_nodes[i])
            else:
                print(opened_nodes[i], end=" ")
    else:
        for node in opened_nodes:
            print(node, end=" ")
    print("\n")



