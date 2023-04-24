import algorithm
import sys
from maze import Maze


class Node:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.cost = 0
        self.parent = None
        self.up = None
        self.down = None
        self.right = None
        self.left = None
        self.heuristic = 0

    def checking_equation(self, x, y):
        return x == self.x and y == self.y

    def __str__(self):
        return "[" + str(self.x) + ", " + str(self.y) + "]"


class Graph:

    nodes = []
    maze = None

    def __init__(self):
        self.maze = Maze()
        self.root = self.create_node(self.maze.start[0], self.maze.start[1])
        self.maximum_depth = self.find_maximum_depth() - 1
        self.create_heuristic()
        self.root.cost = 0

    def create_node(self, x, y):
        node = Node()
        node.x = x
        node.y = y
        self.nodes.append(node)
        node.cost = 1

        if self.maze.adjacent_nodes(node.x, node.y, "up"):
            node.up = self.node_exist(node.x - 1, node.y)
            if node.up is None:
                node.up = self.create_node(node.x - 1, node.y)
                node.up.parent = node
        if self.maze.adjacent_nodes(node.x, node.y, "right"):
            node.right = self.node_exist(node.x, node.y + 1)
            if node.right is None:
                node.right = self.create_node(node.x, node.y + 1)
                node.right.parent = node
        if self.maze.adjacent_nodes(node.x, node.y, "down"):
            node.down = self.node_exist(node.x + 1, node.y)
            if node.down is None:
                node.down = self.create_node(node.x + 1, node.y)
                node.down.parent = node
        if self.maze.adjacent_nodes(node.x, node.y, "left"):
            node.left = self.node_exist(node.x, node.y - 1)
            if node.left is None:
                node.left = self.create_node(node.x, node.y - 1)
                node.left.parent = node

        return node

    def node_exist(self, x, y):
        for node in self.nodes:
            if node.checking_equation(x, y):
                return node
        return None

    def find_maximum_depth(self):
        maximum_depth = 0
        for node in self.nodes:
            current_node = node
            current_depth = 0
            while current_node is not None:
                current_node = current_node.parent
                current_depth += 1
            maximum_depth = max(maximum_depth, current_depth)
        return maximum_depth

    def node_cost(self, x, y):
        for node in self.nodes:
            if node.checking_equation(x, y):
                return node.cost
        return 0

    def clear_parents(self):
        for node in self.nodes:
            node.parent = None

    def create_heuristic(self):
        for node in self.nodes:
            total_cost = sys.maxsize
            for goal in self.maze.goals:
                cost = 0
                vertical_distance = goal[1] - node.y
                horizontal_distance = goal[0] - node.x
                x = 0
                y = 0
                while vertical_distance > 0:
                    y += 1
                    cost += self.node_cost(node.x, node.y + y)
                    vertical_distance -= 1
                while horizontal_distance > 0:
                    x += 1
                    cost += self.node_cost(node.x + x, node.y + y)
                    horizontal_distance -= 1
                while vertical_distance < 0:
                    y -= 1
                    cost += self.node_cost(node.x + x, node.y + y)
                    vertical_distance += 1
                while horizontal_distance < 0:
                    x -= 1
                    cost += self.node_cost(node.x + x, node.y + y)
                    horizontal_distance += 1
                total_cost = min(total_cost, cost)
            node.heuristic = total_cost


if __name__ == "__main__":
    graph = Graph()
    algorithm.graph = graph
    algorithm.DFS()
    algorithm.BFS()
    algorithm.aStar()
