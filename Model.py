import math

class Model:
    class Node:
        def __init__(self, idd, xx, yy, dem=0, st=0):
            self.x = xx
            self.y = yy
            self.ID = idd
            self.isRouted = False
            self.demand = dem

    def initialize_model(self):
        self.BuildModel()
        self.calculate_matrix()

    def BuildModel(self):
        self.allNodes, self.capacity, self.empty_vehicle_weight, self.depot = self.load_model("Instance.txt")
        self.total_nodes = len(self.allNodes)
        self.matrix = [[0 for _ in range(self.total_nodes)] for _ in range(self.total_nodes)]
        self.total_demand = sum(node.demand for node in self.allNodes)
        self.customers = [node for node in self.allNodes if node.ID != self.depot.ID]

    @staticmethod
    def load_model(file):
        allNodes = []
        all_lines = list(open(file, "r"))

        separator = ','

        line_counter = 0

        ln = all_lines[line_counter]
        no_spaces = ln.split(sep=separator)
        capacity = int(no_spaces[1])

        line_counter += 1
        ln = all_lines[line_counter]
        no_spaces = ln.split(sep=separator)
        empty_vehicle_weight = int(no_spaces[1])

        line_counter += 1
        ln = all_lines[line_counter]
        no_spaces = ln.split(sep=separator)
        tot_customers = int(no_spaces[1])

        line_counter += 3
        ln = all_lines[line_counter]

        no_spaces = ln.split(sep=separator)
        x = float(no_spaces[1])
        y = float(no_spaces[2])
        depot = Model.Node(0, x, y)
        allNodes.append(depot)
        total_demand = 0
        for i in range(tot_customers):
            line_counter += 1
            ln = all_lines[line_counter]
            no_spaces = ln.split(sep=separator)
            idd = int(no_spaces[0])
            x = float(no_spaces[1])
            y = float(no_spaces[2])
            demand = float(no_spaces[3])
            customer = Model.Node(idd, x, y, demand)
            total_demand += demand
            allNodes.append(customer)

        return allNodes, capacity, empty_vehicle_weight, depot

    @staticmethod
    def distance(from_node, to_node):
        dx = from_node.x - to_node.x
        dy = from_node.y - to_node.y
        dist = math.sqrt(dx ** 2 + dy ** 2)
        return dist

    def calculate_matrix(self):
        for i in range(self.total_nodes):
            for j in range(self.total_nodes):
                if i != j:
                    dist_From_I_to_J = self.distance(self.allNodes[i], self.allNodes[j])
                    self.matrix[i][j] = dist_From_I_to_J 
        return self.matrix


class Node:
    def __init__(self, idd, xx, yy, dem):
        self.x = xx
        self.y = yy
        self.ID = idd
        self.demand = dem
        self.isRouted = False

class Route:
    def __init__(self, dp, cap):
        self.sequenceOfNodes = []
        self.sequenceOfNodes.append(dp)
        self.cost = 0
        self.capacity = cap
        self.load = 0