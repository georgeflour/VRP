import math

# Κλάση που περιγράφει τις ιδιότητες που θα έχει κάθε κόμβος και τη λειτουργικότητα του μοντέλου
class Model:
    class Node:
        def __init__(self, idd, xx, yy, dem=0, st=0):
            self.x = xx
            self.y = yy
            self.ID = idd
            self.isRouted = False
            self.demand = dem

    # Αρχικοποιεί το μοντέλο και φορτώνει τα δεδομένα από το αρχείο
    def __init__(self, file_name):
        self.all_nodes, self.capacity, self.empty_vehicle_weight, self.depot = self.load_model(file_name)
        self.total_nodes = len(self.all_nodes)
        self.distance_matrix = [[0 for _ in range(self.total_nodes)] for _ in range(self.total_nodes)]
        self.total_demand = sum(node.demand for node in self.all_nodes)

    # εξάγει τα στοιχεία από το αρχείο Instance.txt σε διάφορες μεταβλητές
    @staticmethod
    def load_model(file_name):
        all_nodes = []
        all_lines = list(open(file_name, "r"))

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
        all_nodes.append(depot)
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
            all_nodes.append(customer)

        return all_nodes, capacity, empty_vehicle_weight, depot

    # Μέθοδος που υπολογίζει την απόσταση ανάμεσα σε δύο κόμβους
    @staticmethod
    def distance(from_node, to_node):
        dx = from_node.x - to_node.x
        dy = from_node.y - to_node.y
        dist = math.sqrt(dx ** 2 + dy ** 2)
        return dist

    # Υπολογίζει τον πίνακα αποστάσεων
    def calculate_distance_matrix(self):
        for i in range(self.total_nodes):
            for j in range(self.total_nodes):
                if i != j:
                    dist_From_I_to_J = self.distance(self.all_nodes[i], self.all_nodes[j])
                    self.distance_matrix[i][j] = dist_From_I_to_J 
        return self.distance_matrix


class Node:
    def __init__(self, idd, xx, yy, dem):
        self.x = xx
        self.y = yy
        self.ID = idd
        self.demand = dem
        self.isRouted = False
        