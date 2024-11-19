import random
import math

class Node:
    def __init__(self, idd, xx, yy, dem=0, st=0):
        self.x = xx
        self.y = yy
        self.ID = idd
        self.isRouted = False
        self.demand = dem

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
    empty_vehicle_weight = 8  # You can keep this as 8 if it's a constant value for now

    line_counter += 1
    ln = all_lines[line_counter]
    no_spaces = ln.split(sep=separator)
    tot_customers = int(no_spaces[1])

    line_counter += 3
    ln = all_lines[line_counter]

    no_spaces = ln.split(sep=separator)
    x = float(no_spaces[1])
    y = float(no_spaces[2])
    depot = Node(0, x, y)
    all_nodes.append(depot)

    for i in range(tot_customers):
        line_counter += 1
        ln = all_lines[line_counter]
        no_spaces = ln.split(sep=separator)
        idd = int(no_spaces[0])
        x = float(no_spaces[1])
        y = float(no_spaces[2])
        demand = float(no_spaces[3])
        customer = Node(idd, x, y, demand)
        all_nodes.append(customer)

    return all_nodes, capacity, empty_vehicle_weight

def distance(from_node, to_node):
    dx = from_node.x - to_node.x
    dy = from_node.y - to_node.y
    dist = math.sqrt(dx ** 2 + dy ** 2)
    return dist

def calculate_route_details(nodes_sequence, empty_vehicle_weight):
    total_load = sum(node.demand for node in nodes_sequence)
    tn_km = 0
    current_load = empty_vehicle_weight + total_load  # Initial load of the vehicle

    for i in range(len(nodes_sequence) - 1):
        from_node = nodes_sequence[i]
        to_node = nodes_sequence[i + 1]

        dist = distance(from_node, to_node)
        tn_km += dist * current_load

        current_load -= to_node.demand

    return tn_km, total_load

def save_solution_to_file(routes, total_cost, file_name):
    with open(file_name, "w") as file:
        file.write(f"Cost:\n{total_cost}\n")
        file.write(f"Routes:\n{len(routes)}\n")
        for route in routes:
            route_str = ",".join(map(str, route))
            file.write(f"{route_str}\n")

def solve_vrp(all_nodes, capacity, empty_vehicle_weight):
    nodes = all_nodes
    depot = nodes[0]
    customers = nodes[1:]
    unvisited = set(customers)
    routes = []
    total_cost = 0

    # Προετοιμασία πίνακα αποστάσεων για ταχύτερη αναζήτηση
    distance_matrix = [[0] * len(nodes) for _ in range(len(nodes))]
    for i in range(len(nodes)):
        for j in range(len(nodes)):
            if i != j:
                distance_matrix[i][j] = distance(nodes[i], nodes[j])

    while unvisited:
        current_route = [depot.ID]  # Start the route with depot (node 0)
        current_capacity = capacity
        current_node = depot
        route_cost = 0

        # Προσπάθεια γεμίσματος του φορτηγού όσο το δυνατόν περισσότερο
        while unvisited and current_capacity > 0:
            # Βρίσκουμε τον πελάτη με την μικρότερη απόσταση και ζήτηση
            best_next_node = None
            min_distance = float('inf')
            min_demand = float('inf')
            
            for node in unvisited:
                if node.demand <= current_capacity:
                    dist = distance_matrix[current_node.ID][node.ID]
                    if dist < min_distance or (dist == min_distance and node.demand < min_demand):
                        best_next_node = node
                        min_distance = dist
                        min_demand = node.demand

            if best_next_node:
                current_route.append(best_next_node.ID)
                current_capacity -= best_next_node.demand
                unvisited.remove(best_next_node)
                route_cost += min_distance
                current_node = best_next_node
            else:
                break

        # Υπολογισμός των λεπτομερειών της διαδρομής
        rt_tn_km, rt_load = calculate_route_details([depot] + [all_nodes[i] for i in current_route[1:]], empty_vehicle_weight)
        
        routes.append(current_route)
        total_cost += rt_tn_km

    return routes, total_cost
# Main code to generate the solution and save it
all_nodes, capacity, empty_vehicle_weight = load_model('Instance.txt')
routes, total_cost = solve_vrp(all_nodes, capacity, empty_vehicle_weight)
save_solution_to_file(routes, total_cost, "Solution.txt")
print("The solution has been saved to Solution.txt")

