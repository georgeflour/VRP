from Model import *
import random
import math


# Αρχικοποίηση του μοντέλου και των δεδομένων

model = Model('/Users/giorgosphlourakes/Desktop/MEDEBE/MEDEBE/MySolution/Instance.txt')
all_nodes, capacity, empty_vehicle_weight, depot = Model.load_model('/Users/giorgosphlourakes/Desktop/MEDEBE/MEDEBE/MySolution/Instance.txt')
total_nodes = len(all_nodes)
total_demand = sum(node.demand for node in all_nodes if node.ID != depot.ID)
distance_matrix = model.calculate_distance_matrix()

# Κλάση που ορίζει την αλληλουχία των κόμβων και το κόστος      
class Solution:
    def __init__(self):
        self.routes = []  # Λίστα που αποθηκεύει τις διαδρομές για κάθε φορτηγό
        self.total_cost = 0  # Συνολικό κόστος της λύσης
        



# Μέθοδος που βρίσκει το j με την ελάχιστη απόσταση για ένα συγκεκριμένο i
def find_min_distance_for_i(distance_matrix, i, visited_nodes):
    min_distance = float('inf')
    min_j = -1
    for j in range(len(distance_matrix[i])):
        if i != j and j not in visited_nodes and distance_matrix[i][j] < min_distance:
            min_distance = distance_matrix[i][j]
            min_j = j
    return min_j, min_distance



def find_next_node_with_min_cost(current_route, all_nodes, distance_matrix, empty_vehicle_weight, visited_nodes, track_space):
    min_cost = float('inf')
    best_node = None

    for candidate_node in all_nodes:
        if candidate_node.ID in visited_nodes or candidate_node in current_route:
            continue  # Skip visited nodes or nodes already in the route

        if candidate_node.demand > track_space:
            continue  # Skip nodes exceeding the remaining truck capacity

        # Trial sequence to calculate cost
        trial_sequence = current_route + [candidate_node]
        trial_cost, _ = calculate_route_details(trial_sequence, empty_vehicle_weight)

        if trial_cost < min_cost:
            min_cost = trial_cost
            best_node = candidate_node

    return best_node, min_cost



# Μέθοδος για υπολογισμό λεπτομερειών διαδρομής
def calculate_route_details(nodes_sequence, empty_vehicle_weight):
    total_load = sum(node.demand for node in nodes_sequence)
    tn_km = 0
    current_load = empty_vehicle_weight + total_load  # Αρχικό φορτίο

    for i in range(len(nodes_sequence) - 1):
        from_node = nodes_sequence[i]
        to_node = nodes_sequence[i + 1]
        distance = distance_matrix[from_node.ID][to_node.ID]
        tn_km += distance * current_load
        current_load -= to_node.demand  # Αφαίρεση της ζήτησης του επόμενου κόμβου από το τρέχον φορτίο

    return tn_km, total_load


# Updated nearest_neighbor to include FindBestSwapMove
def nearest_neighbor(distance_matrix, all_nodes, total_nodes, capacity, empty_vehicle_weight):
    sol = Solution()
    visited_nodes = {depot.ID}  # Start with the depot
    remaining_demand = sum(node.demand for node in all_nodes if node.ID != depot.ID)

    # Create initial routes using nearest neighbor
    while remaining_demand > 0:
        route = Route(depot)  # New route for each truck
        route.sequenceOfNodes.append(depot)  # Start route at the depot
        track_space = capacity

        while track_space > 0 and len(visited_nodes) < total_nodes:
            current_node = route.sequenceOfNodes[-1]  # Last node in the current route
            min_j, _ = find_min_distance_for_i(distance_matrix, current_node.ID, visited_nodes)
            if min_j == -1:  # No more nodes to visit
                break

            candidate_node = all_nodes[min_j]
            if candidate_node.demand <= track_space and not candidate_node.isRouted:
                candidate_node.isRouted = True
                route.sequenceOfNodes.append(candidate_node)
                track_space -= candidate_node.demand
                remaining_demand -= candidate_node.demand
                visited_nodes.add(candidate_node.ID)

        # Calculate route cost and add it to the solution
        rt_tn_km, rt_load = calculate_route_details(route.sequenceOfNodes, empty_vehicle_weight)
        route.cost = rt_tn_km
        route.load = rt_load
        sol.routes.append(route)
        sol.total_cost += rt_tn_km

    return sol



# Κλήση της μεθόδου TSP για πολλαπλά φορτηγά
sol = nearest_neighbor(distance_matrix, all_nodes, total_nodes, capacity, empty_vehicle_weight)

# Μέθοδος για αποθήκευση της λύσης σε αρχείο
def save_solution_to_file(solution, filename='solution6.txt'):
    with open(filename, 'w') as file:
        file.write(f"Cost:\n{solution.total_cost}\n")
        file.write("Routes:\n")
        file.write(f"{len(solution.routes)}\n")
        for route in solution.routes:
            file.write(",".join(map(str, route)) + "\n")

# Χρήση της λύσης από τον αρχικό κώδικά σου

# Εκτέλεση Simulated Annealing πάνω στην αρχική λύση


# Αποθήκευση της λύσης σε αρχείο
save_solution_to_file(sol)


# Εμφάνιση των διαδρομών και του συνολικού κόστους
#for idx, route in enumerate(sol.routes):
#    print(f"Διαδρομή φορτηγού {idx + 1}: {route}")
#print("Συνολικό κόστος:", sol.total_cost)


    
    
    
