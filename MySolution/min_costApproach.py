from Model import Model
import random
import math


# Αρχικοποίηση του μοντέλου και των δεδομένων

model = Model('/MEDEBE/MySolution/Instance.txt')
all_nodes, capacity, empty_vehicle_weight, depot = Model.load_model('/MEDEBE/MySolution/Instance.txt')
total_nodes = len(all_nodes)
total_demand = sum(node.demand for node in all_nodes if node.ID != depot.ID)
distance_matrix = model.calculate_distance_matrix()

# Κλάση που ορίζει την αλληλουχία των κόμβων και το κόστος      
class Solution:
    def __init__(self):
        self.routes = []  # Λίστα που αποθηκεύει τις διαδρομές για κάθε φορτηγό
        self.total_cost = 0  # Συνολικό κόστος της λύσης
        
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


def least_cost(distance_matrix, all_nodes, total_nodes, capacity, empty_vehicle_weight):
    sol = Solution()
    visited_nodes = {depot.ID}  # Start with the depot as visited
    remaining_demand = sum(node.demand for node in all_nodes if node.ID != depot.ID)

    while remaining_demand > 0:
        # Start a new route for a truck
        track_space = capacity
        current_route = [depot]  # Each route starts at the depot

        while track_space > 0 and len(visited_nodes) < total_nodes:
            next_node, min_cost = find_next_node_with_min_cost(
                current_route,
                all_nodes,
                distance_matrix,
                empty_vehicle_weight,
                visited_nodes,
                track_space
            )

            if next_node is None:  # No more feasible nodes to visit
                break

            # Add the node to the current route
            next_node.isRouted = True
            current_route.append(next_node)
            track_space -= next_node.demand
            remaining_demand -= next_node.demand
            visited_nodes.add(next_node.ID)

        # Add the route to the solution and calculate its cost
        rt_tn_km, rt_load = calculate_route_details(current_route, empty_vehicle_weight)
        sol.routes.append([node.ID for node in current_route])  # Save the route as a list of IDs
        sol.total_cost += rt_tn_km

    return sol



# Κλήση της μεθόδου TSP για πολλαπλά φορτηγά
sol = least_cost(distance_matrix, all_nodes, total_nodes, capacity, empty_vehicle_weight)

# Εκτύπωση της βέλτιστης λύσης
#print("Βέλτιστη λύση μετά το SA:")
#for idx, route in enumerate(best_routes):
#    print(f"Route {idx + 1}: {route}")
#print("Συνολικό κόστος:", best_cost)

# Μέθοδος για αποθήκευση της λύσης σε αρχείο
def save_solution_to_file(solution, filename='solution5.txt'):
    with open(filename, 'w') as file:
        file.write(f"Cost:\n{solution.total_cost}\n")
        file.write("Routes:\n")
        file.write(f"{len(solution.routes)}\n")
        for route in solution.routes:
            file.write(",".join(map(str, route)) + "\n")


# Αποθήκευση της λύσης σε αρχείο
save_solution_to_file(sol)