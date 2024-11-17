from Model import Model

# Αρχικοποίηση του μοντέλου και των δεδομένων
model = Model('MEDEBE/MySolution/Instance.txt')
all_nodes, capacity, empty_vehicle_weight, depot = Model.load_model('MEDEBE/MySolution/Instance.txt')
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

# Μέθοδος TSP για πολλαπλά φορτηγά με ανοιχτές διαδρομές και υπολογισμό κόστους
def tsp_method(distance_matrix, all_nodes, total_nodes, capacity):
    sol = Solution()
    visited_nodes = {depot.ID}  # Ξεκινάμε με το depot ως επισκέψιμο
    remaining_demand = sum(node.demand for node in all_nodes if node.ID != depot.ID)

    while remaining_demand > 0:
        # Ξεκινάμε μια νέα διαδρομή για ένα φορτηγό
        track_space = capacity
        current_route = [depot]  # Κάθε διαδρομή ξεκινά από το depot
        current_node = depot.ID

        while track_space > 0 and len(visited_nodes) < total_nodes:
            min_j, min_distance = find_min_distance_for_i(distance_matrix, current_node, visited_nodes)
            if min_j == -1:  # Δεν υπάρχουν άλλοι κόμβοι να επισκεφθεί
                break

            if all_nodes[min_j].demand <= track_space and not all_nodes[min_j].isRouted:
                # Προσθέτουμε τον κόμβο στην ακολουθία της διαδρομής του φορτηγού
                all_nodes[min_j].isRouted = True
                current_route.append(all_nodes[min_j])
                track_space -= all_nodes[min_j].demand
                remaining_demand -= all_nodes[min_j].demand
                visited_nodes.add(min_j)
                current_node = min_j
            else:
                break

        # Προσθήκη της διαδρομής στο Solution και υπολογισμός του κόστους της
        rt_tn_km, rt_load = calculate_route_details(current_route, empty_vehicle_weight)
        sol.routes.append([node.ID for node in current_route])  # Αποθήκευση της διαδρομής ως λίστα ID
        sol.total_cost += rt_tn_km

    return sol

# Μέθοδος για αποθήκευση της λύσης σε αρχείο
def save_solution_to_file(solution, filename='MEDEBE/MySolution/solution.txt'):
    with open(filename, 'w') as file:
        file.write(f"Cost:\n{solution.total_cost}\n")
        file.write("Routes:\n")
        file.write(f"{len(solution.routes)}\n")
        for route in solution.routes:
            file.write(",".join(map(str, route)) + "\n")

# Κλήση της μεθόδου TSP για πολλαπλά φορτηγά
sol = tsp_method(distance_matrix, all_nodes, total_nodes, capacity)

# Αποθήκευση της λύσης σε αρχείο
save_solution_to_file(sol)

# Εμφάνιση των διαδρομών και του συνολικού κόστους
for idx, route in enumerate(sol.routes):
    print(f"Διαδρομή φορτηγού {idx + 1}: {route}")
print("Συνολικό κόστος:", sol.total_cost)
       
        
