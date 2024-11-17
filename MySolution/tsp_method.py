from Model import Model


model = Model('/Users/giorgosphlourakes/Desktop/MEDEBE/MySolution/Instance.txt')
all_nodes, capacity, empty_vehicle_weight, depot = Model.load_model('/Users/giorgosphlourakes/Desktop/MEDEBE/MySolution/Instance.txt')
total_nodes = len(all_nodes)
total_demand = model.total_demand
distance_matrix = model.calculate_distance_matrix()
#print(capacity, empty_vehicle_weight, depot.ID, depot.demand, depot.x, depot.y)

# Κλάση που ορίζει την αλληλουχία των κόμβων και το κόστος      
class Solution :
    def __init__(self) :
        self.node_sequence = []
        self.cost = 0

# Στην αλληλουχία των κόμβων προσθέτουμε τον κόμβο 0 αφού πάντα απο εκεί ξεκινάμε (depot)
# Βάζω το depot.ID γιατί
sol = Solution()
sol.node_sequence.append(depot.ID)

# Μέθοδος που βρίσκει το j με την ελάχιστη απόσταση για ένα συγκεκριμένο i
def find_min_distance_for_i(distance_matrix, i):
    min_distance = float('inf')
    min_j = -1
    for j in range(len(distance_matrix[i])):
        if i != j and distance_matrix[i][j] < min_distance:
            min_distance = distance_matrix[i][j]
            min_j = j
    return min_j, min_distance
# Εδώ βρίσκεται η μεθοδολογία που θα υπολογίζει την μεταβλητή nodes_sequence
#
#
def tsp_method(distance_matrix, all_nodes, total_nodes, total_demand):
    current_demand = 0
    track_space = capacity
    iteration_count = 0  # Για παρακολούθηση του αριθμού των επαναλήψεων
    while current_demand <= total_demand:
        iteration_count += 1
        for i in range(total_nodes):
            min_j, min_distance = find_min_distance_for_i(distance_matrix, i)
            if all_nodes[min_j].demand <= track_space:
                sol.node_sequence.append(min_j)
                track_space -= all_nodes[min_j].demand
                current_demand += all_nodes[min_j].demand
            else:
                break
        # Πρόσθεσε συνθήκες τερματισμού για αποφυγή άπειρων επαναλήψεων
        if iteration_count > total_nodes * 2:  # Ανώτατο όριο για έλεγχο
            print("Η επανάληψη σταμάτησε για να αποτραπεί ατελείωτη εκτέλεση.")
            break
    return sol.node_sequence


sol.node_sequence = tsp_method(distance_matrix, all_nodes, total_nodes, total_demand)
print(sol.node_sequence)
           
        
