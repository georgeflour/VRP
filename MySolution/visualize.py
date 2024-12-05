from Model import Model
import matplotlib.pyplot as plt

# Initialize the model and load data
model = Model('/Users/giorgosphlourakes/Desktop/MEDEBE/MEDEBE/MySolution/Instance.txt')
all_nodes, capacity, empty_vehicle_weight, depot = Model.load_model('/Users/giorgosphlourakes/Desktop/MEDEBE/MEDEBE/MySolution/Instance.txt')
distance_matrix = model.calculate_distance_matrix()


def extract_routes(file_name, all_nodes):
    all_lines = list(open(file_name, "r"))
    line = all_lines[1]

    separator = ','
    routes = []
    line_counter = 4

    # Read the number of vehicles used
    vehs_used = int(all_lines[3].strip())

    # Extract routes
    for i in range(vehs_used):
        ln = all_lines[line_counter].strip()
        no_commas = ln.split(sep=separator)
        route = [int(node_id) for node_id in no_commas]
        routes.append(route)
        line_counter += 1

    return routes

# Extract routes from solution file
routes = extract_routes("solution.txt", all_nodes)


def visualize_routes(routes, all_nodes, depot):
    # Create a figure
    plt.figure(figsize=(10, 8))

    # Plot all nodes
    for node in all_nodes:
        if node.ID == depot.ID:
            plt.scatter(node.x, node.y, c='red', s=100, edgecolors='black', label="Depot")
        else:
            plt.scatter(node.x, node.y, c='blue', s=50)
        plt.text(node.x, node.y, f"{node.ID}", fontsize=9, ha='center', va='center')

    # Plot routes
    colors = plt.cm.tab10.colors  # Use a colormap for route colors
    for idx, route in enumerate(routes):
        route_x = [all_nodes[node_id].x for node_id in route]
        route_y = [all_nodes[node_id].y for node_id in route]
        plt.plot(route_x, route_y, label=f"Route {idx + 1}", color=colors[idx % len(colors)])

    # Add legend, title, and labels
    plt.legend()
    plt.title("Vehicle Routing Solution")
    plt.xlabel("X-coordinate")
    plt.ylabel("Y-coordinate")
    plt.grid(True)

    # Show the plot
    plt.show()

# Visualize the routes
visualize_routes(routes, all_nodes, depot)
