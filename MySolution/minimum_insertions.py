import math

class Node:
    def __init__(self, idd, xx, yy, dem=0, st=0):
        self.x = xx
        self.y = yy
        self.ID = idd
        self.isRouted = False
        self.demand = dem


def load_instance(file_name):
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
    tot_dem = sum(n.demand for n in nodes_sequence)
    tot_load = empty_vehicle_weight + tot_dem
    tn_km = 0
    for i in range(len(nodes_sequence) - 1):
        from_node = nodes_sequence[i]
        to_node = nodes_sequence[i + 1]
        tn_km += distance(from_node, to_node) * tot_load
        tot_load -= to_node.demand
    return tn_km, tot_dem


def save_solution_to_file(file_name, routes, total_cost):
    with open(file_name, "w") as f:
        # Write the total cost (objective function value)
        f.write(f"Cost:\n{total_cost}\n")
        # Write the number of routes
        f.write(f"Routes:\n{len(routes)}\n")
        # Write each route starting from 0, including 0 at the beginning
        for route in routes:
            route_str = "0," + ",".join(str(node.ID) for node in route[1:])  # Include 0 at the start
            f.write(f"{route_str}\n")


def solve_vrp(all_nodes, capacity, empty_vehicle_weight):
    depot = all_nodes[0]
    customers = all_nodes[1:]

    for customer in customers:
        customer.isRouted = False

    routes = []
    total_cost = 0

    while any(not customer.isRouted for customer in customers):
        route = [depot]
        load = 0
        while True:
            nearest_customer = None
            min_dist = float('inf')
            for customer in customers:
                if not customer.isRouted and load + customer.demand <= capacity:
                    dist = distance(route[-1], customer)
                    if dist < min_dist:
                        nearest_customer = customer
                        min_dist = dist

            if nearest_customer is None:
                break

            route.append(nearest_customer)
            nearest_customer.isRouted = True
            load += nearest_customer.demand

        routes.append(route)
        route_cost, _ = calculate_route_details(route, empty_vehicle_weight)
        total_cost += route_cost

    return routes, total_cost


def main():
    instance_file = "Instance.txt"
    solution_file = "solution.txt"

    all_nodes, capacity, empty_vehicle_weight = load_instance(instance_file)
    routes, total_cost = solve_vrp(all_nodes, capacity, empty_vehicle_weight)
    save_solution_to_file(solution_file, routes, total_cost)
    print(f"Solution saved to {solution_file}")


if __name__ == "__main__":
    main()

