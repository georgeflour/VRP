import random
from Model import *
from SolutionDrawer import *

# Κλάση που ορίζει την αλληλουχία των κόμβων και το κόστος      
class Solution:
    def __init__(self):
        self.routes = []  # Λίστα που αποθηκεύει τις διαδρομές για κάθε φορτηγό
        self.total_cost = 0  # Συνολικό κόστος της λύσης


class RelocationMove(object):
    def __init__(self):
        self.originRoutePosition = None
        self.targetRoutePosition = None
        self.originNodePosition = None
        self.targetNodePosition = None
        self.total_costChangeOriginRt = None
        self.total_costChangeTargetRt = None
        self.moveCost = None

    def Initialize(self):
        self.originRoutePosition = None
        self.targetRoutePosition = None
        self.originNodePosition = None
        self.targetNodePosition = None
        self.total_costChangeOriginRt = None
        self.total_costChangeTargetRt = None
        self.moveCost = 10 ** 9


class SwapMove(object):
    def __init__(self):
        self.positionOfFirstRoute = None
        self.positionOfSecondRoute = None
        self.positionOfFirstNode = None
        self.positionOfSecondNode = None
        self.total_costChangeFirstRt = None
        self.total_costChangeSecondRt = None
        self.moveCost = None
    def Initialize(self):
        self.positionOfFirstRoute = None
        self.positionOfSecondRoute = None
        self.positionOfFirstNode = None
        self.positionOfSecondNode = None
        self.total_costChangeFirstRt = None
        self.total_costChangeSecondRt = None
        self.moveCost = 10 ** 9


class CustomerInsertion(object):
    def __init__(self):
        self.customer = None
        self.route = None
        self.total_cost = 10 ** 9

class CustomerInsertionAllPositions(object):
    def __init__(self):
        self.customer = None
        self.route = None
        self.insertionPosition = None
        self.total_cost = 10 ** 9

class TwoOptMove(object):
    def __init__(self):
        self.positionOfFirstRoute = None
        self.positionOfSecondRoute = None
        self.positionOfFirstNode = None
        self.positionOfSecondNode = None
        self.moveCost = None
    def Initialize(self):
        self.positionOfFirstRoute = None
        self.positionOfSecondRoute = None
        self.positionOfFirstNode = None
        self.positionOfSecondNode = None
        self.moveCost = 10 ** 9


class Solver:
    def __init__(self, m: Model):
        self.allNodes = m.allNodes
        self.customers = m.customers
        self.depot = m.allNodes[0]
        self.matrix = m.matrix
        self.capacity = m.capacity
        self.empty_vehicle_weight = m.empty_vehicle_weight
        self.total_nodes = len(m.allNodes)
        self.sol = None
        self.bestSolution = None

    def solve(self):
        self.sol = self.constructor()
        self.save_solution_to_file(self.sol)
        self.ReportSolution(self.sol)
        
        
        self.LocalSearch()
        


        self.ReportSolution(self.sol)
        self.save_solution_to_file(self.sol)
        SolDrawer.draw(0, self.sol, self.allNodes)
        return(self.sol)

    def constructor(self):
        sol = Solution()
        visited_nodes = {self.depot.ID}
        remaining_demand = sum(node.demand for node in self.customers)

        while remaining_demand > 0:
            route = Route(self.depot, self.capacity)
            track_space = self.capacity
            current_node = self.depot

            while track_space > 0 and len(visited_nodes) < len(self.allNodes):
                min_j, min_distance = self.find_min_distance_for_i(current_node.ID, visited_nodes)
                if min_j == -1:
                    break

                candidate_node = self.allNodes[min_j]
                if candidate_node.demand <= track_space and not candidate_node.isRouted:
                    route.sequenceOfNodes.append(candidate_node)
                    candidate_node.isRouted = True
                    track_space -= candidate_node.demand
                    remaining_demand -= candidate_node.demand
                    visited_nodes.add(candidate_node.ID)
                    current_node = candidate_node
                else:
                    break

            # Remove the depot as the last node for this route
            route.total_cost, route.load = self.calculate_route_details(route.sequenceOfNodes, self.empty_vehicle_weight)
            sol.routes.append(route)
            sol.total_cost += route.total_cost

        return sol

    def find_min_distance_for_i(self, i, visited_nodes):
        min_distance = float('inf')
        min_j = -1
        for j in range(len(self.matrix[i])):
            if i != j and j not in visited_nodes and self.matrix[i][j] < min_distance:
                min_distance = self.matrix[i][j]
                min_j = j
        return min_j, min_distance

    def calculate_route_details(self, nodes_sequence, empty_vehicle_weight):
        total_load = sum(node.demand for node in nodes_sequence if node != self.depot)
        tn_km = 0
        current_load = empty_vehicle_weight + total_load

        for i in range(len(nodes_sequence) - 1):
            from_node = nodes_sequence[i]
            to_node = nodes_sequence[i + 1]
            distance = self.matrix[from_node.ID][to_node.ID]
            tn_km += distance * current_load
            current_load -= to_node.demand if to_node != self.depot else 0

        return tn_km, total_load



    def save_solution_to_file(self, solution, filename='solution.txt'):
        with open(filename, 'w') as file:
            file.write(f"Cost:\n{solution.total_cost}\n")
            file.write("Routes:\n")
            file.write(f"{len(solution.routes)}\n")
            for route in solution.routes:
                node_ids = [node.ID for node in route.sequenceOfNodes]
                file.write(",".join(map(str, node_ids)) + "\n")

    def cloneSolution(self, sol):
        cloned = Solution()
        for route in sol.routes:
            clonedRoute = self.cloneRoute(route)
            cloned.routes.append(clonedRoute)
        cloned.total_cost = sol.total_cost
        return cloned

    def cloneRoute(self, rt):
        cloned = Route(self.depot, rt.capacity)
        cloned.sequenceOfNodes = rt.sequenceOfNodes.copy()
        cloned.total_cost = rt.total_cost
        cloned.load = rt.load
        return cloned
    def LocalSearch(self):
        random.seed(1)
        temperature = 1000
        cooling_rate = 0.995
        min_temperature = 1e-3
        max_iterations = 400
        self.bestSolution = self.cloneSolution(self.sol)
        localSearchIterator=0

        rm = RelocationMove()
        sm = SwapMove()
        top = TwoOptMove()

        

        while temperature > min_temperature and localSearchIterator < max_iterations:  # Perform local search until no improvement
            self.InitializeOperators(rm, sm, top)
            operator = random.randint(0, 2)
            # Relocations
            if operator == 0:
                self.FindBestRelocationMove(rm)
                if rm.originRoutePosition is not None:
                    if rm.moveCost < 0 or random.random() < self.acceptance_probability(rm.moveCost, temperature):
                        self.ApplyRelocationMove(rm)
                # Swaps
            elif operator == 1:
                self.FindBestSwapMove(sm)
                if sm.positionOfFirstRoute is not None:
                    if sm.moveCost < 0 or random.random() < self.acceptance_probability(rm.moveCost, temperature):
                        self.ApplySwapMove(sm)
                # Two-opt
            elif operator == 2:
                self.FindBestTwoOptMove(top)
                if top.positionOfFirstRoute is not None and (top.moveCost < 0 or random.random() < self.acceptance_probability(rm.moveCost, temperature)):
                    self.ApplyTwoOptMove(top)

            self.sol.total_cost = self.CalculateTotalCost(self.sol)
            self.TestSolution()

            if self.sol.total_cost < self.bestSolution.total_cost:
                self.bestSolution = self.cloneSolution(self.sol)
            temperature *= cooling_rate
            localSearchIterator += 1
            if localSearchIterator % 100 == 0:
                print(f"Iteration {localSearchIterator}, Temperature: {temperature:.4f}, Best Cost: {self.bestSolution.total_cost}")

        self.sol = self.bestSolution
    def acceptance_probability(self, moveCost, temperature):
        return min(1, math.exp(-moveCost / temperature))

    def FindBestRelocationMove(self, rm):
        for originRouteIndex in range(0, len(self.sol.routes)):
            rt1:Route = self.sol.routes[originRouteIndex]
            for originNodeIndex in range(1, len(rt1.sequenceOfNodes) - 1):
                for targetRouteIndex in range (0, len(self.sol.routes)):
                    rt2:Route = self.sol.routes[targetRouteIndex]
                    for targetNodeIndex in range (0, len(rt2.sequenceOfNodes) - 1):

                        if originRouteIndex == targetRouteIndex and (targetNodeIndex == originNodeIndex or targetNodeIndex == originNodeIndex - 1):
                            continue

                        A = rt1.sequenceOfNodes[originNodeIndex - 1]
                        B = rt1.sequenceOfNodes[originNodeIndex]
                        C = rt1.sequenceOfNodes[originNodeIndex + 1]

                        F = rt2.sequenceOfNodes[targetNodeIndex]
                        G = rt2.sequenceOfNodes[targetNodeIndex + 1]

                        if rt1 != rt2:
                            if rt2.load + B.demand > rt2.capacity:
                                continue

                        costAdded = self.matrix[A.ID][C.ID] + self.matrix[F.ID][B.ID] + self.matrix[B.ID][G.ID]
                        costRemoved = self.matrix[A.ID][B.ID] + self.matrix[B.ID][C.ID] + self.matrix[F.ID][G.ID]

                        originRtCostChange = self.matrix[A.ID][C.ID] - self.matrix[A.ID][B.ID] - self.matrix[B.ID][C.ID]
                        targetRtCostChange = self.matrix[F.ID][B.ID] + self.matrix[B.ID][G.ID] - self.matrix[F.ID][G.ID]

                        moveCost = costAdded - costRemoved

                        if (moveCost < rm.moveCost):
                            self.StoreBestRelocationMove(originRouteIndex, targetRouteIndex, originNodeIndex, targetNodeIndex, moveCost, originRtCostChange, targetRtCostChange, rm)

    def FindBestSwapMove(self, sm):
        for firstRouteIndex in range(0, len(self.sol.routes)):
            rt1:Route = self.sol.routes[firstRouteIndex]
            for secondRouteIndex in range (firstRouteIndex, len(self.sol.routes)):
                rt2:Route = self.sol.routes[secondRouteIndex]
                for firstNodeIndex in range (1, len(rt1.sequenceOfNodes) - 1):
                    startOfSecondNodeIndex = 1
                    if rt1 == rt2:
                        startOfSecondNodeIndex = firstNodeIndex + 1
                    for secondNodeIndex in range (startOfSecondNodeIndex, len(rt2.sequenceOfNodes) - 1):

                        a1 = rt1.sequenceOfNodes[firstNodeIndex - 1]
                        b1 = rt1.sequenceOfNodes[firstNodeIndex]
                        c1 = rt1.sequenceOfNodes[firstNodeIndex + 1]

                        a2 = rt2.sequenceOfNodes[secondNodeIndex - 1]
                        b2 = rt2.sequenceOfNodes[secondNodeIndex]
                        c2 = rt2.sequenceOfNodes[secondNodeIndex + 1]

                        moveCost = None
                        costChangeFirstRoute = None
                        costChangeSecondRoute = None

                        if rt1 == rt2:
                            if firstNodeIndex == secondNodeIndex - 1:
                                # case of consecutive nodes swap
                                costRemoved = self.matrix[a1.ID][b1.ID] + self.matrix[b1.ID][b2.ID] + \
                                              self.matrix[b2.ID][c2.ID]
                                costAdded = self.matrix[a1.ID][b2.ID] + self.matrix[b2.ID][b1.ID] + \
                                            self.matrix[b1.ID][c2.ID]
                                moveCost = costAdded - costRemoved
                            else:

                                costRemoved1 = self.matrix[a1.ID][b1.ID] + self.matrix[b1.ID][c1.ID]
                                costAdded1 = self.matrix[a1.ID][b2.ID] + self.matrix[b2.ID][c1.ID]
                                costRemoved2 = self.matrix[a2.ID][b2.ID] + self.matrix[b2.ID][c2.ID]
                                costAdded2 = self.matrix[a2.ID][b1.ID] + self.matrix[b1.ID][c2.ID]
                                moveCost = costAdded1 + costAdded2 - (costRemoved1 + costRemoved2)
                        else:
                            if rt1.load - b1.demand + b2.demand > self.capacity:
                                continue
                            if rt2.load - b2.demand + b1.demand > self.capacity:
                                continue

                            costRemoved1 = self.matrix[a1.ID][b1.ID] + self.matrix[b1.ID][c1.ID]
                            costAdded1 = self.matrix[a1.ID][b2.ID] + self.matrix[b2.ID][c1.ID]
                            costRemoved2 = self.matrix[a2.ID][b2.ID] + self.matrix[b2.ID][c2.ID]
                            costAdded2 = self.matrix[a2.ID][b1.ID] + self.matrix[b1.ID][c2.ID]

                            costChangeFirstRoute = costAdded1 - costRemoved1
                            costChangeSecondRoute = costAdded2 - costRemoved2

                            moveCost = costAdded1 + costAdded2 - (costRemoved1 + costRemoved2)

                        if moveCost < sm.moveCost:
                            self.StoreBestSwapMove(firstRouteIndex, secondRouteIndex, firstNodeIndex, secondNodeIndex,
                                                   moveCost, costChangeFirstRoute, costChangeSecondRoute, sm)

    def ApplyRelocationMove(self, rm):
        origin_rt = self.sol.routes[rm.originRoutePosition]
        target_rt = self.sol.routes[rm.targetRoutePosition]
        
        # Move node and recompute costs
        B = origin_rt.sequenceOfNodes.pop(rm.originNodePosition)
        target_rt.sequenceOfNodes.insert(rm.targetNodePosition + 1, B)
        
        # Recalculate costs for both routes
        origin_rt.total_cost, origin_rt.load = self.calculate_route_details(origin_rt.sequenceOfNodes, self.empty_vehicle_weight)
        target_rt.total_cost, target_rt.load = self.calculate_route_details(target_rt.sequenceOfNodes, self.empty_vehicle_weight)
        
        # Update total solution cost
        self.sol.total_cost = self.CalculateTotalCost(self.sol)



    def ApplySwapMove(self, sm):
       oldCost = self.CalculateTotalCost(self.sol)
       rt1 = self.sol.routes[sm.positionOfFirstRoute]
       rt2 = self.sol.routes[sm.positionOfSecondRoute]
       b1 = rt1.sequenceOfNodes[sm.positionOfFirstNode]
       b2 = rt2.sequenceOfNodes[sm.positionOfSecondNode]
       rt1.sequenceOfNodes[sm.positionOfFirstNode] = b2
       rt2.sequenceOfNodes[sm.positionOfSecondNode] = b1

       if (rt1 == rt2):
           rt1.total_cost += sm.moveCost
       else:
           rt1.total_cost += sm.total_costChangeFirstRt
           rt2.total_cost += sm.total_costChangeSecondRt
           rt1.load = rt1.load - b1.demand + b2.demand
           rt2.load = rt2.load + b1.demand - b2.demand

       self.sol.total_cost += sm.moveCost

       newCost = self.CalculateTotalCost(self.sol)
       # debuggingOnly


    def ReportSolution(self, sol):
        for idx, route in enumerate(sol.routes):
            print(f"Route {idx + 1}: {' -> '.join(str(node.ID) for node in route.sequenceOfNodes)}")
            print(f"Route cost: {route.total_cost}")
        print(f"Total cost: {sol.total_cost}")

    def GetLastOpenRoute(self):
        if len(self.sol.routes) == 0:
            return None
        else:
            return self.sol.routes[-1]

    def IdentifyBestInsertion(self, bestInsertion, rt):
        for i in range(0, len(self.customers)):
            candidateCust:Node = self.customers[i]
            if candidateCust.isRouted is False:
                if rt.load + candidateCust.demand <= rt.capacity:
                    lastNodePresentInTheRoute = rt.sequenceOfNodes[-2]
                    trialCost = self.matrix[lastNodePresentInTheRoute.ID][candidateCust.ID]
                    if trialCost < bestInsertion.total_cost:
                        bestInsertion.customer = candidateCust
                        bestInsertion.route = rt
                        bestInsertion.total_cost = trialCost

    def ApplyCustomerInsertion(self, insertion):
        insCustomer = insertion.customer
        rt = insertion.route
        #before the second depot occurrence
        insIndex = len(rt.sequenceOfNodes) - 1
        rt.sequenceOfNodes.insert(insIndex, insCustomer)

        beforeInserted = rt.sequenceOfNodes[-3]

        costAdded = self.matrix[beforeInserted.ID][insCustomer.ID] + self.matrix[insCustomer.ID][self.depot.ID]
        costRemoved = self.matrix[beforeInserted.ID][self.depot.ID]

        rt.total_cost += costAdded - costRemoved
        self.sol.total_cost += costAdded - costRemoved

        rt.load += insCustomer.demand

        insCustomer.isRouted = True

    def StoreBestRelocationMove(self, originRouteIndex, targetRouteIndex, originNodeIndex, targetNodeIndex, moveCost, originRtCostChange, targetRtCostChange, rm:RelocationMove):
        rm.originRoutePosition = originRouteIndex
        rm.originNodePosition = originNodeIndex
        rm.targetRoutePosition = targetRouteIndex
        rm.targetNodePosition = targetNodeIndex
        rm.total_costChangeOriginRt = originRtCostChange
        rm.total_costChangeTargetRt = targetRtCostChange
        rm.moveCost = moveCost

    def StoreBestSwapMove(self, firstRouteIndex, secondRouteIndex, firstNodeIndex, secondNodeIndex, moveCost, costChangeFirstRoute, costChangeSecondRoute, sm):
        sm.positionOfFirstRoute = firstRouteIndex
        sm.positionOfSecondRoute = secondRouteIndex
        sm.positionOfFirstNode = firstNodeIndex
        sm.positionOfSecondNode = secondNodeIndex
        sm.total_costChangeFirstRt = costChangeFirstRoute
        sm.total_costChangeSecondRt = costChangeSecondRoute
        sm.moveCost = moveCost

    def CalculateTotalCost(self, sol):
        total_cost = 0
        for route in sol.routes:
            total_load = sum(node.demand for node in route.sequenceOfNodes if node != self.depot)
            tn_km = 0
            current_load = self.empty_vehicle_weight + total_load

            for i in range(len(route.sequenceOfNodes) - 1):
                from_node = route.sequenceOfNodes[i]
                to_node = route.sequenceOfNodes[i + 1]
                distance = self.matrix[from_node.ID][to_node.ID]
                tn_km += distance * current_load
                current_load -= to_node.demand if to_node != self.depot else 0

            total_cost += tn_km

        return total_cost

    def InitializeOperators(self, rm, sm, top):
        rm.Initialize()
        sm.Initialize()
        top.Initialize()

    def FindBestTwoOptMove(self, top):
        for rtInd1 in range(0, len(self.sol.routes)):
            rt1: Route = self.sol.routes[rtInd1]
            for rtInd2 in range(rtInd1, len(self.sol.routes)):
                rt2: Route = self.sol.routes[rtInd2]
                for nodeInd1 in range(0, len(rt1.sequenceOfNodes) - 1):
                    start2 = 0
                    if rt1 == rt2:
                        # Avoid overlapping segments within the same route
                        start2 = nodeInd1 + 2

                    for nodeInd2 in range(start2, len(rt2.sequenceOfNodes) - 1):
                        # Store original segments
                        original_rt1_segment = rt1.sequenceOfNodes[nodeInd1 + 1:]
                        original_rt2_segment = rt2.sequenceOfNodes[nodeInd2 + 1:]

                        # Perform two-opt swap
                        rt1.sequenceOfNodes = rt1.sequenceOfNodes[:nodeInd1 + 1] + original_rt2_segment
                        rt2.sequenceOfNodes = rt2.sequenceOfNodes[:nodeInd2 + 1] + original_rt1_segment

                        # Recalculate the cost and load of both routes
                        first_route_cost, first_route_load = self.calculate_route_details(rt1.sequenceOfNodes, self.empty_vehicle_weight)
                        second_route_cost, second_route_load = self.calculate_route_details(rt2.sequenceOfNodes, self.empty_vehicle_weight)

                        # Check for load violations
                        if first_route_load > self.capacity or second_route_load > self.capacity:
                            # Restore original segments and continue
                            rt1.sequenceOfNodes = rt1.sequenceOfNodes[:nodeInd1 + 1] + original_rt1_segment
                            rt2.sequenceOfNodes = rt2.sequenceOfNodes[:nodeInd2 + 1] + original_rt2_segment
                            continue

                        # Calculate cost change based on tn_km
                        costChangeFirstRoute = first_route_cost - rt1.total_cost
                        costChangeSecondRoute = second_route_cost - rt2.total_cost
                        moveCost = costChangeFirstRoute + costChangeSecondRoute

                        # Restore original segments
                        rt1.sequenceOfNodes = rt1.sequenceOfNodes[:nodeInd1 + 1] + original_rt1_segment
                        rt2.sequenceOfNodes = rt2.sequenceOfNodes[:nodeInd2 + 1] + original_rt2_segment
                        # Debugging: Check evaluated nodes and cost change
                        
                        # Update the best move if this one is better
                        if moveCost < top.moveCost:

                            self.StoreBestTwoOptMove(rtInd1, rtInd2, nodeInd1, nodeInd2, moveCost, top)



    def CapacityIsViolated(self, rt1, nodeInd1, rt2, nodeInd2):

        rt1FirstSegmentLoad = 0
        for i in range(0, nodeInd1 + 1):
            n = rt1.sequenceOfNodes[i]
            rt1FirstSegmentLoad += n.demand
        rt1SecondSegmentLoad = rt1.load - rt1FirstSegmentLoad

        rt2FirstSegmentLoad = 0
        for i in range(0, nodeInd2 + 1):
            n = rt2.sequenceOfNodes[i]
            rt2FirstSegmentLoad += n.demand
        rt2SecondSegmentLoad = rt2.load - rt2FirstSegmentLoad

        if (rt1FirstSegmentLoad + rt2SecondSegmentLoad > rt1.capacity):
            return True
        if (rt2FirstSegmentLoad + rt1SecondSegmentLoad > rt2.capacity):
            return True

        return False

    def StoreBestTwoOptMove(self, rtInd1, rtInd2, nodeInd1, nodeInd2, moveCost, top):
        top.positionOfFirstRoute = rtInd1
        top.positionOfSecondRoute = rtInd2
        top.positionOfFirstNode = nodeInd1
        top.positionOfSecondNode = nodeInd2
        top.moveCost = moveCost

    def ApplyTwoOptMove(self, top):
        rt1:Route = self.sol.routes[top.positionOfFirstRoute]
        rt2:Route = self.sol.routes[top.positionOfSecondRoute]

        if rt1 == rt2:
            # reverses the nodes in the segment [positionOfFirstNode + 1,  top.positionOfSecondNode]
            reversedSegment = reversed(rt1.sequenceOfNodes[top.positionOfFirstNode + 1: top.positionOfSecondNode + 1])
            #lst = list(reversedSegment)
            #lst2 = list(reversedSegment)
            rt1.sequenceOfNodes[top.positionOfFirstNode + 1 : top.positionOfSecondNode + 1] = reversedSegment

            #reversedSegmentList = list(reversed(rt1.sequenceOfNodes[top.positionOfFirstNode + 1: top.positionOfSecondNode + 1]))
            #rt1.sequenceOfNodes[top.positionOfFirstNode + 1: top.positionOfSecondNode + 1] = reversedSegmentList

            rt1.total_cost += top.moveCost

        else:
            #slice with the nodes from position top.positionOfFirstNode + 1 onwards
            relocatedSegmentOfRt1 = rt1.sequenceOfNodes[top.positionOfFirstNode + 1 :]

            #slice with the nodes from position top.positionOfFirstNode + 1 onwards
            relocatedSegmentOfRt2 = rt2.sequenceOfNodes[top.positionOfSecondNode + 1 :]

            del rt1.sequenceOfNodes[top.positionOfFirstNode + 1 :]
            del rt2.sequenceOfNodes[top.positionOfSecondNode + 1 :]

            rt1.sequenceOfNodes.extend(relocatedSegmentOfRt2)
            rt2.sequenceOfNodes.extend(relocatedSegmentOfRt1)

            self.UpdateRouteCostAndLoad(rt1)
            self.UpdateRouteCostAndLoad(rt2)
            

        self.sol.total_cost += top.moveCost

    def UpdateRouteCostAndLoad(self, rt: Route):
        tc = 0
        tl = 0
        for i in range(0, len(rt.sequenceOfNodes)):
            A = rt.sequenceOfNodes[i]
            
            tl += A.demand
        rt.load = tl
        tn_km = 0
        for i in range(0, len(rt.sequenceOfNodes) - 1):
            A = rt.sequenceOfNodes[i]
            B = rt.sequenceOfNodes[i+1]
            tc = self.matrix[A.ID][B.ID]
            tn_km += tc * tl
            tl-= A.demand

        rt.total_cost = tn_km
        


    def TestSolution(self):
        totalSolCost = 0
        for route in self.sol.routes:
            total_load = sum(node.demand for node in route.sequenceOfNodes if node != self.depot)
            tn_km = 0
            current_load = self.empty_vehicle_weight + total_load

            for i in range(len(route.sequenceOfNodes) - 1):
                from_node = route.sequenceOfNodes[i]
                to_node = route.sequenceOfNodes[i + 1]
                distance = self.matrix[from_node.ID][to_node.ID]
                tn_km += distance * current_load
                current_load -= to_node.demand if to_node != self.depot else 0

            totalSolCost += route.total_cost

        
    def IdentifyMinimumCostInsertion(self, best_insertion):
        for i in range(0, len(self.customers)):
            candidateCust: Node = self.customers[i]
            if candidateCust.isRouted is False:
                for rt in self.sol.routes:
                    if rt.load + candidateCust.demand <= rt.capacity:
                        for j in range(0, len(rt.sequenceOfNodes) - 1):
                            A = rt.sequenceOfNodes[j]
                            B = rt.sequenceOfNodes[j + 1]
                            costAdded = self.matrix[A.ID][candidateCust.ID] + self.matrix[candidateCust.ID][
                                B.ID]
                            costRemoved = self.matrix[A.ID][B.ID]
                            trialCost = costAdded - costRemoved
                            if trialCost < best_insertion.total_cost:
                                best_insertion.customer = candidateCust
                                best_insertion.route = rt
                                best_insertion.insertionPosition = j
                                best_insertion.total_cost = trialCost
                    else:
                        continue






    



