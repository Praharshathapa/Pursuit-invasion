
# # BFS
# import random
# import csv

# class Vehicle:
#     def __init__(self, name):
#         self.name = name
#         self.current_location = None
    
#     def move(self, other_vehicle):
#         if self.current_location is None:
#             self.current_location = random.choice(list(locations.keys()))
#         else:
#             connected_locations = [conn[1] for conn in connections if conn[0] == self.current_location]
#             if connected_locations:
#                 if other_vehicle.current_location in connected_locations:
#                     self.current_location = other_vehicle.current_location
#                 else:
#                     shortest_path = find_shortest_path(self.current_location, other_vehicle.current_location)
#                     if shortest_path is not None and len(shortest_path) >= 2:
#                         next_location = shortest_path[1]
#                         self.current_location = next_location
#                     else:
#                         # If no path is found, move randomly to a connected location
#                         self.current_location = random.choice(connected_locations)
#             else:
#                 # If no connected locations, move randomly
#                 self.current_location = random.choice(list(locations.keys()))
    
#     def __str__(self):
#         return f"{self.name} - Current Location: {locations[self.current_location]['name']}"



# def read_locations_from_csv():
#     locations = {}
#     with open('locations.csv', 'r') as csvfile:
#         reader = csv.reader(csvfile)
#         next(reader)  # Skip the header row
#         for row in reader:
#             location_id = int(row[0])
#             name = row[1]
#             latitude = float(row[2])
#             longitude = float(row[3])
#             locations[location_id] = {'name': name, 'coords': (latitude, longitude)}
#     return locations

# def read_connections_from_csv():
#     connections = []
#     with open('connections.csv', 'r') as csvfile:
#         reader = csv.reader(csvfile)
#         next(reader)  # Skip the header row
#         for row in reader:
#             from_location = int(row[0])
#             to_location = int(row[1])
#             distance = int(row[2])
#             connections.append((from_location, to_location, distance))
#     return connections



# def find_shortest_path(start, goal):
#     queue = [[start]]
#     while queue:
#         path = queue.pop(0)
#         current_location = path[-1]
#         if current_location == goal:
#             return path
#         connected_locations = [conn[1] for conn in connections if conn[0] == current_location]
#         for next_location in connected_locations:
#             if next_location not in path:
#                 new_path = list(path)
#                 new_path.append(next_location)
#                 queue.append(new_path)
#     return None


# def prompt_starting_positions():
#     print("Enter the starting positions for vehicle1 and vehicle2:")
#     vehicle1_start = int(input("Starting position for vehicle1: "))
#     vehicle2_start = int(input("Starting position for vehicle2: "))
#     return vehicle1_start, vehicle2_start

# # Read locations and connections from CSV files (same as before)
# locations = read_locations_from_csv()
# connections = read_connections_from_csv()

# # Prompt for starting positions
# vehicle1_start, vehicle2_start = prompt_starting_positions()

# # Set the starting positions for vehicle1 and vehicle2
# vehicle1 = Vehicle("Vehicle 1")
# vehicle1.current_location = vehicle1_start

# vehicle2 = Vehicle("Vehicle 2")
# vehicle2.current_location = vehicle2_start


# # List to store the shortest paths taken by vehicle1
# shortest_paths_vehicle1 = []
# # Simulation loop 
# for _ in range(50):  # Move vehicles for 50 steps
#     vehicle1.move(vehicle2)
#     vehicle2.move(vehicle1)
#     print(vehicle1)
#     print(vehicle2)
#     print()
#     # Check if vehicle1 caught vehicle2
#     if vehicle1.current_location == vehicle2.current_location:
#         print(f"{vehicle1.name} caught {vehicle2.name}!")
#         break
# prompt_starting_positions()
# # Store the shortest path taken by vehicle1 in this step
# shortest_path = find_shortest_path(vehicle1_start, vehicle1.current_location)
# shortest_paths_vehicle1.append(shortest_path)
# # Print the shortest paths taken by vehicle1
# for step, path in enumerate(shortest_paths_vehicle1):
#     print(f"Step {step+1}: {path}")






















# # dijkistra
# import random
# import heapq
# import csv

# class Vehicle:
#     def __init__(self, name, start_location):
#         self.name = name
#         self.current_location = start_location
#         self.path = [start_location]
#         self.visited = set([start_location])

#     def move_towards(self, other_vehicle):
#         distances, previous = dijkstra(self.current_location, other_vehicle)
#         shortest_path = reconstruct_path(previous, other_vehicle)
#         if shortest_path is not None and len(shortest_path) >= 2:
#             next_location = shortest_path[1]
#             self.current_location = next_location
#             self.path.append(next_location)
#             self.visited.add(next_location)

#     def move_randomly(self):
#         connected_locations = [conn[1] for conn in connections if conn[0] == self.current_location]
#         unvisited_locations = [loc for loc in connected_locations if loc not in self.visited]

#         if unvisited_locations:
#             next_location = random.choice(unvisited_locations)
#             self.current_location = next_location
#             self.path.append(next_location)
#             self.visited.add(next_location)
#         else:
#             # If all connected locations are visited, choose any visited location randomly
#             next_location = random.choice(list(self.visited))
#             self.current_location = next_location
#             self.path.append(next_location)
#     def __str__(self):
#         current_location_name = locations[self.current_location]['name']
#         path_names = [locations[location]['name'] for location in self.path]
#         return f"{self.name} - Current Location: {current_location_name} - Path: {path_names}"        


# def read_locations_from_csv():
#     locations = {}
#     with open('locations.csv', 'r') as csvfile:
#         reader = csv.reader(csvfile)
#         next(reader)  # Skip the header row
#         for row in reader:
#             location_id = int(row[0])
#             name = row[1]
#             latitude = float(row[2])
#             longitude = float(row[3])
#             locations[location_id] = {'name': name, 'coords': (latitude, longitude)}
#     return locations


# def read_connections_from_csv():
#     connections = []
#     with open('connections.csv', 'r') as csvfile:
#         reader = csv.reader(csvfile)
#         next(reader)  # Skip the header row
#         for row in reader:
#             from_location = int(row[0])
#             to_location = int(row[1])
#             distance = int(row[2])
#             connections.append((from_location, to_location, distance))
#     return connections


# locations = read_locations_from_csv()
# connections = read_connections_from_csv()


# def dijkstra(start, goal):
#     queue = [(0, start)]
#     distances = {location: float('inf') for location in locations}
#     distances[start] = 0
#     previous = {location: None for location in locations}

#     while queue:
#         current_distance, current_location = heapq.heappop(queue)

#         if current_distance > distances[current_location]:
#             continue

#         if current_location == goal:
#             break

#         for neighbor in get_neighbors(current_location):
#             distance = current_distance + 1  # Assuming all connections have a weight of 1
#             if distance < distances[neighbor]:
#                 distances[neighbor] = distance
#                 previous[neighbor] = current_location
#                 heapq.heappush(queue, (distance, neighbor))

#     return distances, previous


# def get_neighbors(location):
#     return [conn[1] for conn in connections if conn[0] == location or conn[1] == location]


# def reconstruct_path(previous, goal):
#     path = []
#     current_location = goal
#     while current_location is not None:
#         path.append(current_location)
#         current_location = previous[current_location]
#     path.reverse()
#     return path


# # Get user input for starting positions
# vehicle1_start = int(input("Enter starting position for Vehicle 1: "))
# vehicle2_start = int(input("Enter starting position for Vehicle 2: "))

# vehicle1 = Vehicle("Vehicle 1", vehicle1_start)
# vehicle2 = Vehicle("Vehicle 2", vehicle2_start)


# while vehicle1.current_location != vehicle2.current_location:
#     vehicle1.move_towards(vehicle2.current_location)
#     vehicle2.move_randomly()
#     print(vehicle1)
#     print(vehicle2)
#     print()

# # Print final paths
# print("Vehicle 1 Final Path:", vehicle1.path)
# print("Vehicle 2 Final Path:", vehicle2.path)





# def heuristic(location, goal):
#     coords1 = locations[location]["coords"]
#     coords2 = locations[goal]["coords"]

#     # Calculate the maximum distance to any other location
#     max_distance = max(
#         math.sqrt((coords2[0] - coord[0]) ** 2 + (coords2[1] - coord[1]) ** 2)
#         for coord in [loc["coords"] for loc in locations.values()]
#     )

#     # Calculate the distance between the two locations
#     distance = math.sqrt((coords2[0] - coords1[0]) ** 2 + (coords2[1] - coords1[1]) ** 2)

#     # Calculate the heuristic value as a fraction of the maximum distance
#     heuristic_value = distance / max_distance

#     return heuristic_value





import csv
import random
import math
import networkx as nx
import matplotlib.pyplot as plt
import heapq


class PriorityQueue:
    def __init__(self):
        self._queue = []
        self._index = 0

    def push(self, item, priority):
        heapq.heappush(self._queue, (priority, self._index, item))
        self._index += 1

    def pop(self):
        return heapq.heappop(self._queue)[-1]

    def empty(self):
        return len(self._queue) == 0
 

class Vehicle:
    def __init__(self, name, start_location):

        self.name = name
        self.current_location = start_location
        self.path = [start_location]
        self.visited = set([start_location])

    def move(self, other_vehicle):

        if self.current_location == other_vehicle.current_location:
            return  # Already caught the other vehicle

        if self.name == "Chaser":
            self.move_towards(other_vehicle.current_location)
        else:
            self.move_randomly()

    def move_towards(self, target_location):
        distances, previous = astar(self.current_location, target_location)
        shortest_path = reconstruct_path(previous, target_location)
        next_location = shortest_path[1]
        if next_location in self.path:
            return
        self.current_location = next_location
        self.path.append(next_location)

   

    def move_randomly(self):
        connected_locations = [conn[1] for conn in connections if conn[0] == self.current_location]
        unvisited_locations = [loc for loc in connected_locations if loc not in self.visited]

        if unvisited_locations:
            next_location = random.choice(unvisited_locations)
            self.current_location = next_location
            self.path.append(next_location)
            self.visited.add(next_location)
        else:
        # If all connected locations are visited, choose any visited location randomly
            next_location = random.choice(list(self.visited))
            self.current_location = next_location
            self.path.append(next_location)


    def __str__(self):
        return f"{self.name} - Current Location: {locations[self.current_location]['name']} - Path: {self.path}"

  


def read_locations_from_csv():
    locations = {}
    with open('locations.csv', 'r') as csvfile:
        reader = csv.reader(csvfile)
        next(reader)  # Skip the header row
        for row in reader:

            location_id = int(row[0])
            name = row[1]
            latitude = float(row[2])
            longitude = float(row[3])
            locations[location_id] = {'name': name, 'coords': (latitude, longitude)}

    return locations


def read_connections_from_csv():
    connections = []
    with open('connections.csv', 'r') as csvfile:
        reader = csv.reader(csvfile)
        next(reader)  # Skip the header row
        for row in reader:

            from_location = int(row[0])
            to_location = int(row[1])
            distance = int(row[2])
            connections.append((from_location, to_location, distance))

    return connections




locations = read_locations_from_csv()
connections = read_connections_from_csv()


def graph_normal():
    # Create an empty graph
    graph = nx.Graph()

    # Add edges from the connections
    for u, v, d in connections:
        graph.add_edge(u, v, distance=d)

    # Create the labels dictionary
    # labels = {node: str(node) for node in graph.nodes()}
    labels = {node: data["name"] for node, data in locations.items()}


    # Generate the graph with the connections
    pos = nx.spring_layout(graph, seed=42, k=1.2)

    # Draw the graph
    plt.figure(figsize=(12, 8))
    nx.draw_networkx_nodes(graph, pos, node_size=100, node_color='cyan')
    nx.draw_networkx_edges(graph, pos, edgelist=graph.edges(), edge_color='gray', alpha=0.5)
    nx.draw_networkx_labels(graph, pos, labels=labels, font_size=7, font_color='black')
    nx.draw_networkx_edge_labels(graph, pos, edge_labels=nx.get_edge_attributes(graph, 'distance'), font_size=8)
    plt.title("Locations Name")
    plt.axis('off')
    plt.tight_layout()
    plt.show()


def plot_graph_with_paths(vehicle1, vehicle2):
    # Create an empty graph
    graph = nx.Graph()

    # Add edges from the connections
    for u, v, d in connections:
        graph.add_edge(u, v, distance=d)

    # Create the labels dictionary
    labels = {node: data["name"] for node, data in locations.items()}

    # Generate the graph with the connections
    pos = nx.spring_layout(graph, seed=42, k=1.2)

    # Draw the graph
    plt.figure(figsize=(12, 8))
    nx.draw_networkx_nodes(graph, pos, node_size=100, node_color='lime')
    nx.draw_networkx_edges(graph, pos, edgelist=graph.edges(), edge_color='gray', alpha=0.5)
    nx.draw_networkx_labels(graph, pos, labels=labels, font_size=6, font_color='black')
    nx.draw_networkx_edge_labels(graph, pos, edge_labels=nx.get_edge_attributes(graph, 'distance'), font_size=8)

    # Plot the path taken by Vehicle 1
    path1 = vehicle1.path
    nx.draw_networkx_edges(graph, pos, edgelist=list(zip(path1[:-1], path1[1:])), edge_color='red', width=2)

    # Plot the path taken by Vehicle 2
    path2 = vehicle2.path
    nx.draw_networkx_edges(graph, pos, edgelist=list(zip(path2[:-1], path2[1:])), edge_color='violet', width=2)

    # Check if Vehicle 1 has caught Vehicle 2
    if vehicle1.path[-1] == vehicle2.path[-1]:
        caught_node = vehicle1.path[-1]
        nx.draw_networkx_nodes(graph, pos, nodelist=[caught_node], node_size=300, node_color='orange')

    plt.title("Graph with  Optimal Paths")
    plt.axis('off')
    plt.tight_layout()
    plt.show()
    


def astar(start, goal):
    queue = PriorityQueue()
    queue.push(start, 0)
    distances = {location: float('inf') for location in locations}
    distances[start] = 0
    previous = {location: None for location in locations}

    while not queue.empty():
        current_location = queue.pop()

        if current_location == goal:
            break

        for neighbor, cost in get_neighbors(current_location):
            distance = distances[current_location] + cost
            if distance < distances[neighbor]:
                distances[neighbor] = distance
                previous[neighbor] = current_location
                queue.push(neighbor, distance + heuristic(neighbor, goal))

    return distances, previous


def get_neighbors(location):
    neighbors = []
    for connection in connections:
        if connection[0] == location:
            neighbors.append((connection[1], connection[2]))
        elif connection[1] == location:
            neighbors.append((connection[0], connection[2]))
    return neighbors


def heuristic(location, goal):
    lat1, lon1 = locations[location]['coords']
    lat2, lon2 = locations[goal]['coords']
    radius = 6371  # Earth's radius in kilometers

    # Convert latitude and longitude to radians
    lat1 = math.radians(lat1)
    lon1 = math.radians(lon1)
    lat2 = math.radians(lat2)
    lon2 = math.radians(lon2)

    # Haversine formula
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    distance = radius * c

    return distance


def reconstruct_path(previous, goal):
    path = []
    current_location = goal

    while current_location is not None:
        if current_location not in previous:
            return None  # Path cannot be reconstructed
        path.append(current_location)
        current_location = previous[current_location]

    return list(reversed(path))


def main():
    graph_normal()
    vehicle1_start = int(input("Enter starting position for Chaser: "))
    vehicle2_start = int(input("Enter starting position for Runner: "))

    # Create Vehicle objects
    vehicle1 = Vehicle("Chaser", vehicle1_start)
    vehicle2 = Vehicle("Runner", vehicle2_start)


    while vehicle1.current_location != vehicle2.current_location:
        print(vehicle1)
        print(vehicle2)
        print()
        vehicle1.move_towards(vehicle2.current_location)
        vehicle2.move_randomly()

   
    # Print the final positions
    print("Final Positions:")
    print(vehicle1)
    print(vehicle2)

    # Plot the graph with paths
    plot_graph_with_paths(vehicle1, vehicle2)


if __name__ == "__main__":
    main()






# def move_towards(self, target_location):
#     distances, previous = astar(self.current_location, target_location)
#     shortest_path = reconstruct_path(previous, target_location)

#     assert shortest_path is not None and len(shortest_path) >= 2, "Invalid shortest path"

#     next_location = shortest_path[1]
#     self.current_location = next_location
#     self.path.append(next_location)




# def main():
#     graph_normal()
#     vehicle1_start = int(input("Enter starting position for Chaser: "))
#     vehicle2_start = int(input("Enter starting position for Runner: "))

#     # Create Vehicle objects
#     vehicle1 = Vehicle("Chaser", vehicle1_start)
#     vehicle2 = Vehicle("Runner", vehicle2_start)


#     while vehicle1.current_location != vehicle2.current_location:
#         vehicle1.move_towards(vehicle2.current_location)
#         vehicle2.move_randomly()
#         print(vehicle1)
#         print(vehicle2)
#         print()

#     assert vehicle1.current_location == vehicle2.current_location, "Chaser and Runner caught"
#     assert plt.get_fignums(), "Graph display failed"
#     assert len(vehicle1.path) > 1 and len(vehicle2.path) > 1, "Invalid path length"
#     # Print the final positions
#     print("Final Positions:")
#     print(vehicle1)
#     print(vehicle2)

#     # Plot the graph with paths
#     plot_graph_with_paths(vehicle1, vehicle2)
#     assert plt.get_fignums(), "Graph display failed"


# if __name__ == "__main__":
#     main()


