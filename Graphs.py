import heapq

class Graph:
    def __init__(self):
        self.vertices = {}

    def add_vertex(self, vertex):
        if vertex not in self.vertices:
            self.vertices[vertex] = {}

    def add_edge(self, source, destination, weight):
        if source in self.vertices and destination in self.vertices:
            self.vertices[source][destination] = weight
            self.vertices[destination][source] = weight  # For undirected graph

    def get_neighbors(self, vertex):
        if vertex in self.vertices:
            return self.vertices[vertex]
        else:
            return {}

def dijkstra(graph, start):
    # Priority queue to store (distance, vertex)
    priority_queue = []
    # Dictionary to store the shortest path to each vertex
    distances = {vertex: float('infinity') for vertex in graph.vertices}
    # Dictionary to store the shortest path tree
    previous_vertices = {vertex: None for vertex in graph.vertices}
    
    # Set the distance to the start node to 0 and add it to the priority queue
    distances[start] = 0
    heapq.heappush(priority_queue, (0, start))
    
    while priority_queue:
        current_distance, current_vertex = heapq.heappop(priority_queue)
        
        if current_distance > distances[current_vertex]:
            continue
        
        # Explore neighbors
        for neighbor, weight in graph.get_neighbors(current_vertex).items():
            distance = current_distance + weight
            
            # Only consider this new path if it's better
            if distance < distances[neighbor]:
                distances[neighbor] = distance
                previous_vertices[neighbor] = current_vertex
                heapq.heappush(priority_queue, (distance, neighbor))
    
    return distances, previous_vertices

def print_paths(start, previous_vertices):
    paths = {}
    for vertex in previous_vertices:
        path = []
        current = vertex
        while current is not None:
            path.append(current)
            current = previous_vertices[current]
        path.reverse()
        paths[vertex] = path
    print(f"Paths from {start}:")
    for vertex, path in paths.items():
        print(f"Path to {vertex}: {' -> '.join(path)}")

# Example Usage
graph = Graph()
graph.add_vertex('A')
graph.add_vertex('B')
graph.add_vertex('C')
graph.add_vertex('D')
graph.add_edge('A', 'B', 5)
graph.add_edge('B', 'C', 3)
graph.add_edge('A', 'C', 10)
graph.add_edge('C', 'D', 2)
graph.add_edge('B', 'D', 6)

start_vertex = 'A'
distances, previous_vertices = dijkstra(graph, start_vertex)

print(f"Shortest distances from {start_vertex}:")
for vertex, distance in distances.items():
    print(f"Distance to {vertex}: {distance}")

print_paths(start_vertex, previous_vertices)
