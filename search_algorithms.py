import math
import json
import random

import heapq

def euclidean_dist(a, b):
    return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2 + (a[2]-b[2])**2)

class MapGraph:
    """A Graph"""

    def __init__(self, json_file):
        """Constructor takes in a file path and loads in Vertices and Edges"""
        self.graph = None
        with open('graph.json') as json_file:
            self.graph = json.load(json_file)

    def get_vertices(self):
        """Returns vertices"""
        return list(self.graph["E"].keys())

    def get_neighbors(self, v):
        """Returns edges"""
        return self.graph["E"][v]

    def get_position(self, v):
        """Returns position of vertex"""
        return self.graph["V"][v]["position"]
    
    def find_closest_vertex(self, point):
        """Returns the closest vertex to point"""
        verts = self.get_vertices()
        closest = verts[0]
        min_dist = math.inf
        for v in verts:
            dist = euclidean_dist(point, self.get_position(v))
            if (min_dist > dist):
                min_dist = dist
                closest = v
        return closest


def point_to_point(start, dest):
    return [start, dest]

def fly(start, dest):
    path = []
    path.append(start)
    dist = euclidean_dist(start, dest)
    ds = 1.0/20.0
    d = ds
    while d < 1.0:
        d = d + ds
        path.append([(dest[0]-start[0])*d+start[0], start[1] + 50*((-(2*d-1)**2+1)), (dest[2]-start[2])*d+start[2]])
    path.append(dest)
    return path

def random_graph(start, dest):
    graph = MapGraph('graph.json')
    
    start_node = graph.find_closest_vertex(start)
    node = random.choice(graph.get_vertices())
    path = fly(graph.get_position(start_node), graph.get_position(node))
    visitied = {}

    for i in range(0, 1000):
        neighbors = graph.get_neighbors(node)
        for neighbor in neighbors:
            if (neighbor not in visitied):
                node = neighbor
                path.append(graph.get_position(node))
                visitied[node] = True
                break
    return path

def breadth_first(start, dest):
    graph = MapGraph('graph.json')
    
    start_node = graph.find_closest_vertex(start)
    dest_node = graph.find_closest_vertex(dest)

    from collections import deque

    queue = deque([start_node])
    visited = set()
    parent = {}  # To reconstruct path

    while queue:
        current = queue.popleft()
        if current == dest_node:
            break
        visited.add(current)
        for neighbor in graph.get_neighbors(current):
            if neighbor not in visited and neighbor not in queue:
                parent[neighbor] = current
                queue.append(neighbor)

    path = []
    node = dest_node
    while node != start_node:
        path.append(graph.get_position(node))
        node = parent[node]
    path.append(graph.get_position(start_node))
    path.reverse()
    
    return path

def depth_first(start, dest):
    graph = MapGraph('graph.json')
    start_node = graph.find_closest_vertex(start)
    dest_node = graph.find_closest_vertex(dest)

    stack = [start_node]
    visited = set()
    parent = {}

    while stack:
        current = stack.pop()
        if current == dest_node:
            break
        if current not in visited:
            visited.add(current)
            for neighbor in graph.get_neighbors(current):
                if neighbor not in visited:
                    parent[neighbor] = current
                    stack.append(neighbor)

    path = []
    node = dest_node
    while node != start_node:
        path.append(graph.get_position(node))
        node = parent[node]
    path.append(graph.get_position(start_node))
    path.reverse()
    return path

def depth_first_best(start, dest):
    from collections import deque

    graph = MapGraph('graph.json')
    start_node = graph.find_closest_vertex(start)
    dest_node = graph.find_closest_vertex(dest)

    stack = [start_node]
    visited = set()
    parent = {}

    while stack:
        current = stack.pop()

        if current == dest_node:
            break

        if current not in visited:
            visited.add(current)
            neighbors = graph.get_neighbors(current)
            # Sorting neighbors by distance to destination
            sorted_neighbors = sorted(
                neighbors,
                key=lambda n: euclidean_dist(graph.get_position(n), graph.get_position(dest_node)),
                reverse=False
            )
            for neighbor in sorted_neighbors:
                if neighbor not in visited and neighbor not in stack:
                    parent[neighbor] = current
                    stack.append(neighbor)

    if dest_node not in parent:
        return point_to_point(start, dest)

    path = [dest_node]
    while path[-1] != start_node:
        path.append(parent[path[-1]])
    path.reverse()

    return [graph.get_position(node) for node in path]


def dijkstra(start, dest):
    graph = MapGraph('graph.json')
    start_node = graph.find_closest_vertex(start)
    dest_node = graph.find_closest_vertex(dest)

    dist = {v: float('inf') for v in graph.get_vertices()}
    dist[start_node] = 0
    parent = {}
    heap = [(0, start_node)]

    while heap:
        cost, u = heapq.heappop(heap)
        if u == dest_node:
            break
        for v in graph.get_neighbors(u):
            alt = cost + euclidean_dist(graph.get_position(u), graph.get_position(v))
            if alt < dist[v]:
                dist[v] = alt
                parent[v] = u
                heapq.heappush(heap, (alt, v))

    path = []
    node = dest_node
    while node != start_node:
        path.append(graph.get_position(node))
        node = parent[node]
    path.append(graph.get_position(start_node))
    path.reverse()
    return path

def astar(start, dest):
    graph = MapGraph('graph.json')
    start_node = graph.find_closest_vertex(start)
    dest_node = graph.find_closest_vertex(dest)

    open_set = [(0, start_node)]
    came_from = {}
    g_score = {v: float('inf') for v in graph.get_vertices()}
    g_score[start_node] = 0

    def h(v):
        return euclidean_dist(graph.get_position(v), graph.get_position(dest_node))

    while open_set:
        _, current = heapq.heappop(open_set)
        if current == dest_node:
            break

        for neighbor in graph.get_neighbors(current):
            tent_g = g_score[current] + euclidean_dist(graph.get_position(current), graph.get_position(neighbor))
            if tent_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tent_g
                f_score = tent_g + h(neighbor)
                heapq.heappush(open_set, (f_score, neighbor))

    path = []
    node = dest_node
    while node != start_node:
        path.append(graph.get_position(node))
        node = came_from[node]
    path.append(graph.get_position(start_node))
    path.reverse()
    return path

def min_spanning_tree(start, dest):
    graph = MapGraph('graph.json')
    start_vertex = graph.find_closest_vertex(start)
    dest_vertex = graph.find_closest_vertex(dest)

    if start_vertex == dest_vertex:
        return [start, dest]

    vertices = graph.get_vertices()
    n = len(vertices)
    
    vertex_to_index = {vertices[i]: i for i in range(n)}
    index_to_vertex = {i: vertices[i] for i in range(n)}
    
    mst_edges = {}
    for v in vertices:
        mst_edges[v] = []
    
    key = [float('inf')] * n
    parent = [None] * n
    
    start_index = vertex_to_index[start_vertex]
    key[start_index] = 0

    mst_set = [False] * n
    
    # Constructing MST with n vertices
    for _ in range(n):
        min_key = float('inf')
        u_index = -1
        
        for v_index in range(n):
            if not mst_set[v_index] and key[v_index] < min_key:
                min_key = key[v_index]
                u_index = v_index
        
        if u_index == -1:
            break

        u = index_to_vertex[u_index]
        mst_set[u_index] = True
        
        if parent[u_index] is not None:
            parent_vertex = index_to_vertex[parent[u_index]]
            mst_edges[parent_vertex].append(u)
            mst_edges[u].append(parent_vertex)
        
        for v in graph.get_neighbors(u):
            v_index = vertex_to_index[v]
            
            weight = euclidean_dist(graph.get_position(u), graph.get_position(v))
            
            if not mst_set[v_index] and weight < key[v_index]:
                parent[v_index] = u_index
                key[v_index] = weight
    
    # Now finding the path from start to destination in the MST using BFS
    queue = [start_vertex]
    visited = {start_vertex: True}
    parent_map = {start_vertex: None}
    
    while queue:
        current = queue.pop(0)
        
        if current == dest_vertex:
            break
        
        for neighbor in mst_edges[current]:
            if neighbor not in visited:
                visited[neighbor] = True
                parent_map[neighbor] = current
                queue.append(neighbor)
    
    path = []
    current = dest_vertex
    
    if current not in parent_map:
        return point_to_point(start, dest)
    
    while current is not None:
        path.append(graph.get_position(current))
        current = parent_map.get(current)
    
    # reversing the path to go from start to destination
    path.reverse()
    final_path = [start] + path[1:-1] + [dest]
    
    return final_path

def search(algorithm, start, dest):
    if (algorithm == 'p2p'):
        return point_to_point(start, dest)
    elif(algorithm == 'fly'):
        return fly(start, dest)
    elif(algorithm == 'random'):
        return random_graph(start, dest)
    elif(algorithm == 'bfs'):
        return breadth_first(start, dest)
    elif(algorithm == 'bfsh'):
        return breadth_first_hub(start, dest)
    elif(algorithm == 'dfs'):
        return depth_first(start, dest)
    elif(algorithm == 'dfsb'):
        return depth_first_best(start, dest)
    elif(algorithm == 'bf'):
        return bellman_ford(start, dest)
    elif(algorithm == 'bfn'):
        return bellman_ford_negative(start, dest)
    elif(algorithm == 'dijkstra'):
        return dijkstra(start, dest)
    elif(algorithm == 'astar'):
        return astar(start, dest)
    elif(algorithm == 'mst'):
        return min_spanning_tree(start, dest)

    return []

if __name__ == "__main__":
    pass