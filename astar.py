import heapq
import networkx as nx
from itertools import islice

def heuristic(a, b):
    """Manhattan distance heuristic for grid coordinates."""
    return abs(a[0]-b[0]) + abs(a[1]-b[1])

def euclidean_heuristic(a, b):
    """Euclidean distance heuristic."""
    return ((a[0]-b[0])**2 + (a[1]-b[1])**2)**0.5

def astar_networkx(G, start, goal, heuristic_fn=None, weight='weight'):
    """
    A* pathfinding using NetworkX.

    Parameters:
        G: NetworkX graph
        start: starting node
        goal: goal node
        heuristic_fn: heuristic function (u, v) -> float
        weight: edge weight attribute name

    Returns:
        path: list of nodes from start to goal
        visited: list of visited nodes during search
    """
    if heuristic_fn is None:
        heuristic_fn = lambda u, v: 0  # Dijkstra behavior

    try:
        path = nx.astar_path(G, start, goal, heuristic=heuristic_fn, weight=weight)
        # NetworkX doesn't expose visited nodes directly, so we return path as visited
        visited = path.copy()
        return path, visited
    except nx.NetworkXNoPath:
        return None, []

# In astar.py - Make sure your k_shortest_paths function uses the actual node IDs
# NOT the coordinates when calling NetworkX functions

def k_shortest_paths(G, source, target, k=5, weight='weight'):
    """
    Find k shortest simple paths from source to target.
    
    Parameters:
    -----------
    G : NetworkX graph
    source : node (should be a (row, col) tuple for grid graphs)
    target : node (should be a (row, col) tuple for grid graphs)
    k : number of paths
    weight : edge weight attribute
    
    Returns:
    --------
    List of paths, where each path is a list of nodes (tuples)
    """
    # Debug: Check if nodes exist
    if not G.has_node(source):
        print(f"❌ Source node {source} not in graph!")
        print(f"   Available nodes sample: {list(G.nodes())[:5]}")
        return []
    
    if not G.has_node(target):
        print(f"❌ Target node {target} not in graph!")
        print(f"   Available nodes sample: {list(G.nodes())[:5]}")
        return []
    
    try:
        paths = list(islice(nx.shortest_simple_paths(G, source, target, weight=weight), k))
        return paths
    except nx.NetworkXNoPath:
        print(f"⚠️  No path exists between {source} and {target}")
        return []
    except nx.NodeNotFound as e:
        print(f"❌ Node error: {e}")
        return []

def get_path_length(G, path, weight='weight'):
    """Calculate total weight of a path."""
    if len(path) < 2:
        return 0
    total = 0
    for i in range(len(path)-1):
        if G.has_edge(path[i], path[i+1]):
            total += G[path[i]][path[i+1]].get(weight, 1)
        else:
            return float('inf')
    return total
