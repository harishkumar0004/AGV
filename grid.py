import networkx as nx

GRID = 0.5
ROWS = 7
COLS = 6

def generate_nodes():
    """
    Generate nodes dictionary for visualization.
    Returns dict mapping node_id -> (x, y) for display.
    """
    nodes = {}
    node_id = 1
    for r in range(ROWS):
        for c in range(COLS):
            nodes[node_id] = (r * GRID, c * GRID)
            node_id += 1
    return nodes

def node_id_to_grid_coord(node_id, cols=COLS):
    """Convert 1-based node_id to 0-based (row, col)."""
    node_id_0 = node_id - 1
    row = node_id_0 // cols
    col = node_id_0 % cols
    return (row, col)

def grid_coord_to_node_id(row, col, cols=COLS):
    """Convert 0-based (row, col) to 1-based node_id."""
    return row * cols + col + 1

def create_networkx_grid(rows=ROWS, cols=COLS, obstacles=None, weight=1):
    """
    Create a NetworkX grid graph.

    Parameters:
        rows: number of rows
        cols: number of columns
        obstacles: set of (row, col) tuples to exclude
        weight: default edge weight

    Returns:
        G: NetworkX Graph with nodes as (row, col) tuples
    """
    # Manually build grid to ensure correct dimensions
    G = nx.Graph()
    
    # Add all nodes
    for r in range(rows):
        for c in range(cols):
            G.add_node((r, c))
    
    # Add edges (4-neighborhood: up, down, left, right)
    for r in range(rows):
        for c in range(cols):
            # Right neighbor
            if c + 1 < cols:
                G.add_edge((r, c), (r, c + 1), weight=weight)
            # Down neighbor
            if r + 1 < rows:
                G.add_edge((r, c), (r + 1, c), weight=weight)

    # Remove obstacle nodes if specified
    if obstacles:
        for obs in obstacles:
            if G.has_node(obs):
                G.remove_node(obs)

    return G

def get_neighbors(node_id, nodes):
    """Get neighbors for visualization purposes."""
    x, y = nodes[node_id]
    result = []
    for nid, (nx, ny) in nodes.items():
        if abs(nx - x) + abs(ny - y) == GRID:
            result.append(nid)
    return result

def get_all_neighbors_for_visualization(nodes, rows=ROWS, cols=COLS):
    """Precompute all neighbors for visualization."""
    neighbors = {}
    for node_id in nodes:
        neighbors[node_id] = get_neighbors(node_id, nodes)
    return neighbors

def block_node(G, node):
    """Block a node by removing it from the graph."""
    if G.has_node(node):
        G.remove_node(node)
        return True
    return False

def unblock_node(G, node, rows=ROWS, cols=COLS, weight=1):
    """Unblock a node by adding it back with proper edges."""
    if G.has_node(node):
        return False

    r, c = node
    G.add_node(node)

    # Reconnect to neighbors
    for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
        nr, nc = r + dr, c + dc
        neighbor = (nr, nc)
        if 0 <= nr < rows and 0 <= nc < cols and G.has_node(neighbor):
            G.add_edge(node, neighbor, weight=weight)

    return True
