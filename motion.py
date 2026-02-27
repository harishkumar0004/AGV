# 0=LEFT, 1=FORWARD, 2=RIGHT
NORTH, EAST, SOUTH, WEST = 0, 1, 2, 3

# Direction mapping for grid coordinates (row, col)
# NORTH = row-1, SOUTH = row+1, WEST = col-1, EAST = col+1
DIR_VECTORS = {
    NORTH: (-1, 0),
    EAST: (0, 1),
    SOUTH: (1, 0),
    WEST: (0, -1)
}

def heading_from_coords(current, next_pos):
    """
    Determine heading direction from current to next position.
    Coordinates are (row, col).
    """
    dr = next_pos[0] - current[0]
    dc = next_pos[1] - current[1]

    if dr < 0: return NORTH
    if dr > 0: return SOUTH
    if dc > 0: return EAST
    if dc < 0: return WEST
    return None

def path_to_commands(path_coords, start_heading=SOUTH):
    """
    Convert path (list of (row, col) coordinates) to robot commands.

    Commands: 0=LEFT, 1=FORWARD, 2=RIGHT
    """
    if len(path_coords) < 2:
        return []

    cmds = []
    h = start_heading

    for i in range(len(path_coords) - 1):
        current = path_coords[i]
        next_pos = path_coords[i + 1]

        d = heading_from_coords(current, next_pos)
        if d is None:
            continue

        turn = (d - h) % 4

        if turn == 1: 
            cmds.append(2)      # RIGHT
        elif turn == 3: 
            cmds.append(0)      # LEFT
        elif turn == 2: 
            cmds += [2, 2]      # 180 degree turn

        cmds.append(1)          # FORWARD
        h = d

    return cmds
