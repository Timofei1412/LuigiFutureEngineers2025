import math, heapq

HEIGHT = 6
WIDTH = 6

def create_adjacency_array(grid):
    """
    Converts an 8x8 grid of tuples into an adjacency list for pathfinding 
    without diagonal movements.
    
    Each grid element is a tuple of (bool height, int weight, int type, int dir)
    - height: True if elevated/blocked
    - weight: Movement cost
    - type: Terrain type
    - dir: Movement direction constraints
    
    Returns: A dictionary where keys are (row, col) tuples and values are lists
             of adjacent nodes with movement costs.
    """
    adjacency = {}
    rows = len(grid)
    cols = len(grid[0]) if rows > 0 else 0
    
    # Only cardinal directions (no diagonals): up, down, left, right
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
    
    for i in range(rows):
        for j in range(cols):
            current_node = (i, j)
            current_cell = grid[i][j]
            adjacency[current_node] = []
            
            for di, dj in directions:
                ni, nj = i + di, j + dj
                
                # Check bounds
                if 0 <= ni < rows and 0 <= nj < cols:
                    neighbor_cell = grid[ni][nj]
                    
                    # Check if movement is allowed
                    if is_movement_allowed(current_cell, neighbor_cell, di, dj):
                        # Cost is the neighbor's weight (or could be average)
                        cost = neighbor_cell[1]  
                        adjacency[current_node].append(((ni, nj), cost))
    
    return adjacency

def is_movement_allowed(current_cell, neighbor_cell, di, dj):
    """
    Determines if movement between cells is allowed based on:
    - Height differences
    - Terrain types
    - Direction constraints
    """
    current_height, _, current_type, current_dir = current_cell
    neighbor_height, _, neighbor_type, neighbor_dir = neighbor_cell
    
    # 1. Check if either cell is blocked (height=True means blocked)
    if current_height or neighbor_height:
        return False
    
    # 2. Check terrain type constraints (example: type 3 is blocked)
    if neighbor_type == 3:  # Adjust based on your type system
        return False
    
    # 3. Check direction constraints (example implementation)
    # Assuming dir is a bitmask or enum defining allowed directions
    # This is a placeholder - implement your actual direction rules
    if current_dir == 1 and di == 1:  # Example: can't move down
        return False
    if current_dir == 2 and dj == -1:  # Example: can't move left
        return False
    
    return True
    
# # # height(ramp 1), cost, tube(red 1, green 2), direction(vertical 1, horozontal 2)
# # inputArray = [[(0, 1, 0, 0), (0, 1, 0, 0), (0, 1, 0, 0), (0, 1, 0, 0), (0, 1, 0, 0), (0, 1, 0, 0), (0, 1, 0, 0), (0, 1, 0, 0),],
# #               [(0, 1, 0, 0), (0, 1, 0, 0), (0, 1, 0, 0), (0, 1, 0, 0), (0, 1, 0, 0), (0, 1, 0, 0), (0, 1, 0, 0), (0, 1, 0, 0),],
# #               [(0, 1, 0, 0), (0, 1, 0, 0), (0, 1, 0, 0), (0, 1, 0, 0), (0, 1, 0, 0), (0, 1, 0, 0), (0, 1, 0, 0), (0, 1, 0, 0),],
# #               [(0, 1, 0, 0), (0, 1, 0, 0), (0, 1, 0, 0), (0, 1, 0, 0), (0, 1, 0, 0), (0, 1, 0, 0), (0, 1, 0, 0), (0, 1, 0, 0),],
# #               [(0, 1, 0, 0), (0, 1, 0, 0), (0, 1, 0, 0), (0, 1, 0, 0), (0, 1, 0, 0), (0, 1, 0, 0), (0, 1, 0, 0), (0, 1, 0, 0),],
# #               [(0, 1, 0, 0), (0, 1, 0, 0), (0, 1, 0, 0), (0, 1, 0, 0), (0, 1, 0, 0), (0, 1, 0, 0), (0, 1, 0, 0), (0, 1, 0, 0),],
# #               [(0, 1, 0, 0), (0, 1, 0, 0), (0, 1, 0, 0), (0, 1, 0, 0), (0, 1, 0, 0), (0, 1, 0, 0), (0, 1, 0, 0), (0, 1, 0, 0),],
# #               [(0, 1, 0, 0), (0, 1, 0, 0), (0, 1, 0, 0), (0, 1, 0, 0), (0, 1, 0, 0), (0, 1, 0, 0), (0, 1, 0, 0), (0, 1, 0, 0),],
# #             ]


# # # 3 red, 3 green
# # pointOfInterest = [
# #     (0, 0),
# #     (0, 0),
# #     (0, 0),
# #     (0, 0),
# #     (0, 0),
# #     (0, 0),
# #     (0, 0),
# # ]



# graph= [
#     [0, 0, 0, 0, 0, 0],
#     [0, 0, 0, 0, 0, 0],
#     [0, 0, 0, 0, 0, 0],
#     [0, 0, 0, 0, 0, 0],
#     [0, 0, 0, 0, 0, 0],
#     [0, 0, 0, 0, 0, 0],
# ]


# manh = [[0, 0, 0, 0, 0, 0],
#        [0, 0, 0, 0, 0, 0],
#        [0, 0, 0, 0, 0, 0],
#        [0, 0, 0, 0, 0, 0],
#        [0, 0, 0, 0, 0, 0],
#        [0, 0, 0, 0, 0, 0],
#        ]

# startPoint = (4, 2)

# for i in range(6):
#     for j in range(6):
#         arr[i][j] = abs(startPoint[0] - i) + abs(startPoint[1] - j)  

# for i in range(6):
#     print(arr[i])
