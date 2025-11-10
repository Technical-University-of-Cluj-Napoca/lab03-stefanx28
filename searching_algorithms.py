from utils import *
from collections import deque
from queue import PriorityQueue
from grid import Grid
from spot import Spot



def reconstruct_path(came_from, current, draw):
    while current in came_from:
        current = came_from[current]
        current.make_path()
        draw()

def bfs(draw: callable, grid: Grid, start: Spot, end: Spot) -> bool:
    """
    Breadth-First Search (BFS) Algorithm.
    Args:
        draw (callable): A function to call to update the Pygame window.
        grid (Grid): The Grid object containing the spots.
        start (Spot): The starting spot.
        end (Spot): The ending spot.
    Returns:
        bool: True if a path is found, False otherwise.
    """
    for row in grid.grid:
        for spot in row:
            spot.update_neighbors(grid.grid)

    queue = [start]
    visited = {start}
    came_from = {}
    
    while queue:
        current = queue.pop(0)

        if current == end:
            reconstruct_path(came_from, end, draw)
            end.make_end()
            start.make_start
            return True

        for adj in current.neighbors:
            if adj not in visited and not adj.is_barrier():
                visited.add(adj)
                came_from[adj] = current
                queue.append(adj)
                adj.make_open()
        
        draw()

        if current != start:
            current.make_closed()

    return False
    pass

def dfs(draw: callable, grid: Grid, start: Spot, end: Spot) -> bool:
    """
    Depdth-First Search (DFS) Algorithm.
    Args:
        draw (callable): A function to call to update the Pygame window.
        grid (Grid): The Grid object containing the spots.
        start (Spot): The starting spot.
        end (Spot): The ending spot.
    Returns:
        bool: True if a path is found, False otherwise.
    """

    for row in grid.grid:
        for spot in row:
            spot.update_neighbors(grid.grid)
    
    stack = [start]
    visited = {start}
    came_from = {}

    while stack:
        current = stack.pop()

        if current == end:
            reconstruct_path(came_from, end, draw)
            end.make_end()
            start.make_start
            return True
        
        for adj in current.neighbors:
            if adj not in visited and not adj.is_barrier():
                visited.add(adj)
                came_from[adj] = current
                stack.append(adj)
                adj.make_open()
        
        draw()

        if current != start:
            current.make_closed()

    return False


    pass

def dls(draw: callable, grid: Grid, start: Spot, end: Spot) -> bool:
    """
    Depdth-First Search (DFS) Algorithm.
    Args:
        draw (callable): A function to call to update the Pygame window.
        grid (Grid): The Grid object containing the spots.
        start (Spot): The starting spot.
        end (Spot): The ending spot.
    Returns:
        bool: True if a path is found, False otherwise.
    """

    for row in grid.grid:
        for spot in row:
            spot.update_neighbors(grid.grid)
    
    stack = [start]
    visited = {start}
    came_from = {}

    while stack:
        current = stack.pop()

        if current == end:
            reconstruct_path(came_from, end, draw)
            end.make_end()
            start.make_start
            return True
        
        for adj in current.neighbors:
            if adj not in visited and not adj.is_barrier():
                visited.add(adj)
                came_from[adj] = current
                stack.append(adj)
                adj.make_open()
        
        if current != start:
            current.make_closed()
        
        draw()

    return False


    pass
def iddfs(draw: callable, grid: Grid, start: Spot, end: Spot, max_depth: int = 50) -> bool:
    for depth in range(max_depth + 1):
        if dls(draw, grid, start, end, depth= depth):
            return True
    return False
    pass

def ucs(draw: callable, grid: Grid, start: Spot, end: Spot) -> bool:
    for row in grid.grid:
        for spot in row:
            spot.update_neighbors(grid.grid)
    
    count = 0
    pq = PriorityQueue()
    pq.put((0, count, start))
    came_from = {}
    g_score = {spot: float('inf') for row in grid.grid for spot in row}
    g_score[start] = 0
    visited = set()
    
    while not pq.empty():
        current = pq.get()[2]

        if current == end:
            reconstruct_path(came_from, end, draw)
            end.make_end()
            start.make_start
            return True

        if current not in visited:
            visited.add(current)

        for adj in current.neighbors:
            temp = g_score[current] + 1
            if temp < g_score[adj]:
                g_score[adj] = temp
                came_from[adj] = current
                count += 1
                pq.put((temp, count, adj))
                adj.make_open()

        if current != start:
            current.make_closed()
        draw()
                
    pass


def dijkstra(draw: callable, grid: Grid, start: Spot, end: Spot) -> bool:
    for row in grid.grid:
        for spot in row:
            spot.update_neighbors(grid.grid)
    
    count = 0
    pq = PriorityQueue()
    pq.put((0, count, start))

    came_from = {}
    distance = {spot: float('inf') for row in grid.grid for spot in row}
    distance[start] = 0
    visited = set()
    open_set = {start}
    
    while not pq.empty():
        current = pq.get()[2]
        open_set.remove(current)

        if current == end:
            reconstruct_path(came_from, end, draw)
            end.make_end()
            start.make_start
            return True

        if current not in visited:
            visited.add(current)

        for adj in current.neighbors:
            temp = distance[current] + 1
            if temp < distance[adj]:
                distance[adj] = temp
                came_from[adj] = current
                count += 1
                pq.put((temp, count, adj))
                open_set.add(adj)
                adj.make_open()

        if current != start:
            current.make_closed()
        draw()

        return False
                
    pass

def h_manhattan_distance(p1: tuple[int, int], p2: tuple[int, int]) -> float:
    """
    Heuristic function for A* algorithm: uses the Manhattan distance between two points.
    Args:
        p1 (tuple[int, int]): The first point (x1, y1).
        p2 (tuple[int, int]): The second point (x2, y2).
    Returns:
        float: The Manhattan distance between p1 and p2.
    """
    x1, y1 = p1
    x2, y2 = p2
    return (abs(x2 - x1) + abs(y2-y1))
    pass

def h_euclidian_distance(p1: tuple[int, int], p2: tuple[int, int]) -> float:
    """
    Heuristic function for A* algorithm: uses the Euclidian distance between two points.
    Args:
        p1 (tuple[int, int]): The first point (x1, y1).
        p2 (tuple[int, int]): The second point (x2, y2).
    Returns:
        float: The Manhattan distance between p1 and p2.
    """

    x1, y1 = p1
    x2, y2 = p2
    return ( (  (x2 - x1)  ** 2 + (y2 - y1) ** 2 ) ** 0.5)
    pass


def astar(draw: callable, grid: Grid, start: Spot, end: Spot) -> bool:
    """
    A* Pathfinding Algorithm.
    Args:
        draw (callable): A function to call to update the Pygame window.
        grid (Grid): The Grid object containing the spots.
        start (Spot): The starting spot.
        end (Spot): The ending spot.
    Returns:
        bool: True if a path is found, False otherwise.
    """
    for row in grid.grid:
        for spot in row:
            spot.update_neighbors(grid.grid)
    count = 0
    open_set = PriorityQueue
    open_set.put((0, count, start))

    came_from = {}
    g_score = {spot: float("inf") for row in grid.grid for spot in row}
    g_score[start] = 0

    f_score = {spot: float("inf") for row in grid.grid for spot in row}
    f_score[start] = h_manhattan_distance(start.get_position(), end.get_position())
    
    open_set_hash = {start}

    while not open_set.empty():
        current = open_set.get()[2]
        open_set_hash.remove(current)

        if current == end:
            reconstruct_path(came_from, end, draw)
            end.make_end()
            start.make_start
            return True
        
        for adj in current.neighbors:
            temp_g = g_score[current] + 1
            if temp_g < g_score[adj]:
                came_from[adj] = current
                g_score[adj] = temp_g + h_manhattan_distance(adj.get_position(), end.get_position())

            if adj not in open_set_hash:
                count += 1
                open_set.put((f_score[adj], count, adj))
                open_set_hash(adj)
                adj.make(open)

            draw()

            if current != start:
                current.make_closed()
    return False

    pass

# and the others algorithms...
# ▢ Depth-Limited Search (DLS)
# ▢ Uninformed Cost Search (UCS)
# ▢ Greedy Search
# ▢ Iterative Deepening Search/Iterative Deepening Depth-First Search (IDS/IDDFS)
# ▢ Iterative Deepening A* (IDA)
# Assume that each edge (graph weight) equalss