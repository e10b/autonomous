"""
A* search on a Grid — pure Python, NO ROS in this file.

    python -m unittest tests.test_search -v

Movement rules (fixed, so the tests can check them)
---------------------------------------------------
* 8-connected: a cell has up to 8 neighbours (up, down, left, right and the 4 diagonals).
* Cost of a move: 1.0 for up/down/left/right, sqrt(2) for a diagonal. (Cost is in
  cell-widths, not metres — multiply by grid.resolution if you need metres.)
* No corner cutting: a diagonal move is only allowed if BOTH of the two cells it
  squeezes between are passable. (A real rover cannot slip between two obstacles
  that touch at a corner.)
* Cells outside the grid don't exist.
"""

import heapq
import math
from itertools import count
from typing import Dict, List, Optional

from grid_utils import Grid, Cell
from sim.grid_values import OCCUPIED, UNKNOWN


def heuristic(a: Cell, b: Cell) -> float:
    """
    Estimate of the cost to travel from cell `a` to cell `b` on an EMPTY grid
    under the movement rules above.

    Choose and justify your choice in the README. The one hard requirement:
    it must NEVER OVERESTIMATE the true cost (that's what "admissible" means, and
    A* is only guaranteed to return the shortest path if it holds). The test
    checks this, and it is worth understanding why some very common heuristics
    fail it on an 8-connected grid.
    """
    dr = abs(a[0] - b[0])
    dc = abs(a[1] - b[1])
    diagonal = min(dr, dc)
    straight = max(dr, dc) - diagonal
    return math.sqrt(2.0) * diagonal + straight


def astar(grid: Grid, start: Cell, goal: Cell, unknown_is_free: bool = False) -> Optional[List[Cell]]:
    """
    Find the cheapest path from `start` to `goal`.

    Args:
        grid             the grid to search. OCCUPIED cells can't be entered.
        start, goal      (row, col) cells.
        unknown_is_free  how to treat UNKNOWN cells: True = passable, False = blocked.
                         Which is right for a rover that has only seen a small
                         circle of the world so far? Look at the very first tick
                         of a run before you decide.

    Returns:
        The path as a list of (row, col) cells, INCLUDING both start and goal, or
        None if there is no path. If start == goal, returns [start].

    Things to think about (the tests poke at each):
      * What if the goal is occupied?
      * The rover might already be standing on a cell that your obstacle
        inflation marked as blocked. Should search refuse to leave it?
      * Speed: a 100 x 100 grid must finish in well under a second. What data
        structure makes "give me the cheapest open node" fast?
    """
    if not grid.in_bounds(*start) or not grid.in_bounds(*goal):
        return None

    def passable(cell: Cell) -> bool:
        value = grid.get(*cell)
        return value != OCCUPIED and (unknown_is_free or value != UNKNOWN)

    # A rover can be caught inside an inflated area and must be allowed to
    # leave it, but entering a blocked goal can never produce a valid route.
    if goal != start and not passable(goal):
        return None
    if start == goal:
        return [start]

    directions = (
        (-1, -1, math.sqrt(2.0)), (-1, 0, 1.0), (-1, 1, math.sqrt(2.0)),
        (0, -1, 1.0),                              (0, 1, 1.0),
        (1, -1, math.sqrt(2.0)),  (1, 0, 1.0),  (1, 1, math.sqrt(2.0)),
    )
    serial = count()
    frontier = [(heuristic(start, goal), next(serial), start)]
    came_from: Dict[Cell, Cell] = {}
    best_cost = {start: 0.0}

    while frontier:
        priority, _, current = heapq.heappop(frontier)
        current_cost = best_cost[current]
        if priority > current_cost + heuristic(current, goal) + 1e-12:
            continue
        if current == goal:
            path = [current]
            while current != start:
                current = came_from[current]
                path.append(current)
            path.reverse()
            return path

        for dr, dc, move_cost in directions:
            neighbour = current[0] + dr, current[1] + dc
            if not grid.in_bounds(*neighbour) or not passable(neighbour):
                continue
            if dr and dc:
                side_a = current[0] + dr, current[1]
                side_b = current[0], current[1] + dc
                if not passable(side_a) or not passable(side_b):
                    continue
            candidate = current_cost + move_cost
            if candidate + 1e-12 >= best_cost.get(neighbour, float("inf")):
                continue
            best_cost[neighbour] = candidate
            came_from[neighbour] = current
            heapq.heappush(
                frontier,
                (candidate + heuristic(neighbour, goal), next(serial), neighbour),
            )
    return None
