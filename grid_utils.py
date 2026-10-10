"""
Grid helpers — pure Python, NO ROS in this file.

Keeping the grid maths free of ROS is deliberate: it means you can test it with
a plain `python -m unittest` without starting the simulator. Run the tests as you go:

    python -m unittest tests.test_grid_utils -v

What is provided vs what you write
----------------------------------
Provided:   Grid (the data container), Grid.from_msg, Grid.from_ascii, Grid.to_ascii,
            in_bounds, index, get, is_occupied, copy, path_length_m
You write:  Grid.world_to_cell, Grid.cell_to_world, inflate
"""

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

from sim.grid_values import UNKNOWN, FREE, OCCUPIED

Cell = Tuple[int, int]        # (row, col)  — row first!
Point = Tuple[float, float]   # (x, y) in world metres — x first!


@dataclass
class Grid:
    """
    A 2-D occupancy grid. Same layout as GridSnapshot (read the docstring in
    sim/messages.py carefully — everything below depends on it).

        width, height        number of columns / rows
        resolution           metres per cell
        origin_x, origin_y   world position of the LOWER-LEFT CORNER of cell (0, 0)
        occupancy            flat, row-major: occupancy[row * width + col]
                             UNKNOWN (-1), FREE (0) or OCCUPIED (100)
        confidence           same layout, 0.0 – 1.0 (may be None; you can ignore it in Part 1)
    """
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    occupancy: List[int]
    confidence: Optional[List[float]] = None

    # ── provided helpers ────────────────────────────────────────────────────
    @classmethod
    def from_msg(cls, msg) -> "Grid":
        """Build a Grid from a GridSnapshot message (copies the lists)."""
        return cls(msg.width, msg.height, msg.resolution, msg.origin_x, msg.origin_y,
                   list(msg.occupancy), list(msg.confidence))

    @classmethod
    def from_ascii(cls, rows: Sequence[str], resolution: float = 1.0,
                   origin: Point = (0.0, 0.0)) -> "Grid":
        """
        Build a small grid from text. Handy for tests and for debugging on paper.

            '#' occupied    '.' free    '?' unknown

        The FIRST line is the TOP of the map (highest row), so what you type is
        what you'd see if you drew the map — but row 0 is the LAST line.
        """
        rows = list(rows)[::-1]
        ch = {"#": OCCUPIED, ".": FREE, "?": UNKNOWN}
        occ = [ch[c] for row in rows for c in row]
        return cls(len(rows[0]), len(rows), resolution, origin[0], origin[1], occ,
                   [0.0 if v == UNKNOWN else 1.0 for v in occ])

    def to_ascii(self) -> str:
        """Inverse of from_ascii — print a grid to see what you're working with."""
        ch = {OCCUPIED: "#", FREE: ".", UNKNOWN: "?"}
        return "\n".join(
            "".join(ch[self.occupancy[r * self.width + c]] for c in range(self.width))
            for r in range(self.height - 1, -1, -1)
        )

    def in_bounds(self, row: int, col: int) -> bool:
        return 0 <= row < self.height and 0 <= col < self.width

    def index(self, row: int, col: int) -> int:
        """Position of cell (row, col) in the flat occupancy list."""
        return row * self.width + col

    def get(self, row: int, col: int) -> int:
        return self.occupancy[row * self.width + col]

    def is_occupied(self, row: int, col: int) -> bool:
        return self.occupancy[row * self.width + col] == OCCUPIED

    def copy(self) -> "Grid":
        return Grid(self.width, self.height, self.resolution, self.origin_x, self.origin_y,
                    list(self.occupancy), None if self.confidence is None else list(self.confidence))

    # ── TODO: you write these two ───────────────────────────────────────────
    def world_to_cell(self, x: float, y: float) -> Cell:
        """
        Which cell contains the world point (x, y)?  Returns (row, col).

        Things to get right (the tests check each of them):
          * the return order is (row, col) but the arguments are (x, y)
          * (origin_x, origin_y) is the lower-left CORNER of cell (0, 0)
          * a point exactly on a cell's lower/left edge belongs to that cell
          * a point just outside the grid should give an out-of-range cell
            (e.g. row -1), not row 0 — think about what int() does to -0.3
        The result may lie outside the grid; callers use in_bounds().
        """
        col = math.floor((x - self.origin_x) / self.resolution)
        row = math.floor((y - self.origin_y) / self.resolution)
        return row, col

    def cell_to_world(self, row: int, col: int) -> Point:
        """
        World coordinates (x, y) of the CENTRE of cell (row, col).

        Paths are sequences of world points, so this is what turns the cells your
        search returns into something the rover can drive to.
        """
        x = self.origin_x + (col + 0.5) * self.resolution
        y = self.origin_y + (row + 0.5) * self.resolution
        return x, y


def path_length_m(points_xy: Sequence[Point]) -> float:
    """Total length in metres of a polyline of (x, y) world points."""
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(points_xy, points_xy[1:]))


# ── TODO: you write this one ────────────────────────────────────────────────
def inflate(grid: Grid, radius_m: float) -> Grid:
    """
    Return a NEW grid in which every cell within `radius_m` metres of an OCCUPIED
    cell is also marked OCCUPIED. (Don't modify the grid you were given.)

    Why: the planner treats the rover as a single point, but the real rover is a
    disc of radius `robot_radius_m`. A path that runs along the very edge of an
    obstacle is fine for a point and impossible for a disc. Growing every obstacle
    by the rover's radius lets you keep planning for a point.

    Notes:
      * radius_m is in METRES, cells are `grid.resolution` metres — convert.
      * Obstacles near the edge of the grid must not cause an IndexError.
      * Cells that are UNKNOWN or FREE may become OCCUPIED; OCCUPIED never becomes FREE.
      * Distance can be measured between cell centres. Whether you use a square or
        a circular neighbourhood is up to you — the tests accept either as long as
        the cells directly next to an obstacle (up/down/left/right) are covered
        for radius >= one cell.
    """
    if radius_m < 0:
        raise ValueError("radius_m must be non-negative")
    if grid.resolution <= 0:
        raise ValueError("grid resolution must be positive")

    result = grid.copy()
    if radius_m == 0:
        return result

    radius_cells = math.ceil(radius_m / grid.resolution)
    offsets = [
        (dr, dc)
        for dr in range(-radius_cells, radius_cells + 1)
        for dc in range(-radius_cells, radius_cells + 1)
        if math.hypot(dr, dc) * grid.resolution <= radius_m + 1e-12
    ]
    obstacles = [
        (row, col)
        for row in range(grid.height)
        for col in range(grid.width)
        if grid.is_occupied(row, col)
    ]
    for row, col in obstacles:
        for dr, dc in offsets:
            rr, cc = row + dr, col + dc
            if result.in_bounds(rr, cc):
                result.occupancy[result.index(rr, cc)] = OCCUPIED

    return result
