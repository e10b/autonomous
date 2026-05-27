"""
Global Map Manager for Autonomous Navigation

This module provides a memory-efficient, sliding-window probabilistic occupancy grid
for persistent obstacle tracking and global pathfinding. It keeps the robot safely
padded in a bounds-checked arena, gracefully "rolling" the memory matrix when the
robot approaches an edge. This prevents infinite memory growth while ensuring the
robot never loses track of obstacles in its immediate vicinity.

**Coordinate frame:** all (x, y) inputs and outputs are in the **odom frame**
(from `/autonomous/localization/odometry/local`). The map origin is placed at the
robot's odom position on the first call to `initialize_origin`. Theta* waypoints
returned by `get_path` are therefore also in odom frame and can be passed directly
to `AlignAndMove` for obstacle-avoidance path following.

Class Overview:
    GlobalMapManager: The primary class maintaining the continuous 2D numpy grid.

Function Overview:
    - initialize_origin: Centers the map origin on the robot at startup (odom frame).
    - check_and_shift: Enforces boundary padding limits. If the robot strays too close
      to the map's edge, it shifts the entire array and origin to keep it centered.
    - update_map: Projects a short-range local grid into the global frame and applies
      clamped probabilistic scoring (0=free, 100=obstacle, 50=unknown) to filter noise.
    - clip_target_to_bounds: Ray-traces a vector to a goal and clips it to the exact
      edge of the current loaded map boundary.
    - get_path: Executes the Theta* algorithm across the global map (with Minkowski
      inflation for robot radius) to return smooth, any-angle odom-frame waypoints.
"""
import numpy as np
import logging
import math
import heapq

class GlobalMapManager:
    def __init__(self, map_size_m=60.0, overlap_m=5.0, shift_m=30.0, cell_size_m=0.10,
                 free_space_decay=0, obstacle_confidence_increase=100, obstacle_threshold=60,
                 logger=None):
        """
        Initializes the Global Map Manager.
        The map is a large continuous sliding window grid.
        When the robot approaches the overlap margin, the map shifts by shift_m.

        Args:
            map_size_m (float): Total continuous size of the square map canvas in meters (default: 60.0m).
            overlap_m (float): The boundary safety margin in meters. If the robot crosses into this
                               margin from the edge, a shift is triggered (default: 5.0m).
            shift_m (float): The exact distance in meters the map will translate when a shift is
                             triggered, keeping the robot centered (default: 30.0m).
            cell_size_m (float): The physical resolution of a single grid cell in meters (default: 0.10m).
            free_space_decay (int): Points subtracted from a cell per free-space observation (default: 0).
                                    Set to 5 on the real rover so transient detections fade over time.
            obstacle_confidence_increase (int): Points added to a cell per obstacle observation (default: 100).
            obstacle_threshold (int): Grid value above which a cell is treated as a solid obstacle (default: 60).
            logger (logging.Logger): Optional logger instance for debug and transition tracking.
        """
        self.logger = logger or logging.getLogger("GlobalMapManager")
        
        # Adjustable parameters
        self.map_size_m = map_size_m    # Total size of the map (e.g., 60.0m)
        self.overlap_m = overlap_m      # Shift trigger boundary margin (e.g., 5.0m)
        self.shift_m = shift_m          # Distance to shift when hitting boundary (e.g., 30.0m)
        self.cell_size_m = cell_size_m  # Size of a single grid cell (e.g., 0.10m)
        self.free_space_decay = free_space_decay                        # Points removed per free observation (0 = no decay)
        self.obstacle_confidence_increase = obstacle_confidence_increase # Points added per obstacle observation
        self.obstacle_threshold = obstacle_threshold                     # Value above which a cell is an obstacle
        
        # Derived grid dimensions
        self.grid_size = int(self.map_size_m / self.cell_size_m)
        
        # Initialize grid with 50 (unknown). 0 = free, 100 = obstacle.
        self.grid = np.full((self.grid_size, self.grid_size), 50, dtype=np.uint8)
        
        # Global origin (top-left corner) in odom-frame coordinates.
        # Start as None, will be set on the first position update.
        self.global_origin_x = None
        self.global_origin_y = None
        
        self.logger.info(f"Initialized {self.map_size_m}m x {self.map_size_m}m Global Map Manager (cells: {self.cell_size_m}m)")

    def initialize_origin(self, robot_x, robot_y):
        """
        Sets the initial global origin so the robot starts exactly in the
        center of the total map.
        """
        if self.global_origin_x is None or self.global_origin_y is None:
            # Place robot at the dead center of the map
            half_map = self.map_size_m / 2.0
            self.global_origin_x = robot_x - half_map
            self.global_origin_y = robot_y - half_map
            self.logger.info(f"Set initial global origin to ({self.global_origin_x:.2f}, {self.global_origin_y:.2f})")
            return True
        return False

    # Might need to change based on the data format of the local grid from obstacle_detection. Assuming it's a 20x20 numpy array with 1s and 0s.
    def update_map(self, local_grid, robot_x, robot_y, robot_yaw):
        """
        Updates the global occupancy grid using the local 2x2m grid.
        
        args:
            local_grid: 2D numpy array (20x20) where 1=obstacle, 0=free.
            robot_x, robot_y: Robot position in odom frame (meters).
            robot_yaw: Math heading in radians (0=East, CCW+).
        """
        # Ensure we have an origin initialized
        self.initialize_origin(robot_x, robot_y)
        
        # Check boundaries and shift grid if necessary BEFORE updating
        self.check_and_shift(robot_x, robot_y)
        
        # The local grid from obstacle_detection is usually 20x20 cells
        cells_x, cells_z = local_grid.shape
        
        # 1. Create coordinate grids for the 20x20 local mask
        # In obstacle_detection: xi is left/right, zi is forward
        x_idx = np.arange(cells_x)
        z_idx = np.arange(cells_z)
        xx, zz = np.meshgrid(x_idx, z_idx, indexing='ij')
        
        # 2. Convert indices back to local meters relative to camera
        local_x_m = (xx - (cells_x / 2.0)) * self.cell_size_m
        local_z_m = zz * self.cell_size_m
        
        # 3. Rotate by robot yaw and translate to global frame
        # Standard convention: Forward is local X, Left is local Y.
        # Camera x-axis points RIGHT, so negate to convert to the left-hand convention.
        rover_local_x = local_z_m
        rover_local_y = -local_x_m
        
        cos_y = np.cos(robot_yaw)
        sin_y = np.sin(robot_yaw)
        
        global_x_m = robot_x + (rover_local_x * cos_y - rover_local_y * sin_y)
        global_y_m = robot_y + (rover_local_x * sin_y + rover_local_y * cos_y)
        
        # 4. Map global meters to our sliding array's indices
        grid_idx_x = ((global_x_m - self.global_origin_x) / self.cell_size_m).astype(np.int32)
        grid_idx_y = ((global_y_m - self.global_origin_y) / self.cell_size_m).astype(np.int32)
        
        # 5. Filter bounds (though mathematically padded, always good to be safe)
        valid_mask = (grid_idx_x >= 0) & (grid_idx_x < self.grid_size) & \
                     (grid_idx_y >= 0) & (grid_idx_y < self.grid_size)
                     
        valid_idx_x = grid_idx_x[valid_mask]
        valid_idx_y = grid_idx_y[valid_mask]
        valid_local_vals = local_grid[valid_mask]
        
        # 6. Apply Probabilistic Updating (clamped between 0 and 100)
        updates = np.full(valid_local_vals.shape, -self.free_space_decay, dtype=np.int16)
        updates[valid_local_vals > 0] = self.obstacle_confidence_increase
        
        # Upcast the entire grid to prevent overflow during accumulation
        temp_grid = self.grid.astype(np.int16)
        
        # Accumulate updates: if multiple local cells map to the same global cell, they add up.
        # The +100 for an obstacle will easily overpower several -5s from overlapping free space.
        np.add.at(temp_grid, (valid_idx_x, valid_idx_y), updates)
        
        # Clip to 0-100 range and save back to the global uint8 grid
        self.grid = np.clip(temp_grid, 0, 100).astype(np.uint8)

    def check_and_shift(self, robot_x, robot_y):
        """
        Monitors the distance between the robot and the map boundaries.
        If the robot encroaches on the `overlap_m` boundary, the map shifts by `shift_m` 
        to ensure the robot is always safely padded with context in all directions.
        """
        if self.global_origin_x is None or self.global_origin_y is None:
            return

        relative_x = robot_x - self.global_origin_x
        relative_y = robot_y - self.global_origin_y

        cells_to_shift = int(self.shift_m / self.cell_size_m)

        # -----------------------------
        # X-Axis Bounds Check (North/South)
        # -----------------------------
        if relative_x < self.overlap_m:
            # Too close to top/back edge, shift grid contents down
            self.grid = np.roll(self.grid, shift=cells_to_shift, axis=0)
            self.grid[:cells_to_shift, :] = 50  # Grey out the new blank territory
            self.global_origin_x -= self.shift_m
            self.logger.info(f"Shifted Global Map -X. New origin: {self.global_origin_x:.2f}")

        elif relative_x > (self.map_size_m - self.overlap_m):
            # Too close to bottom/front edge, shift grid contents up
            self.grid = np.roll(self.grid, shift=-cells_to_shift, axis=0)
            self.grid[-cells_to_shift:, :] = 50
            self.global_origin_x += self.shift_m
            self.logger.info(f"Shifted Global Map +X. New origin: {self.global_origin_x:.2f}")

        # -----------------------------
        # Y-Axis Bounds Check (East/West)
        # -----------------------------
        if relative_y < self.overlap_m:
            # Too close to left edge, shift grid contents right
            self.grid = np.roll(self.grid, shift=cells_to_shift, axis=1)
            self.grid[:, :cells_to_shift] = 50
            self.global_origin_y -= self.shift_m
            self.logger.info(f"Shifted Global Map -Y. New origin: {self.global_origin_y:.2f}")

        elif relative_y > (self.map_size_m - self.overlap_m):
            # Too close to right edge, shift grid contents left
            self.grid = np.roll(self.grid, shift=-cells_to_shift, axis=1)
            self.grid[:, -cells_to_shift:] = 50
            self.global_origin_y += self.shift_m
            self.logger.info(f"Shifted Global Map +Y. New origin: {self.global_origin_y:.2f}")

    def clip_target_to_bounds(self, robot_x, robot_y, target_a, target_b, target_type='NED', gps_converter=None, margin_m=1.0):
        """
        Draws a vector from the robot to the target. If the target is outside the current
        global map bounds, it finds the exact intersection point with the map edge (minus a
        small safety margin) and returns it. If the target is inside, it returns the target.

        args:
            robot_x, robot_y: Robot position in odom frame (meters).
            target_a, target_b: Target position in odom frame (X, Y). The 'lat_lon' path
                is kept for backward compatibility but is not used in normal operation.
            target_type: 'NED' (default, odom-frame XY) or 'lat_lon' (legacy).
            gps_converter: An instance of GPS_2_NED (required only if target_type is 'lat_lon').
        """
        if self.global_origin_x is None or self.global_origin_y is None:
            if target_type == 'NED':
                return target_a, target_b
            elif gps_converter:
                return gps_converter.gps_to_xy(target_a, target_b)
            return robot_x, robot_y

        # Convert lat/lon to odom-frame XY if needed (legacy path, not used normally)
        if target_type == 'lat_lon':
            if gps_converter is None:
                self.logger.error("gps_converter must be provided to convert lat_lon targets!")
                return robot_x, robot_y
            target_x, target_y = gps_converter.gps_to_xy(target_a, target_b)
        else:
            target_x, target_y = target_a, target_b

        min_x = self.global_origin_x + margin_m
        max_x = self.global_origin_x + self.map_size_m - margin_m
        min_y = self.global_origin_y + margin_m
        max_y = self.global_origin_y + self.map_size_m - margin_m
        
        # 1. If target is already inside the bounding box, return it directly
        if min_x <= target_x <= max_x and min_y <= target_y <= max_y:
            return target_x, target_y
            
        dx = target_x - robot_x
        dy = target_y - robot_y
        
        # 2. Prevent division by zero and find Ray-Box intersection "time" (t)
        t_x = float('inf')
        if dx > 0:
            t_x = (max_x - robot_x) / dx
        elif dx < 0:
            t_x = (min_x - robot_x) / dx
            
        t_y = float('inf')
        if dy > 0:
            t_y = (max_y - robot_y) / dy
        elif dy < 0:
            t_y = (min_y - robot_y) / dy
            
        # 3. The true intersection is the smallest positive t
        t_int = min(t_x, t_y)
        
        # 4. Compute intersection point along the vector
        intersect_x = robot_x + t_int * dx
        intersect_y = robot_y + t_int * dy
        
        return intersect_x, intersect_y

    def nearest_free_point(self, x, y, robot_radius_m=1.0):
        """Return the nearest odom-frame point to (x, y) that is outside every obstacle and
        inflation zone. Returns (x, y) unchanged if the point is already free."""
        if self.global_origin_x is None:
            return (x, y)

        binary_grid = (self.grid > self.obstacle_threshold).astype(np.uint8)
        radius_cells = max(1, int(robot_radius_m / self.cell_size_m))
        inflated = self._minkowski_sum_grid(binary_grid, radius_cells)

        sc = max(0, min(self.grid_size - 1, int((x - self.global_origin_x) / self.cell_size_m)))
        sr = max(0, min(self.grid_size - 1, int((y - self.global_origin_y) / self.cell_size_m)))

        if inflated[sc, sr] == 0:
            return (x, y)

        from collections import deque
        queue = deque([(sc, sr)])
        visited = {(sc, sr)}
        while queue:
            c, r = queue.popleft()
            for dc in (-1, 0, 1):
                for dr in (-1, 0, 1):
                    if dc == 0 and dr == 0:
                        continue
                    nc, nr = c + dc, r + dr
                    if 0 <= nc < self.grid_size and 0 <= nr < self.grid_size and (nc, nr) not in visited:
                        visited.add((nc, nr))
                        if inflated[nc, nr] == 0:
                            return (self.global_origin_x + nc * self.cell_size_m,
                                    self.global_origin_y + nr * self.cell_size_m)
                        queue.append((nc, nr))
        return (x, y)

    def get_path(self, robot_x, robot_y, target_a, target_b, target_type='NED', gps_converter=None, robot_radius_m=1.0):
        """
        Calculates a path from the robot to a target coordinate using Theta*.
        All inputs and outputs are in odom frame.
        Returns a list of (x, y) odom-frame waypoints.
        """
        if self.global_origin_x is None or self.global_origin_y is None:
            return []

        # 0. Automatically clip target to map boundaries along the sightline vector (and handle conversions)
        target_x, target_y = self.clip_target_to_bounds(
            robot_x, robot_y, target_a, target_b, target_type=target_type, gps_converter=gps_converter
        )

        # 1. Convert global coordinates to grid indices
        start_c = int((robot_x - self.global_origin_x) / self.cell_size_m)
        start_r = int((robot_y - self.global_origin_y) / self.cell_size_m)
        goal_c = int((target_x - self.global_origin_x) / self.cell_size_m)
        goal_r = int((target_y - self.global_origin_y) / self.cell_size_m)

        # Clip to bounds just in case the target is technically outside the map 
        # (Theta* will navigate to the boundary)
        start_c = max(0, min(self.grid_size - 1, start_c))
        start_r = max(0, min(self.grid_size - 1, start_r))
        goal_c = max(0, min(self.grid_size - 1, goal_c))
        goal_r = max(0, min(self.grid_size - 1, goal_r))

        start = (start_c, start_r)
        goal = (goal_c, goal_r)

        # 2. Create binary grid and inflate
        binary_grid = (self.grid > self.obstacle_threshold).astype(np.uint8)
        
        radius_cells = max(1, int(robot_radius_m / self.cell_size_m))
        inflated = self._minkowski_sum_grid(binary_grid, radius_cells)

        # Ensure start and goal are free from their OWN inflation zones so the algorithm doesn't fail instantly
        def clear_radius(grid, center, radius):
            cx, cy = center
            for i in range(-radius, radius + 1):
                for j in range(-radius, radius + 1):
                    if i*i + j*j <= radius*radius:
                        if 0 <= cx+i < self.grid_size and 0 <= cy+j < self.grid_size:
                            grid[cx+i, cy+j] = 0

        clear_radius(inflated, start, radius_cells)
        clear_radius(inflated, goal, radius_cells)

        # 3. Pathfinding
        path_idx = self._theta_star(inflated, start, goal)
        if not path_idx:
            self.logger.warning("Theta* failed to find a path to the target!")
            return []

        # 4. Convert indices back to odom-frame coordinate waypoints
        path_meters = []
        for (c, r) in path_idx:
            px = self.global_origin_x + (c * self.cell_size_m)
            py = self.global_origin_y + (r * self.cell_size_m)
            path_meters.append((px, py))

        return path_meters

    def _minkowski_sum_grid(self, grid: np.ndarray, radius: int) -> np.ndarray:
        if radius <= 0:
            return grid.copy()

        summed = grid.copy()
        r2 = radius * radius
        gx, gy = grid.shape

        occupied = np.argwhere(grid == 1)
        for (i, j) in occupied:
            i0 = max(0, i - radius)
            i1 = min(gx - 1, i + radius)
            j0 = max(0, j - radius)
            j1 = min(gy - 1, j + radius)

            for k in range(i0, i1 + 1):
                dx = k - i
                dx2 = dx * dx
                for inner in range(j0, j1 + 1):
                    dy_ = inner - j
                    if dx2 + dy_ * dy_ <= r2:
                        summed[k, inner] = 1

        return summed

    def _line_of_sight(self, grid, a, b):
        x0, y0 = a
        x1, y1 = b
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x1 > x0 else -1
        sy = 1 if y1 > y0 else -1
        err = dx - dy
        
        x, y = x0, y0
        while True:
            if grid[x, y] == 1:
                return False
            if (x, y) == (x1, y1):
                return True
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy

    def _euclidean_heuristic(self, a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def _get_neighbors(self, grid, node):
        x, y = node
        neighbors = []
        # Uses full 8-neighborhood for diagonal path smoothing
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                nx, ny = x + dx, y + dy
                if 0 <= nx < grid.shape[0] and 0 <= ny < grid.shape[1]:
                    if grid[nx, ny] != 1:
                        neighbors.append((nx, ny))
        return neighbors

    def _theta_star(self, grid, start, goal):
        open_set = []
        heapq.heappush(open_set, (0, start))
        g = {start: 0}
        parent = {start: start}
        visited = set()

        while open_set:
            _, current = heapq.heappop(open_set)

            if current == goal:
                path = [current]
                while parent[current] != current:
                    current = parent[current]
                    path.append(current)
                path.reverse()
                return path

            if current in visited:
                continue
            visited.add(current)

            for neighbor in self._get_neighbors(grid, current):
                if neighbor in visited:
                    continue

                if self._line_of_sight(grid, parent[current], neighbor):
                    pc = parent[current]
                    tentative_distance = g[pc] + self._euclidean_heuristic(pc, neighbor)
                    new_parent = pc
                else:
                    tentative_distance = g[current] + self._euclidean_heuristic(current, neighbor)
                    new_parent = current

                if neighbor not in g or tentative_distance < g[neighbor]:
                    g[neighbor] = tentative_distance
                    parent[neighbor] = new_parent
                    f = tentative_distance + self._euclidean_heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f, neighbor))

        return None

    def print_ascii_map(self, robot_x, robot_y, window=40, robot_radius_m=1.0, path_waypoints=None, goal=None):
        """Writes a localized ASCII representation of the grid to a file for external viewing."""
        if self.global_origin_x is None or self.global_origin_y is None:
            print("DEBUG: Global origin not set, cannot print map.")
            return
        else:
            print("global origin:", self.global_origin_x, self.global_origin_y)

        robot_c = int((robot_x - self.global_origin_x) / self.cell_size_m)
        robot_r = int((robot_y - self.global_origin_y) / self.cell_size_m)

        goal_c = goal_r = None
        if goal is not None:
            goal_c = int((goal[0] - self.global_origin_x) / self.cell_size_m)
            goal_r = int((goal[1] - self.global_origin_y) / self.cell_size_m)

        min_c = max(0, robot_c - window // 2)
        max_c = min(self.grid_size, robot_c + window // 2)
        min_r = max(0, robot_r - window // 2)
        max_r = min(self.grid_size, robot_r + window // 2)

        # Compute inflation on the window + radius padding to avoid edge artifacts
        radius_cells = max(1, int(robot_radius_m / self.cell_size_m))
        pad_min_c = max(0, min_c - radius_cells)
        pad_max_c = min(self.grid_size, max_c + radius_cells)
        pad_min_r = max(0, min_r - radius_cells)
        pad_max_r = min(self.grid_size, max_r + radius_cells)

        window_grid = self.grid[pad_min_c:pad_max_c, pad_min_r:pad_max_r]
        binary_window = (window_grid > self.obstacle_threshold).astype(np.uint8)
        inflated_window = self._minkowski_sum_grid(binary_window, radius_cells)

        # Build path cell dict {(c, r): char} using Bresenham's line between waypoints.
        # Screen layout: c increases rightward (= east/+x), r increases upward (= north/+y).
        # Character reflects the segment's angle on screen.
        path_cells = {}
        if path_waypoints and len(path_waypoints) >= 2:
            def _path_char(dc, dr):
                if abs(dr) < abs(dc) * 0.4:
                    return '_'   # mostly horizontal on screen (north/south)
                if abs(dc) < abs(dr) * 0.4:
                    return '|'   # mostly vertical on screen (east/west)
                return '\\' if (dc >= 0) == (dr >= 0) else '/'

            pts = [(int((wx - self.global_origin_x) / self.cell_size_m),
                    int((wy - self.global_origin_y) / self.cell_size_m))
                   for wx, wy in path_waypoints]

            for i in range(len(pts) - 1):
                c0, r0 = pts[i]
                c1, r1 = pts[i + 1]
                char = _path_char(c1 - c0, r1 - r0)
                dc = abs(c1 - c0)
                dr = abs(r1 - r0)
                sc = 1 if c1 > c0 else -1
                sr = 1 if r1 > r0 else -1
                err = dc - dr
                c, r = c0, r0
                while True:
                    path_cells[(c, r)] = char
                    if c == c1 and r == r1:
                        break
                    e2 = 2 * err
                    if e2 > -dr:
                        err -= dr
                        c += sc
                    if e2 < dc:
                        err += dc
                        r += sr

        out_str = f"--- Center Window ({window}x{window} cells) ---\n"
        out_str += "Legend: [R]=Rover | [G]=Goal | [##]=Obstacle | [++]=Inflated | [..]=Free | [  ]=Unknown | [_/\\|]=Path\n"

        for r in range(max_r - 1, min_r - 1, -1):
            row_str = ""
            for c in range(min_c, max_c):
                wc = c - pad_min_c
                wr = r - pad_min_r
                if c == robot_c and r == robot_r:
                    row_str += " R "
                elif goal_c is not None and c == goal_c and r == goal_r:
                    row_str += " G "
                elif self.grid[c, r] > self.obstacle_threshold:
                    row_str += "## "
                elif inflated_window[wc, wr] == 1:
                    row_str += "++ "
                elif (c, r) in path_cells:
                    row_str += f" {path_cells[(c, r)]} "
                elif self.grid[c, r] < 45:
                    row_str += ".. "
                else:
                    row_str += "   "
            out_str += row_str + "\n"

        try:
            with open("/tmp/rover_map.txt", "w") as f:
                f.write(out_str)
        except Exception:
            pass
