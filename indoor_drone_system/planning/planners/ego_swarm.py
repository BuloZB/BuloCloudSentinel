"""
EGO-Swarm Planner Implementation

This module implements the EGO-Swarm algorithm for path planning in complex environments.
Based on the paper: "EGO-Swarm: A Fully Autonomous and Decentralized Quadrotor Swarm System in Cluttered Environments"
"""

import numpy as np
import time
from typing import Dict, List, Any, Optional, Tuple

class EgoSwarmPlanner:
    """Implementation of the EGO-Swarm planning algorithm."""
    
    def __init__(self, config: Dict[str, Any]):
        """Initialize the EGO-Swarm planner."""
        self.config = config
        
        # Extract configuration parameters
        self.max_velocity = config.get('max_velocity', 1.0)
        self.max_acceleration = config.get('max_acceleration', 0.5)
        self.safety_distance = config.get('safety_distance', 0.5)
        self.planning_horizon = config.get('planning_horizon', 3.0)
        self.update_rate = config.get('update_rate', 10.0)
        
        # Initialize state
        self.map = None
        self.drone_positions = {}
        self.obstacles = []
        
        # Initialize grid for A* fallback
        self.grid = None
        self.grid_resolution = 0.1
    
    def update_map(self, map_data: Dict[str, Any]):
        """Update the map data."""
        self.map = map_data
        
        # Extract obstacles from map
        self._extract_obstacles_from_map()
        
        # Initialize grid for A* fallback
        self._initialize_grid()
    
    def update_drone_positions(self, drone_positions: Dict[str, Dict[str, Any]]):
        """Update the positions of all drones."""
        self.drone_positions = drone_positions
    
    def plan(self, start: Dict[str, float], goal: Dict[str, float]) -> Tuple[List[Dict[str, float]], bool]:
        """
        Plan a path from start to goal.
        
        Args:
            start: Start position {x, y, z}
            goal: Goal position {x, y, z}
            
        Returns:
            Tuple of (path, success)
            - path: List of positions [{x, y, z}, ...]
            - success: Whether planning was successful
        """
        if self.map is None:
            return [], False
        
        try:
            # Convert positions to numpy arrays
            start_pos = np.array([start['x'], start['y'], start['z']])
            goal_pos = np.array([goal['x'], goal['y'], goal['z']])
            
            # Check if direct path is possible
            if self._is_path_clear(start_pos, goal_pos):
                # Generate direct path with time parameterization
                path = self._generate_direct_path(start_pos, goal_pos)
                return path, True
            
            # Use EGO-Swarm algorithm
            path = self._ego_swarm_plan(start_pos, goal_pos)
            
            if len(path) > 0:
                return path, True
            
            # Fallback to A* if EGO-Swarm fails
            path = self._a_star_fallback(start_pos, goal_pos)
            
            if len(path) > 0:
                return path, True
            
            return [], False
        except Exception as e:
            print(f"Error in EGO-Swarm planning: {str(e)}")
            return [], False
    
    def _extract_obstacles_from_map(self):
        """Extract obstacles from the map data."""
        if self.map is None:
            return
        
        # Reset obstacles
        self.obstacles = []
        
        # Extract occupied cells from grid map
        if 'data' in self.map and 'resolution' in self.map:
            resolution = self.map['resolution']
            width = self.map['width']
            height = self.map['height']
            origin_x = self.map['origin']['x']
            origin_y = self.map['origin']['y']
            
            for i in range(height):
                for j in range(width):
                    idx = i * width + j
                    if idx < len(self.map['data']) and self.map['data'][idx] > 50:  # Occupied cell
                        x = origin_x + j * resolution
                        y = origin_y + i * resolution
                        self.obstacles.append(np.array([x, y, 0.0]))  # Assuming 2D map
    
    def _initialize_grid(self):
        """Initialize grid for A* fallback."""
        if self.map is None:
            return
        
        # Create grid with inflated obstacles
        resolution = self.map['resolution']
        width = self.map['width']
        height = self.map['height']
        
        self.grid = np.zeros((height, width), dtype=np.uint8)
        
        # Mark occupied cells
        for i in range(height):
            for j in range(width):
                idx = i * width + j
                if idx < len(self.map['data']) and self.map['data'][idx] > 50:
                    self.grid[i, j] = 1
        
        # Inflate obstacles
        inflation_cells = int(self.safety_distance / resolution) + 1
        if inflation_cells > 0:
            inflated_grid = np.copy(self.grid)
            for i in range(height):
                for j in range(width):
                    if self.grid[i, j] == 1:
                        # Inflate obstacle
                        for di in range(-inflation_cells, inflation_cells + 1):
                            for dj in range(-inflation_cells, inflation_cells + 1):
                                ni, nj = i + di, j + dj
                                if 0 <= ni < height and 0 <= nj < width:
                                    inflated_grid[ni, nj] = 1
            
            self.grid = inflated_grid
    
    def _is_path_clear(self, start: np.ndarray, goal: np.ndarray) -> bool:
        """Check if a direct path from start to goal is clear of obstacles."""
        direction = goal - start
        distance = np.linalg.norm(direction)
        
        if distance < 1e-6:
            return True
        
        direction = direction / distance
        
        # Check for collisions along the path
        step_size = self.grid_resolution if self.grid_resolution else 0.1
        num_steps = int(distance / step_size) + 1
        
        for i in range(num_steps + 1):
            t = i / num_steps
            point = start + t * (goal - start)
            
            # Check if point is in collision
            if self._is_in_collision(point):
                return False
        
        return True
    
    def _is_in_collision(self, point: np.ndarray) -> bool:
        """Check if a point is in collision with obstacles."""
        # Check map boundaries
        if self.map is None:
            return False
        
        resolution = self.map['resolution']
        width = self.map['width']
        height = self.map['height']
        origin_x = self.map['origin']['x']
        origin_y = self.map['origin']['y']
        
        # Convert point to grid coordinates
        grid_x = int((point[0] - origin_x) / resolution)
        grid_y = int((point[1] - origin_y) / resolution)
        
        # Check if point is within grid bounds
        if grid_x < 0 or grid_x >= width or grid_y < 0 or grid_y >= height:
            return True  # Out of bounds is considered collision
        
        # Check if point is in collision in the grid
        if self.grid is not None:
            return self.grid[grid_y, grid_x] == 1
        
        # Check against raw map data
        idx = grid_y * width + grid_x
        if idx < len(self.map['data']):
            return self.map['data'][idx] > 50
        
        return False
    
    def _generate_direct_path(self, start: np.ndarray, goal: np.ndarray) -> List[Dict[str, float]]:
        """Generate a direct path from start to goal with time parameterization."""
        direction = goal - start
        distance = np.linalg.norm(direction)
        
        if distance < 1e-6:
            return [{'x': start[0], 'y': start[1], 'z': start[2]}]
        
        # Calculate number of waypoints based on distance
        num_waypoints = max(2, int(distance / 0.2) + 1)
        
        # Generate waypoints
        path = []
        for i in range(num_waypoints):
            t = i / (num_waypoints - 1)
            point = start + t * direction
            path.append({
                'x': float(point[0]),
                'y': float(point[1]),
                'z': float(point[2])
            })
        
        return path
    
    def _ego_swarm_plan(self, start: np.ndarray, goal: np.ndarray) -> List[Dict[str, float]]:
        """Plan a path using the EGO-Swarm algorithm."""
        # Initialize path with start point
        path = [{'x': float(start[0]), 'y': float(start[1]), 'z': float(start[2])}]
        
        # Initialize current position
        current_pos = start.copy()
        
        # Set parameters
        dt = 1.0 / self.update_rate
        max_steps = 1000  # Prevent infinite loops
        goal_threshold = 0.2  # Distance to consider goal reached
        
        # Main planning loop
        for step in range(max_steps):
            # Check if goal is reached
            distance_to_goal = np.linalg.norm(goal - current_pos)
            if distance_to_goal < goal_threshold:
                # Add goal to path
                path.append({'x': float(goal[0]), 'y': float(goal[1]), 'z': float(goal[2])})
                break
            
            # Calculate attractive force (goal attraction)
            f_att = self._attractive_force(current_pos, goal)
            
            # Calculate repulsive force (obstacle avoidance)
            f_rep = self._repulsive_force(current_pos)
            
            # Calculate total force
            f_total = f_att + f_rep
            
            # Normalize force if it exceeds max acceleration
            force_magnitude = np.linalg.norm(f_total)
            if force_magnitude > self.max_acceleration:
                f_total = f_total * (self.max_acceleration / force_magnitude)
            
            # Update position using simple dynamics
            new_pos = current_pos + f_total * dt * dt
            
            # Check if new position is in collision
            if self._is_in_collision(new_pos):
                # Try to find a valid direction
                valid_direction_found = False
                for angle in np.linspace(0, 2*np.pi, 12):
                    # Try different directions
                    direction = np.array([np.cos(angle), np.sin(angle), 0.0])
                    test_pos = current_pos + direction * 0.2
                    
                    if not self._is_in_collision(test_pos):
                        new_pos = test_pos
                        valid_direction_found = True
                        break
                
                if not valid_direction_found:
                    # No valid direction found, terminate planning
                    break
            
            # Update current position
            current_pos = new_pos
            
            # Add new position to path
            path.append({
                'x': float(current_pos[0]),
                'y': float(current_pos[1]),
                'z': float(current_pos[2])
            })
        
        return path
    
    def _attractive_force(self, current: np.ndarray, goal: np.ndarray) -> np.ndarray:
        """Calculate attractive force towards goal."""
        direction = goal - current
        distance = np.linalg.norm(direction)
        
        if distance < 1e-6:
            return np.zeros(3)
        
        # Normalize direction
        direction = direction / distance
        
        # Calculate force magnitude (constant within planning horizon, decreases beyond)
        if distance <= self.planning_horizon:
            force_magnitude = self.max_acceleration
        else:
            force_magnitude = self.max_acceleration * (self.planning_horizon / distance)
        
        return direction * force_magnitude
    
    def _repulsive_force(self, position: np.ndarray) -> np.ndarray:
        """Calculate repulsive force from obstacles."""
        repulsive_force = np.zeros(3)
        
        # Check against all obstacles
        for obstacle in self.obstacles:
            direction = position - obstacle
            distance = np.linalg.norm(direction)
            
            # Skip if obstacle is too far
            if distance > self.safety_distance * 2:
                continue
            
            if distance < 1e-6:
                # Avoid division by zero, add random direction
                direction = np.random.randn(3)
                direction = direction / np.linalg.norm(direction)
                distance = 1e-6
            else:
                direction = direction / distance
            
            # Calculate repulsive force (inversely proportional to distance)
            if distance < self.safety_distance:
                force_magnitude = self.max_acceleration * (1.0 - distance / self.safety_distance)
                repulsive_force += direction * force_magnitude
        
        # Check against other drones
        for drone_id, drone_data in self.drone_positions.items():
            drone_pos = np.array([
                drone_data['position']['x'],
                drone_data['position']['y'],
                drone_data['position']['z']
            ])
            
            direction = position - drone_pos
            distance = np.linalg.norm(direction)
            
            # Skip if drone is too far or it's the same position
            if distance > self.safety_distance * 2 or distance < 1e-6:
                continue
            
            direction = direction / distance
            
            # Calculate repulsive force (inversely proportional to distance)
            if distance < self.safety_distance:
                force_magnitude = self.max_acceleration * (1.0 - distance / self.safety_distance)
                repulsive_force += direction * force_magnitude
        
        return repulsive_force
    
    def _a_star_fallback(self, start: np.ndarray, goal: np.ndarray) -> List[Dict[str, float]]:
        """Fallback to A* algorithm if EGO-Swarm fails."""
        # This is a simplified A* implementation for fallback
        # In a real implementation, you would use a more sophisticated A* algorithm
        
        if self.grid is None or self.map is None:
            return []
        
        # Convert start and goal to grid coordinates
        resolution = self.map['resolution']
        origin_x = self.map['origin']['x']
        origin_y = self.map['origin']['y']
        
        start_grid = (
            int((start[1] - origin_y) / resolution),
            int((start[0] - origin_x) / resolution)
        )
        
        goal_grid = (
            int((goal[1] - origin_y) / resolution),
            int((goal[0] - origin_x) / resolution)
        )
        
        # Check if start or goal is in collision
        height, width = self.grid.shape
        if (start_grid[0] < 0 or start_grid[0] >= height or
            start_grid[1] < 0 or start_grid[1] >= width or
            self.grid[start_grid] == 1):
            return []
        
        if (goal_grid[0] < 0 or goal_grid[0] >= height or
            goal_grid[1] < 0 or goal_grid[1] >= width or
            self.grid[goal_grid] == 1):
            return []
        
        # A* algorithm
        from queue import PriorityQueue
        
        # Define heuristic function (Euclidean distance)
        def heuristic(a, b):
            return np.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)
        
        # Initialize data structures
        open_set = PriorityQueue()
        open_set.put((0, start_grid))
        came_from = {}
        g_score = {start_grid: 0}
        f_score = {start_grid: heuristic(start_grid, goal_grid)}
        open_set_hash = {start_grid}
        
        # Define possible movements (8-connected grid)
        movements = [
            (0, 1), (1, 0), (0, -1), (-1, 0),  # 4-connected
            (1, 1), (1, -1), (-1, 1), (-1, -1)  # Diagonals
        ]
        
        # A* search
        while not open_set.empty():
            _, current = open_set.get()
            open_set_hash.remove(current)
            
            if current == goal_grid:
                # Reconstruct path
                grid_path = []
                while current in came_from:
                    grid_path.append(current)
                    current = came_from[current]
                
                grid_path.append(start_grid)
                grid_path.reverse()
                
                # Convert grid path to world coordinates
                path = []
                for grid_point in grid_path:
                    x = origin_x + grid_point[1] * resolution
                    y = origin_y + grid_point[0] * resolution
                    z = start[2]  # Maintain same height as start
                    
                    path.append({'x': float(x), 'y': float(y), 'z': float(z)})
                
                # Add exact goal position as last point
                path.append({'x': float(goal[0]), 'y': float(goal[1]), 'z': float(goal[2])})
                
                return path
            
            # Check neighbors
            for movement in movements:
                neighbor = (current[0] + movement[0], current[1] + movement[1])
                
                # Check if neighbor is valid
                if (neighbor[0] < 0 or neighbor[0] >= height or
                    neighbor[1] < 0 or neighbor[1] >= width or
                    self.grid[neighbor] == 1):
                    continue
                
                # Calculate tentative g_score
                tentative_g_score = g_score[current]
                if movement[0] != 0 and movement[1] != 0:
                    # Diagonal movement costs more
                    tentative_g_score += 1.414
                else:
                    tentative_g_score += 1.0
                
                # Check if this path is better
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal_grid)
                    
                    if neighbor not in open_set_hash:
                        open_set.put((f_score[neighbor], neighbor))
                        open_set_hash.add(neighbor)
        
        # No path found
        return []
