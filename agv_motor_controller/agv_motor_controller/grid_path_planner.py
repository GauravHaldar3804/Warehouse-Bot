#!/usr/bin/env python3
"""
Grid Path Planning and Navigation System
Implements pathfinding algorithms for 4x4 grid navigation
Supports obstacle avoidance and multiple pathfinding methods
"""

import matplotlib
matplotlib.use('Agg')  # Use non-interactive backend
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from collections import deque
import heapq
from typing import List, Dict, Tuple, Set
import numpy as np

class GridNode:
    """Represents a node in the grid"""
    def __init__(self, label: str, x: float, y: float, col: str, row: str):
        self.label = label
        self.x = x
        self.y = y
        self.col = col
        self.row = row
        self.is_obstacle = False
    
    def __repr__(self):
        return f"GridNode({self.label})"
    
    def distance_to(self, other: 'GridNode') -> float:
        """Calculate Euclidean distance to another node"""
        return np.sqrt((self.x - other.x)**2 + (self.y - other.y)**2)


class GridPathPlanner:
    """Path planning system for 4x4 grid"""
    
    def __init__(self, grid_size: int = 4):
        """
        Initialize the grid path planner
        
        Args:
            grid_size (int): Size of the grid (default 4x4)
        """
        self.grid_size = grid_size
        self.nodes: Dict[str, GridNode] = {}
        self.obstacles: Set[str] = set()
        self._create_grid()
    
    def _create_grid(self):
        """Create grid nodes including intermediate nodes and dock nodes"""
        cols = [chr(65 + i) for i in range(self.grid_size)]  # A, B, C, D
        rows = [str(i + 1) for i in range(self.grid_size)]   # 1, 2, 3, 4
        
        # Create main grid nodes (intersections)
        for i, col in enumerate(cols):
            for j, row in enumerate(rows):
                label = f"{col}{row}"
                x = i
                y = (self.grid_size - 1 - j)
                self.nodes[label] = GridNode(label, x, y, col, row)
        
        # Create intermediate nodes on vertical lines (between rows)
        for i, col in enumerate(cols):
            for j in range(self.grid_size - 1):
                row1 = str(j + 1)
                row2 = str(j + 2)
                label = f"{col}{row1}{row2}"
                x = i
                y = (self.grid_size - 1 - j) - 0.5
                self.nodes[label] = GridNode(label, x, y, col, f"{row1}-{row2}")
        
        # Create intermediate nodes on horizontal lines (between columns)
        horizontal_intermediate_nodes = []
        for i in range(self.grid_size - 1):
            col1 = chr(65 + i)
            col2 = chr(65 + i + 1)
            for j, row in enumerate(rows):
                label = f"{col1}{col2}{row}"
                x = i + 0.5
                y = (self.grid_size - 1 - j)
                self.nodes[label] = GridNode(label, x, y, f"{col1}-{col2}", row)
                horizontal_intermediate_nodes.append((label, x, y))
        
        # Create dock nodes extending upward from horizontal intermediate nodes (except row 1)
        for horiz_label, horiz_x, horiz_y in horizontal_intermediate_nodes:
            # Skip creating dock nodes for row 1 (y = 3)
            if horiz_y == 3:
                continue
            doc_label = f"DOC-{horiz_label}"
            doc_x = horiz_x
            doc_y = horiz_y + 0.5  # Extend 0.5 units upward
            self.nodes[doc_label] = GridNode(doc_label, doc_x, doc_y, f"dock-{horiz_label}", f"doc")
        
        # Create home nodes extending to the right from cd horizontal intermediate nodes
        for j, row in enumerate(rows):
            home_label = f"HOME-{row}"
            home_x = (self.grid_size - 1) + 0.5  # Extend 0.5 units to the right of column d
            home_y = (self.grid_size - 1 - j)
            self.nodes[home_label] = GridNode(home_label, home_x, home_y, "home", row)
    
    def add_obstacle(self, node_label: str):
        """Mark a node as obstacle"""
        if node_label in self.nodes:
            self.nodes[node_label].is_obstacle = True
            self.obstacles.add(node_label)
            print(f"Obstacle added at {node_label}")
    
    def remove_obstacle(self, node_label: str):
        """Remove obstacle from a node"""
        if node_label in self.obstacles:
            self.nodes[node_label].is_obstacle = False
            self.obstacles.remove(node_label)
            print(f"Obstacle removed from {node_label}")
    
    def get_neighbors(self, node_label: str) -> List[GridNode]:
        """Get valid (non-obstacle) neighbors of a node"""
        if node_label not in self.nodes:
            return []
        
        node = self.nodes[node_label]
        neighbors = []
        
        # Check all other nodes and find adjacent ones based on position
        for other_label, other_node in self.nodes.items():
            if other_node.is_obstacle or other_label == node_label:
                continue
            
            # Check if nodes are adjacent (distance ≈ 0.5 on grid)
            dist = np.sqrt((node.x - other_node.x)**2 + (node.y - other_node.y)**2)
            
            if abs(dist - 0.5) < 0.01:  # Adjacent nodes (tolerance for floating point)
                neighbors.append(other_node)
        
        return neighbors
    
    def get_direction(self, from_label: str, to_label: str) -> Tuple[float, float]:
        """Get normalized direction vector from one node to another"""
        if from_label not in self.nodes or to_label not in self.nodes:
            return (0, 0)
        
        from_node = self.nodes[from_label]
        to_node = self.nodes[to_label]
        
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        
        # Normalize
        dist = np.sqrt(dx**2 + dy**2)
        if dist == 0:
            return (0, 0)
        
        return (dx/dist, dy/dist)
    
    def get_turn_direction(self, prev_dir: Tuple[float, float], curr_dir: Tuple[float, float]) -> str:
        """Determine turn direction: LEFT, RIGHT, STRAIGHT"""
        if prev_dir == (0, 0) or curr_dir == (0, 0):
            return "START"
        
        # Calculate cross product to determine turn direction
        cross = prev_dir[0] * curr_dir[1] - prev_dir[1] * curr_dir[0]
        
        # Calculate dot product to check if going straight
        dot = prev_dir[0] * curr_dir[0] + prev_dir[1] * curr_dir[1]
        
        # If dot product is close to 1, going straight
        if abs(dot - 1.0) < 0.01:
            return "STRAIGHT"
        # If dot product is close to -1, doing a U-turn
        elif abs(dot + 1.0) < 0.01:
            return "UTURN"
        # If cross product is positive, turn left
        elif cross > 0.01:
            return "LEFT"
        # If cross product is negative, turn right
        elif cross < -0.01:
            return "RIGHT"
        else:
            return "STRAIGHT"
    
    def get_path_with_directions(self, path: List[str]) -> List[Tuple[str, str]]:
        """Convert path to list of (node, direction) tuples"""
        if len(path) < 2:
            return [(path[0], "STOP")] if path else []
        
        result = []
        prev_direction = (0, 0)
        
        for i, node in enumerate(path):
            if i == 0:
                # First node: START
                result.append((node, "START"))
                # Get direction to next node
                prev_direction = self.get_direction(path[0], path[1])
            elif i == len(path) - 1:
                # Last node: STOP
                result.append((node, "STOP"))
            else:
                # Middle nodes: determine turn
                curr_direction = self.get_direction(path[i-1], path[i])
                next_direction = self.get_direction(path[i], path[i+1])
                
                turn = self.get_turn_direction(curr_direction, next_direction)
                result.append((node, turn))
                
                prev_direction = next_direction
        
        return result
    
    def bfs_path(self, start: str, goal: str) -> List[str]:
        """
        Breadth-First Search pathfinding
        
        Args:
            start (str): Starting node label
            goal (str): Goal node label
        
        Returns:
            List[str]: Path from start to goal, empty list if no path exists
        """
        if start not in self.nodes or goal not in self.nodes:
            return []
        if self.nodes[start].is_obstacle or self.nodes[goal].is_obstacle:
            return []
        
        visited = {start}
        queue = deque([(start, [start])])
        
        while queue:
            current_label, path = queue.popleft()
            
            if current_label == goal:
                return path
            
            for neighbor in self.get_neighbors(current_label):
                if neighbor.label not in visited:
                    visited.add(neighbor.label)
                    queue.append((neighbor.label, path + [neighbor.label]))
        
        return []
    
    def dijkstra_path(self, start: str, goal: str) -> List[str]:
        """
        Dijkstra's algorithm pathfinding
        
        Args:
            start (str): Starting node label
            goal (str): Goal node label
        
        Returns:
            List[str]: Shortest path from start to goal
        """
        if start not in self.nodes or goal not in self.nodes:
            return []
        if self.nodes[start].is_obstacle or self.nodes[goal].is_obstacle:
            return []
        
        distances = {node: float('inf') for node in self.nodes}
        distances[start] = 0
        previous = {node: None for node in self.nodes}
        visited = set()
        
        pq = [(0, start)]
        
        while pq:
            current_dist, current = heapq.heappop(pq)
            
            if current in visited:
                continue
            
            visited.add(current)
            
            if current == goal:
                # Reconstruct path
                path = []
                node = goal
                while node is not None:
                    path.append(node)
                    node = previous[node]
                return path[::-1]
            
            for neighbor in self.get_neighbors(current):
                distance = distances[current] + current_dist + 1
                
                if distance < distances[neighbor.label]:
                    distances[neighbor.label] = distance
                    previous[neighbor.label] = current
                    heapq.heappush(pq, (distance, neighbor.label))
        
        return []
    
    def astar_path(self, start: str, goal: str) -> List[str]:
        """
        A* pathfinding algorithm
        
        Args:
            start (str): Starting node label
            goal (str): Goal node label
        
        Returns:
            List[str]: Optimal path from start to goal
        """
        if start not in self.nodes or goal not in self.nodes:
            return []
        if self.nodes[start].is_obstacle or self.nodes[goal].is_obstacle:
            return []
        
        def heuristic(node_label: str) -> float:
            """Manhattan distance heuristic"""
            node = self.nodes[node_label]
            goal_node = self.nodes[goal]
            return abs(node.x - goal_node.x) + abs(node.y - goal_node.y)
        
        open_set = {start}
        came_from = {}
        g_score = {node: float('inf') for node in self.nodes}
        g_score[start] = 0
        f_score = {node: float('inf') for node in self.nodes}
        f_score[start] = heuristic(start)
        
        pq = [(f_score[start], start)]
        
        while pq:
            _, current = heapq.heappop(pq)
            
            if current == goal:
                # Reconstruct path
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                return path[::-1]
            
            open_set.discard(current)
            
            for neighbor in self.get_neighbors(current):
                tentative_g_score = g_score[current] + 1
                
                if tentative_g_score < g_score[neighbor.label]:
                    came_from[neighbor.label] = current
                    g_score[neighbor.label] = tentative_g_score
                    f_score[neighbor.label] = g_score[neighbor.label] + heuristic(neighbor.label)
                    
                    if neighbor.label not in open_set:
                        open_set.add(neighbor.label)
                        heapq.heappush(pq, (f_score[neighbor.label], neighbor.label))
        
        return []
    
    def visualize_path(self, start: str, goal: str, path: List[str], 
                      algorithm_name: str = "BFS", output_path: str = None):
        """
        Visualize the grid and the planned path
        
        Args:
            start (str): Starting node label
            goal (str): Goal node label
            path (List[str]): Path to visualize
            algorithm_name (str): Name of the algorithm used
            output_path (str): Path to save the image
        """
        fig, ax = plt.subplots(1, 1, figsize=(14, 14))
        
        cols = [chr(65 + i) for i in range(self.grid_size)]  # A, B, C, D
        rows = [str(i + 1) for i in range(self.grid_size)]   # 1, 2, 3, 4
        cell_size = 1
        
        # Draw grid lines (black)
        for i in range(self.grid_size + 1):
            ax.plot([i * cell_size, i * cell_size], 
                   [0, self.grid_size * cell_size], 
                   'k-', linewidth=2)
            ax.plot([0, self.grid_size * cell_size], 
                   [i * cell_size, i * cell_size], 
                   'k-', linewidth=2)
        
        # Draw all nodes (uniform light gray, except start/goal which will be set later)
        for node in self.nodes.values():
            if node.label in self.obstacles:
                # Draw obstacle as black circle
                circle = patches.Circle((node.x, node.y), radius=0.1, 
                                       facecolor='black', edgecolor='black', linewidth=2, zorder=3)
                ax.add_patch(circle)
            else:
                # All nodes same style: light gray with black edges
                circle = patches.Circle((node.x, node.y), radius=0.08, 
                                       facecolor='lightgray', edgecolor='black', linewidth=1.5, zorder=2)
                ax.add_patch(circle)
            
            # Add labels for all nodes with uniform styling
            if node.label not in self.obstacles:
                ax.text(node.x, node.y, node.label, 
                       fontsize=6, ha='center', va='center', fontweight='bold', alpha=0.7)
        
        # Draw edges from horizontal intermediate nodes to their dock nodes (black lines)
        for node in self.nodes.values():
            if node.label.startswith('DOC-'):
                # Extract the parent node label
                parent_label = node.label.replace('DOC-', '')
                if parent_label in self.nodes:
                    parent_node = self.nodes[parent_label]
                    # Draw edge line between them (black, dashed)
                    ax.plot([parent_node.x, node.x], [parent_node.y, node.y], 
                           'k--', linewidth=1.5, alpha=0.5, zorder=1)
        
        # Draw edges from D nodes to home nodes (black lines)
        for node in self.nodes.values():
            if node.label.startswith('HOME-'):
                # Find the corresponding CD horizontal intermediate node
                row = node.label.replace('HOME-', '')
                cd_label = f"CD{row}"
                if cd_label in self.nodes:
                    cd_node = self.nodes[cd_label]
                    # Draw edge line between them (black, dashed)
                    ax.plot([cd_node.x, node.x], [cd_node.y, node.y], 
                           'k--', linewidth=1.5, alpha=0.5, zorder=1)
        
        # Draw path (blue, thick)
        if path and len(path) > 1:
            path_x = [self.nodes[label].x for label in path]
            path_y = [self.nodes[label].y for label in path]
            
            # Draw path line (blue, thick)
            ax.plot(path_x, path_y, 'b-', linewidth=3.5, alpha=0.8, label='Planned Path', zorder=1.5)
        
        # Highlight start node (green)
        if start in self.nodes:
            start_node = self.nodes[start]
            circle = patches.Circle((start_node.x, start_node.y), radius=0.08, 
                                   facecolor='green', edgecolor='darkgreen', 
                                   linewidth=2, zorder=3)
            ax.add_patch(circle)
        
        # Highlight goal node (red)
        if goal in self.nodes:
            goal_node = self.nodes[goal]
            circle = patches.Circle((goal_node.x, goal_node.y), radius=0.08, 
                                   facecolor='red', edgecolor='darkred', 
                                   linewidth=2, zorder=3)
            ax.add_patch(circle)
        
        # Add column and row labels
        for i, col in enumerate(cols):
            ax.text(i * cell_size, -0.45, col, fontsize=12, fontweight='bold',
                   ha='center', va='top')
        
        for j, row in enumerate(rows):
            y = (self.grid_size - 1 - j) * cell_size
            ax.text(-0.45, y, row, fontsize=12, fontweight='bold',
                   ha='right', va='center')
        
        ax.set_xlim(-0.8, self.grid_size + 1)
        ax.set_ylim(-0.8, self.grid_size + 0.5)
        ax.set_aspect('equal')
        ax.axis('off')
        
        # Title
        path_length = len(path) - 1 if path else 0
        title = f'Path Planning: {start} → {goal}\n({algorithm_name}, Path Length: {path_length} steps, Total Nodes in Path: {len(path)})'
        plt.title(title, fontsize=13, fontweight='bold', pad=20)
        
        # Simplified legend
        from matplotlib.patches import Patch
        legend_elements = [
            Patch(facecolor='lightgray', edgecolor='black', label='Nodes'),
            plt.Line2D([0], [0], color='b', linewidth=3.5, label='Planned Path'),
            Patch(facecolor='green', edgecolor='darkgreen', label='Start'),
            Patch(facecolor='red', edgecolor='darkred', label='Goal'),
        ]
        ax.legend(handles=legend_elements, loc='upper right', fontsize=10)
        
        if output_path:
            plt.savefig(output_path, dpi=150, bbox_inches='tight')
            print(f"Visualization saved to: {output_path}")
        
        plt.close()  # Close the figure to free memory
        return fig, ax


def demo_path_planning():
    """Demonstrate path planning with Dijkstra algorithm including dock nodes"""
    
    # Create planner
    planner = GridPathPlanner(grid_size=4)
    
    # Print grid structure
    print("\n" + "=" * 60)
    print("GRID STRUCTURE - 4x4 with Enhanced Nodes")
    print("=" * 60)
    print(f"Total nodes created: {len(planner.nodes)}")
    print("\nNode Types:")
    print("  • Main Intersection Nodes (16): A1, A2, A3, A4, B1, B2, B3, B4, C1, C2, C3, C4, D1, D2, D3, D4")
    print("  • Vertical Intermediate Nodes (12): A12, A23, A34, B12, B23, B34, C12, C23, C34, D12, D23, D34")
    print("  • Horizontal Intermediate Nodes (12): AB1, AB2, AB3, AB4, BC1, BC2, BC3, BC4, CD1, CD2, CD3, CD4")
    print("  • Dock Nodes (9): DOC-AB2, DOC-AB3, DOC-AB4, DOC-BC2, DOC-BC3, DOC-BC4, DOC-CD2, DOC-CD3, DOC-CD4")
    print("  • Home Nodes (4): HOME-1, HOME-2, HOME-3, HOME-4")
    
    print("\nAvailable Nodes:")
    print("  ", sorted([node for node in planner.nodes.keys()]))
    
    # Get user input
    print("\n" + "=" * 60)
    print("PATH PLANNING - Dijkstra Algorithm")
    print("=" * 60)
    
    while True:
        start_node = input("\nEnter start node (or 'quit' to exit): ").strip().upper()
        if start_node == 'QUIT':
            print("Exiting...")
            break
        
        if start_node not in planner.nodes:
            print(f"❌ Node '{start_node}' not found! Try again.")
            continue
        
        end_node = input("Enter goal node: ").strip().upper()
        
        if end_node not in planner.nodes:
            print(f"❌ Node '{end_node}' not found! Try again.")
            continue
        
        if start_node == end_node:
            print("❌ Start and goal nodes must be different!")
            continue
        
        # Find path using Dijkstra
        path = planner.dijkstra_path(start_node, end_node)
        
        if path:
            print(f"\n✅ Path found!")
            print(f"  Start: {start_node} → Goal: {end_node}")
            print(f"  Path: {' → '.join(path)}")
            print(f"  Total steps: {len(path) - 1}")
            
            # Get path with directions
            path_with_directions = planner.get_path_with_directions(path)
            print(f"\n  📍 Detailed Navigation Instructions:")
            print(f"  {'-' * 50}")
            for i, (node, direction) in enumerate(path_with_directions):
                print(f"  {i+1:2d}. Node: {node:10s} | Action: {direction}")
            print(f"  {'-' * 50}")
            
            # Generate visualization
            output_file = f'path_dijkstra_{start_node}_{end_node}.png'
            planner.visualize_path(start_node, end_node, path, 'Dijkstra', output_file)
        else:
            print(f"❌ No path found from {start_node} to {end_node}!")
        
        print("\n" + "-" * 60)


if __name__ == "__main__":
    demo_path_planning()
