#!/usr/bin/env python3
"""
Script to create a 4x4 grid with labeled nodes at intersections.
Vertical lines: a, b, c, d
Horizontal lines: 1, 2, 3, 4
Intersections: a1, a2, a3, a4, b1, b2, b3, b4, c1, c2, c3, c4, d1, d2, d3, d4
"""

import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np

def create_grid_visualization(output_path='grid_4x4.png'):
    """
    Create and visualize a 3x3 grid with labeled nodes.
    
    Args:
        output_path (str): Path to save the visualization image
    """
    
    # Create figure and axis
    fig, ax = plt.subplots(1, 1, figsize=(10, 10))
    
    # Grid parameters
    cols = ['a', 'b', 'c', 'd']
    rows = ['1', '2', '3', '4']
    grid_size = 4
    cell_size = 1
    
    # Draw grid lines
    for i in range(grid_size + 1):
        # Vertical lines
        ax.plot([i * cell_size, i * cell_size], 
                [0, grid_size * cell_size], 
                'k-', linewidth=2)
        # Horizontal lines
        ax.plot([0, grid_size * cell_size], 
                [i * cell_size, i * cell_size], 
                'k-', linewidth=2)
    
    # Add nodes and labels at intersections
    node_data = []
    
    for i, col in enumerate(cols):
        for j, row in enumerate(rows):
            x = i * cell_size
            y = (grid_size - 1 - j) * cell_size  # Flip y-axis so row 3 is on top
            node_label = f"{col}{row}"
            
            # Draw node circle
            circle = patches.Circle((x, y), radius=0.1, 
                                   color='red', zorder=3)
            ax.add_patch(circle)
            
            # Add label near node
            ax.text(x, y - 0.2, node_label, 
                   fontsize=12, fontweight='bold',
                   ha='center', va='top',
                   bbox=dict(boxstyle='round,pad=0.3', 
                            facecolor='yellow', alpha=0.7))
            
            node_data.append({
                'label': node_label,
                'x': x,
                'y': y,
                'col': col,
                'row': row
            })
    
    # Add column labels (a, b, c) at the bottom
    for i, col in enumerate(cols):
        x = i * cell_size
        ax.text(x, -0.4, col, fontsize=14, fontweight='bold',
               ha='center', va='top',
               bbox=dict(boxstyle='round,pad=0.4', 
                        facecolor='lightblue', alpha=0.8))
    
    # Add row labels (1, 2, 3) on the left
    for j, row in enumerate(rows):
        y = (grid_size - 1 - j) * cell_size
        ax.text(-0.4, y, row, fontsize=14, fontweight='bold',
               ha='right', va='center',
               bbox=dict(boxstyle='round,pad=0.4', 
                        facecolor='lightgreen', alpha=0.8))
    
    # Set axis properties
    ax.set_xlim(-0.7, grid_size - 0.3)
    ax.set_ylim(-0.7, grid_size - 0.3)
    ax.set_aspect('equal')
    ax.axis('off')
    
    # Add title
    plt.title('4x4 Grid with Node Intersections', 
             fontsize=16, fontweight='bold', pad=20)
    
    # Save figure
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"Grid visualization saved to: {output_path}")
    
    # Print node data
    print("\nNode Coordinates:")
    print("=" * 50)
    print(f"{'Node Label':<15} {'Column':<10} {'Row':<10} {'X':<10} {'Y':<10}")
    print("-" * 50)
    for node in node_data:
        print(f"{node['label']:<15} {node['col']:<10} {node['row']:<10} "
              f"{node['x']:<10.2f} {node['y']:<10.2f}")
    
    plt.show()
    return node_data


def get_node_by_label(node_data, label):
    """
    Get node coordinates by label.
    
    Args:
        node_data (list): List of node dictionaries
        label (str): Node label (e.g., 'a1', 'b2')
    
    Returns:
        dict: Node data if found, None otherwise
    """
    for node in node_data:
        if node['label'] == label:
            return node
    return None


if __name__ == "__main__":
    # Create and display grid
    nodes = create_grid_visualization('grid_4x4.png')
    
    # Example: Query a node
    print("\n" + "=" * 50)
    print("Example Node Query:")
    sample_node = get_node_by_label(nodes, 'b2')
    if sample_node:
        print(f"Node {sample_node['label']}: x={sample_node['x']}, y={sample_node['y']}")
