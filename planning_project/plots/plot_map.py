#!/usr/bin/env python3
"""
Standalone script to visualize grid map saved from C++ planning node.

Usage:
    python3 plot_map.py <map_file> [output_file] [--trajectory trajectory_file]

Examples:
    python3 plot_map.py ../results/map.json
    python3 plot_map.py ../results/map.json ../results/map_visualization.png
    python3 plot_map.py ../results/map.json --trajectory ../results/trajectory.json
    python3 plot_map.py ../results/map.json output.png --trajectory ../results/trajectory.json
"""

import json
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.collections import PatchCollection
import sys
import os
import numpy as np


def load_map(filename):
    """Load map data from JSON file."""
    with open(filename, 'r') as f:
        data = json.load(f)
    return data


def load_trajectory(filename):
    """Load trajectory data from JSON file."""
    with open(filename, 'r') as f:
        data = json.load(f)
    return data


def plot_trajectory_on_axes(ax, traj_data, show_labels=True):
    """
    Plot trajectory, waypoints, and Dubins curves on existing axes.

    Args:
        ax: Matplotlib axes object
        traj_data: Dictionary containing trajectory data from JSON
        show_labels: Whether to include labels in legend (set False to avoid duplicates)
    """
    # Plot trajectory path
    if 'trajectory' in traj_data and len(traj_data['trajectory']) > 0:
        traj_points = traj_data['trajectory']
        traj_x = [pt['x'] for pt in traj_points]
        traj_y = [pt['y'] for pt in traj_points]
        label = 'Dubins Path' if show_labels else None
        ax.plot(traj_x, traj_y, 'b-', linewidth=2, alpha=0.7, label=label, zorder=10)

    # Plot Dubins arcs (show individual arc segments)
    dubins_label_added = False
    if 'dubins_curves' in traj_data:
        for curve in traj_data['dubins_curves']:
            for arc in curve['arcs']:
                if arc['length'] < 1e-6:
                    continue

                # Sample points along arc
                num_samples = max(10, int(arc['length'] * 20))
                arc_x = []
                arc_y = []

                x = arc['start']['x']
                y = arc['start']['y']
                theta = arc['start']['theta']
                k = arc['k']
                length = arc['length']
                step = length / num_samples

                for i in range(num_samples + 1):
                    arc_x.append(x)
                    arc_y.append(y)

                    if i < num_samples:
                        if abs(k) < 1e-6:  # Straight
                            x += step * np.cos(theta)
                            y += step * np.sin(theta)
                        else:  # Arc
                            dtheta = k * step
                            x += (np.sin(theta + dtheta) - np.sin(theta)) / k
                            y += (-np.cos(theta + dtheta) + np.cos(theta)) / k
                            theta += dtheta

                # Plot arc segment with different style (thinner, lighter)
                # Add label only once
                label = None
                if show_labels and not dubins_label_added:
                    label = 'Dubins Path'
                    dubins_label_added = True
                ax.plot(arc_x, arc_y, 'c-', linewidth=1, alpha=0.3, label=label, zorder=9)

    # Plot victims
    if 'victims' in traj_data:
        visited_x, visited_y = [], []
        unvisited_x, unvisited_y = [], []

        for victim in traj_data['victims']:
            if victim['visited']:
                visited_x.append(victim['x'])
                visited_y.append(victim['y'])
            else:
                unvisited_x.append(victim['x'])
                unvisited_y.append(victim['y'])

        if visited_x:
            label = 'Visited Victims' if show_labels else None
            ax.scatter(visited_x, visited_y, c='lime', s=150, marker='o',
                      edgecolors='darkgreen', linewidths=2, label=label,
                      zorder=12)
        if unvisited_x:
            label = 'Unvisited Victims' if show_labels else None
            ax.scatter(unvisited_x, unvisited_y, c='lightcoral', s=100, marker='o',
                      edgecolors='darkred', linewidths=1.5, label=label,
                      zorder=11, alpha=0.6)

    # Plot start position
    if 'start' in traj_data:
        start = traj_data['start']
        label = 'Start' if show_labels else None
        ax.scatter(start['x'], start['y'], c='blue', s=200, marker='s',
                  edgecolors='darkblue', linewidths=2, label=label, zorder=13)
        # Draw orientation arrow
        arrow_len = 0.5
        dx = arrow_len * np.cos(start['theta'])
        dy = arrow_len * np.sin(start['theta'])
        ax.arrow(start['x'], start['y'], dx, dy, head_width=0.2, head_length=0.2,
                fc='blue', ec='darkblue', linewidth=2, zorder=13)

    # Plot goal position
    if 'goal' in traj_data:
        goal = traj_data['goal']
        label = 'Goal (Gate)' if show_labels else None
        ax.scatter(goal['x'], goal['y'], c='red', s=200, marker='*',
                  edgecolors='darkred', linewidths=2, label=label, zorder=13)
        # Draw orientation arrow
        arrow_len = 0.5
        dx = arrow_len * np.cos(goal['theta'])
        dy = arrow_len * np.sin(goal['theta'])
        ax.arrow(goal['x'], goal['y'], dx, dy, head_width=0.2, head_length=0.2,
                fc='red', ec='darkred', linewidth=2, zorder=13)

    # Add statistics text
    if 'stats' in traj_data:
        stats = traj_data['stats']
        stats_text = f"Path Stats:\n"
        stats_text += f"Distance: {stats['total_distance']:.2f} m\n"
        stats_text += f"Time: {stats['total_time']:.2f} s\n"
        stats_text += f"Victims: {stats['num_victims_visited']}"
        ax.text(0.98, 0.02, stats_text, transform=ax.transAxes,
                fontsize=9, verticalalignment='bottom', horizontalalignment='right',
                bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.8))


def plot_map(map_data, output_file=None, show=True, trajectory_data=None):
    """
    Visualize the grid map.

    Args:
        map_data: Dictionary containing map data from JSON
        output_file: Optional path to save the figure
        show: Whether to display the plot interactively
        trajectory_data: Optional trajectory data to overlay on map
    """
    fig, ax = plt.subplots(figsize=(12, 10))

    # Get bounds
    bounds = map_data['bounds']
    min_x = bounds['min_x']
    min_y = bounds['min_y']
    max_x = bounds['max_x']
    max_y = bounds['max_y']

    # Prepare cell collections
    free_cells = []
    occupied_cells = []
    mixed_cells = []

    # Color scheme
    colors = {
        'free': '#E8F5E9',      # Light green
        'occupied': '#333333',   # Dark gray
        'mixed': '#FFE082'       # Light orange
    }

    edge_colors = {
        'free': '#81C784',       # Green
        'occupied': '#000000',   # Black
        'mixed': '#FF9800'       # Orange
    }

    # Group cells by state
    for cell in map_data['cells']:
        x0, y0, x1, y1 = cell['x0'], cell['y0'], cell['x1'], cell['y1']
        state = cell['state']

        rect = patches.Rectangle((x0, y0), x1 - x0, y1 - y0,
                                linewidth=0.5)

        if state == 'free':
            free_cells.append(rect)
        elif state == 'occupied':
            occupied_cells.append(rect)
        elif state == 'mixed':
            mixed_cells.append(rect)

    # Create patch collections and add to plot
    if free_cells:
        free_collection = PatchCollection(free_cells,
                                         facecolor=colors['free'],
                                         edgecolor=edge_colors['free'],
                                         linewidth=0.3,
                                         label=f'Free ({len(free_cells)})')
        ax.add_collection(free_collection)

    if occupied_cells:
        occupied_collection = PatchCollection(occupied_cells,
                                             facecolor=colors['occupied'],
                                             edgecolor=edge_colors['occupied'],
                                             linewidth=0.3,
                                             label=f'Obstacles (Inflated) ({len(occupied_cells)})')
        ax.add_collection(occupied_collection)

    if mixed_cells:
        mixed_collection = PatchCollection(mixed_cells,
                                          facecolor=colors['mixed'],
                                          edgecolor=edge_colors['mixed'],
                                          linewidth=0.3,
                                          label=f'Mixed ({len(mixed_cells)})')
        ax.add_collection(mixed_collection)

    # Plot original obstacles (non-inflated outlines)
    if 'original_obstacles' in map_data:
        for i, obs_points in enumerate(map_data['original_obstacles']):
            if len(obs_points) >= 3:
                polygon = patches.Polygon(obs_points, closed=True,
                                         fill=False, edgecolor='red',
                                         linewidth=2, linestyle='-',
                                         label='Original Obstacles' if i == 0 else '')
                ax.add_patch(polygon)

    # Set axis properties
    ax.set_xlim(min_x - 0.5, max_x + 0.5)
    ax.set_ylim(min_y - 0.5, max_y + 0.5)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3, linestyle='--', linewidth=0.5)

    # Plot trajectory if provided
    if trajectory_data is not None:
        plot_trajectory_on_axes(ax, trajectory_data)

    # Labels and title
    ax.set_xlabel('X (m)', fontsize=12)
    ax.set_ylabel('Y (m)', fontsize=12)

    refined_status = "Refined" if map_data['refined'] else "Original"
    total_cells = len(map_data['cells'])
    title = f'Grid Map Visualization ({refined_status})\n'
    title += f'Resolution: {map_data["resolution"]:.2f}m, Total cells: {total_cells}'
    if trajectory_data is not None:
        title += ' + Dubins Path'
    ax.set_title(title, fontsize=14, fontweight='bold')

    # Legend - manually add handles for PatchCollections
    from matplotlib.patches import Patch
    handles, labels = ax.get_legend_handles_labels()

    # Add obstacles patch to legend (only if occupied cells exist)
    if occupied_cells:
        handles.insert(0, Patch(facecolor=colors['occupied'], edgecolor=edge_colors['occupied'],
                               label='Obstacles (Inflated)'))

    ax.legend(handles=handles, loc='upper right', fontsize=10, framealpha=0.9)

    # Add map info text
    info_text = f"Grid size: {map_data['grid_size']['width']}x{map_data['grid_size']['height']}\n"
    info_text += f"Bounds: [{min_x:.1f}, {max_x:.1f}] x [{min_y:.1f}, {max_y:.1f}]"
    ax.text(0.02, 0.98, info_text, transform=ax.transAxes,
            fontsize=9, verticalalignment='top',
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.7))

    plt.tight_layout()

    # Save if output file specified
    if output_file:
        plt.savefig(output_file, dpi=300, bbox_inches='tight')
        print(f"Figure saved to: {output_file}")

    # Show if requested
    if show:
        plt.show()

    return fig, ax


def plot_map_with_depth(map_data, output_file=None, show=True, trajectory_data=None):
    """
    Visualize the grid map with color-coded depth levels (for refined maps).

    Args:
        map_data: Dictionary containing map data from JSON
        output_file: Optional path to save the figure
        show: Whether to display the plot interactively
        trajectory_data: Optional trajectory data to overlay on map
    """
    if not map_data['refined']:
        print("Map is not refined, using standard visualization")
        return plot_map(map_data, output_file, show, trajectory_data)

    fig, ax = plt.subplots(figsize=(12, 10))

    # Get bounds
    bounds = map_data['bounds']
    min_x = bounds['min_x']
    min_y = bounds['min_y']
    max_x = bounds['max_x']
    max_y = bounds['max_y']

    # Find max depth
    max_depth = max(cell['depth'] for cell in map_data['cells'])

    # Depth-based colormap
    from matplotlib import cm
    import numpy as np

    depth_cmap = cm.get_cmap('viridis', max_depth + 1)

    # Plot cells
    for cell in map_data['cells']:
        x0, y0, x1, y1 = cell['x0'], cell['y0'], cell['x1'], cell['y1']
        state = cell['state']
        depth = cell['depth']

        if state == 'free':
            color = depth_cmap(depth / max_depth) if max_depth > 0 else 'lightgreen'
            edgecolor = 'green'
        elif state == 'occupied':
            color = 'black'
            edgecolor = 'black'
        else:  # mixed
            color = 'orange'
            edgecolor = 'darkorange'

        rect = patches.Rectangle((x0, y0), x1 - x0, y1 - y0,
                                facecolor=color, edgecolor=edgecolor,
                                linewidth=0.3, alpha=0.8)
        ax.add_patch(rect)

    # Plot original obstacles (non-inflated outlines)
    if 'original_obstacles' in map_data:
        for i, obs_points in enumerate(map_data['original_obstacles']):
            if len(obs_points) >= 3:
                polygon = patches.Polygon(obs_points, closed=True,
                                         fill=False, edgecolor='red',
                                         linewidth=2, linestyle='-',
                                         label='Original Obstacles' if i == 0 else '')
                ax.add_patch(polygon)

    # Set axis properties
    ax.set_xlim(min_x - 0.5, max_x + 0.5)
    ax.set_ylim(min_y - 0.5, max_y + 0.5)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3, linestyle='--', linewidth=0.5)

    # Plot trajectory if provided
    if trajectory_data is not None:
        plot_trajectory_on_axes(ax, trajectory_data)

    # Labels and title
    ax.set_xlabel('X (m)', fontsize=12)
    ax.set_ylabel('Y (m)', fontsize=12)

    total_cells = len(map_data['cells'])
    title = f'Grid Map with Depth Visualization (Refined)\n'
    title += f'Resolution: {map_data["resolution"]:.2f}m, Total cells: {total_cells}, Max depth: {max_depth}'
    if trajectory_data is not None:
        title += ' + Dubins Path'
    ax.set_title(title, fontsize=14, fontweight='bold')

    # Colorbar for depth
    sm = plt.cm.ScalarMappable(cmap=depth_cmap,
                               norm=plt.Normalize(vmin=0, vmax=max_depth))
    sm.set_array([])
    cbar = plt.colorbar(sm, ax=ax, label='Subdivision Depth')

    plt.tight_layout()

    # Save if output file specified
    if output_file:
        plt.savefig(output_file, dpi=300, bbox_inches='tight')
        print(f"Figure saved to: {output_file}")

    # Show if requested
    if show:
        plt.show()

    return fig, ax


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)

    # Parse arguments
    args = sys.argv[1:]
    input_file = None
    output_file = None
    trajectory_file = None

    i = 0
    while i < len(args):
        if args[i] == '--trajectory':
            if i + 1 < len(args):
                trajectory_file = args[i + 1]
                i += 2
            else:
                print("Error: --trajectory requires a file path")
                sys.exit(1)
        elif input_file is None:
            input_file = args[i]
            i += 1
        elif output_file is None:
            output_file = args[i]
            i += 1
        else:
            print(f"Error: Unexpected argument: {args[i]}")
            print(__doc__)
            sys.exit(1)

    if input_file is None:
        print("Error: Map file is required")
        print(__doc__)
        sys.exit(1)

    if not os.path.exists(input_file):
        print(f"Error: Input file not found: {input_file}")
        sys.exit(1)

    print(f"Loading map from: {input_file}")
    map_data = load_map(input_file)

    print(f"Map info:")
    print(f"  Resolution: {map_data['resolution']:.3f} m")
    print(f"  Grid size: {map_data['grid_size']['width']} x {map_data['grid_size']['height']}")
    print(f"  Refined: {map_data['refined']}")
    print(f"  Total cells: {len(map_data['cells'])}")

    # Count cells by state
    state_counts = {'free': 0, 'occupied': 0, 'mixed': 0}
    for cell in map_data['cells']:
        state_counts[cell['state']] += 1
    print(f"  Free: {state_counts['free']}, Occupied: {state_counts['occupied']}, Mixed: {state_counts['mixed']}")

    # Load trajectory if provided
    trajectory_data = None
    if trajectory_file:
        if not os.path.exists(trajectory_file):
            print(f"Warning: Trajectory file not found: {trajectory_file}")
        else:
            print(f"\nLoading trajectory from: {trajectory_file}")
            trajectory_data = load_trajectory(trajectory_file)
            print(f"Trajectory info:")
            if 'stats' in trajectory_data:
                stats = trajectory_data['stats']
                print(f"  Distance: {stats['total_distance']:.2f} m")
                print(f"  Time: {stats['total_time']:.2f} s")
                print(f"  Victims visited: {stats['num_victims_visited']}")

    # Create both visualizations
    print("\nGenerating standard visualization...")
    plot_map(map_data, output_file, show=False, trajectory_data=trajectory_data)

    if map_data['refined'] and output_file:
        # Also save depth visualization
        base, ext = os.path.splitext(output_file)
        depth_output = f"{base}_depth{ext}"
        print(f"Generating depth visualization...")
        plot_map_with_depth(map_data, depth_output, show=False, trajectory_data=trajectory_data)

    # Show interactively
    plt.show()


if __name__ == "__main__":
    main()
