#!/usr/bin/env python3
"""
Visualize Informed RRT* tree and path from JSON file.

Usage:
    python3 plot_rrt.py ../results/rrt_tree.json
    python3 plot_rrt.py ../results/rrt_tree.json output.png
    python3 plot_rrt.py ../results/rrt_tree.json --show-all-edges
    python3 plot_rrt.py ../results/rrt_tree.json --trajectory ../results/trajectory.json
"""

import json
import sys
import os
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.collections import LineCollection
import numpy as np

from plot_map import load_trajectory, plot_trajectory_on_axes


def load_rrt_data(filename):
    """Load RRT* data from JSON file."""
    with open(filename, 'r') as f:
        return json.load(f)


def plot_rrt(data, output_file=None, show_all_edges=True, show_smoothed=True, trajectory_data=None):
    """Plot RRT* tree visualization."""
    fig, ax = plt.subplots(1, 1, figsize=(14, 10))

    bounds = data['bounds']
    ax.set_xlim(bounds['min_x'] - 0.5, bounds['max_x'] + 0.5)
    ax.set_ylim(bounds['min_y'] - 0.5, bounds['max_y'] + 0.5)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title('Informed RRT* Path Planning')

    # Plot borders (world boundaries) if available
    if 'border' in data:
        border = data['border']
        if len(border) >= 3:
            border_polygon = patches.Polygon(border, closed=True,
                                            fill=False, edgecolor='black',
                                            linewidth=3, linestyle='--',
                                            label='World Border')
            ax.add_patch(border_polygon)

    # Plot obstacles (inflated)
    for i, obs_points in enumerate(data.get('obstacles', [])):
        if len(obs_points) >= 3:
            polygon = patches.Polygon(obs_points, closed=True,
                                       facecolor='gray', edgecolor='black',
                                       alpha=0.7, linewidth=1,
                                       label='Inflated Obstacles' if i == 0 else '')
            ax.add_patch(polygon)

    # Plot original obstacles (non-inflated outlines)
    if 'original_obstacles' in data:
        for i, obs_points in enumerate(data['original_obstacles']):
            if len(obs_points) >= 3:
                polygon = patches.Polygon(obs_points, closed=True,
                                         fill=False, edgecolor='red',
                                         linewidth=2, linestyle='-',
                                         label='Original Obstacles' if i == 0 else '')
                ax.add_patch(polygon)

    # Color map for different segments
    colors = plt.cm.tab10(np.linspace(0, 1, len(data['segments'])))

    # Statistics
    total_nodes = 0
    total_edges = 0

    # Plot each segment
    for seg_idx, segment in enumerate(data['segments']):
        color = colors[seg_idx]
        start = segment['start']
        goal = segment['goal']

        # Plot tree edges (light gray for exploration)
        if show_all_edges and segment.get('edges'):
            edges = segment['edges']
            total_edges += len(edges)

            # Use LineCollection for efficiency
            lines = [[(e[0][0], e[0][1]), (e[1][0], e[1][1])] for e in edges]
            lc = LineCollection(lines, colors=[(0.7, 0.7, 0.7, 0.3)], linewidths=0.5)
            ax.add_collection(lc)

        total_nodes += segment.get('num_nodes', 0)

        # Plot raw RRT* path
        if segment.get('path') and len(segment['path']) > 1:
            path = np.array(segment['path'])
            ax.plot(path[:, 0], path[:, 1], '-', color=color, linewidth=1.5,
                    alpha=0.5)  # No label - will be shown by Dubins overlay

        # Plot smoothed path (thicker)
        if show_smoothed and segment.get('smoothed') and len(segment['smoothed']) > 1:
            smoothed = np.array(segment['smoothed'])
            ax.plot(smoothed[:, 0], smoothed[:, 1], '-', color=color,
                    linewidth=3, alpha=0.9)  # No label - will be shown by Dubins overlay

        # Plot start and goal of segment
        if segment['direct']:
            # Direct connection (no RRT needed)
            ax.plot([start[0], goal[0]], [start[1], goal[1]], '--',
                    color='green', linewidth=2, alpha=0.7)

    # Only show route points if no trajectory overlay (avoid duplication)
    if trajectory_data is None:
        # Plot route points
        route = np.array(data['route'])
        ax.plot(route[:, 0], route[:, 1], 'o', color='blue', markersize=10, zorder=5)

        # Mark start and goal
        ax.plot(route[0, 0], route[0, 1], 's', color='green', markersize=15,
                label='Start', zorder=6)
        ax.plot(route[-1, 0], route[-1, 1], '*', color='red', markersize=20,
                label='Goal', zorder=6)

        # Mark intermediate waypoints (victims)
        for i in range(1, len(route) - 1):
            ax.plot(route[i, 0], route[i, 1], 'o', color='orange', markersize=12, zorder=5)
            ax.annotate(f'V{i}', (route[i, 0], route[i, 1]), textcoords="offset points",
                       xytext=(5, 5), fontsize=9, color='orange')

    # Overlay Dubins trajectory if provided
    if trajectory_data is not None:
        plot_trajectory_on_axes(ax, trajectory_data, show_labels=True)

    # Statistics box
    stats_text = f'Segments: {len(data["segments"])}\n'
    stats_text += f'Total nodes: {total_nodes}\n'
    stats_text += f'Total edges: {total_edges}'

    props = dict(boxstyle='round', facecolor='wheat', alpha=0.8)
    ax.text(0.02, 0.98, stats_text, transform=ax.transAxes, fontsize=10,
            verticalalignment='top', bbox=props)

    # Legend
    ax.legend(loc='upper right')

    plt.tight_layout()

    if output_file:
        plt.savefig(output_file, dpi=150, bbox_inches='tight')
        print(f"Saved to {output_file}")
    else:
        plt.show()


def plot_rrt_detailed(data, output_file=None):
    """Plot detailed view with subplots for each segment."""
    n_segments = len(data['segments'])
    if n_segments == 0:
        print("No segments to plot")
        return

    # Create subplot grid
    cols = min(3, n_segments)
    rows = (n_segments + cols - 1) // cols

    fig, axes = plt.subplots(rows, cols, figsize=(5 * cols, 5 * rows))
    if n_segments == 1:
        axes = np.array([axes])
    axes = axes.flatten()

    bounds = data['bounds']

    for seg_idx, segment in enumerate(data['segments']):
        ax = axes[seg_idx]
        start = segment['start']
        goal = segment['goal']

        # Set bounds with margin around segment
        margin = 1.0
        seg_min_x = min(start[0], goal[0]) - margin
        seg_max_x = max(start[0], goal[0]) + margin
        seg_min_y = min(start[1], goal[1]) - margin
        seg_max_y = max(start[1], goal[1]) + margin

        ax.set_xlim(seg_min_x, seg_max_x)
        ax.set_ylim(seg_min_y, seg_max_y)
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)

        # Plot obstacles
        for obs_points in data.get('obstacles', []):
            if len(obs_points) >= 3:
                polygon = patches.Polygon(obs_points, closed=True,
                                          facecolor='gray', edgecolor='black',
                                          alpha=0.5, linewidth=1)
                ax.add_patch(polygon)

        # Plot tree edges
        if segment.get('edges'):
            for edge in segment['edges']:
                ax.plot([edge[0][0], edge[1][0]], [edge[0][1], edge[1][1]],
                       '-', color='lightblue', linewidth=0.5, alpha=0.5)

        # Plot raw path
        if segment.get('path') and len(segment['path']) > 1:
            path = np.array(segment['path'])
            ax.plot(path[:, 0], path[:, 1], '-', color='blue', linewidth=2,
                   alpha=0.7, label='Raw')

        # Plot smoothed path
        if segment.get('smoothed') and len(segment['smoothed']) > 1:
            smoothed = np.array(segment['smoothed'])
            ax.plot(smoothed[:, 0], smoothed[:, 1], '-', color='red',
                   linewidth=3, alpha=0.9, label='Smoothed')

        # Plot start and goal
        ax.plot(start[0], start[1], 's', color='green', markersize=12)
        ax.plot(goal[0], goal[1], '*', color='red', markersize=15)

        # Title with stats
        direct_str = "DIRECT" if segment['direct'] else f"RRT* ({segment.get('num_nodes', 0)} nodes)"
        ax.set_title(f'Segment {seg_idx}: {direct_str}\nCost: {segment.get("cost", 0):.2f}m')
        ax.legend(loc='upper right', fontsize=8)

    # Hide unused subplots
    for idx in range(n_segments, len(axes)):
        axes[idx].set_visible(False)

    plt.tight_layout()

    if output_file:
        base = output_file.rsplit('.', 1)[0]
        detailed_file = f"{base}_detailed.png"
        plt.savefig(detailed_file, dpi=150, bbox_inches='tight')
        print(f"Saved detailed view to {detailed_file}")
    else:
        plt.show()


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 plot_rrt.py <rrt_tree.json> [output.png] [--show-all-edges] [--detailed] [--trajectory trajectory.json]")
        sys.exit(1)

    json_file = sys.argv[1]
    output_file = None
    show_all_edges = True
    detailed = False
    trajectory_file = None

    i = 2
    while i < len(sys.argv):
        arg = sys.argv[i]
        if arg == '--show-all-edges':
            show_all_edges = True
        elif arg == '--hide-edges':
            show_all_edges = False
        elif arg == '--detailed':
            detailed = True
        elif arg == '--trajectory':
            if i + 1 < len(sys.argv):
                trajectory_file = sys.argv[i + 1]
                i += 1
            else:
                print("Error: --trajectory requires a file path")
                sys.exit(1)
        elif not arg.startswith('--'):
            output_file = arg
        i += 1

    # Load trajectory if provided
    trajectory_data = None
    if trajectory_file:
        if not os.path.exists(trajectory_file):
            print(f"Warning: Trajectory file not found: {trajectory_file}")
        else:
            print(f"Loading trajectory from: {trajectory_file}")
            trajectory_data = load_trajectory(trajectory_file)
            if 'stats' in trajectory_data:
                stats = trajectory_data['stats']
                print(f"  Distance: {stats['total_distance']:.2f} m, "
                      f"Time: {stats['total_time']:.2f} s, "
                      f"Victims: {stats['num_victims_visited']}")

    try:
        data = load_rrt_data(json_file)
        print(f"Loaded RRT* data: {len(data['segments'])} segments")

        plot_rrt(data, output_file, show_all_edges=show_all_edges,
                 trajectory_data=trajectory_data)

        if detailed:
            plot_rrt_detailed(data, output_file)

    except FileNotFoundError:
        print(f"Error: File not found: {json_file}")
        sys.exit(1)
    except json.JSONDecodeError as e:
        print(f"Error: Invalid JSON: {e}")
        sys.exit(1)


if __name__ == '__main__':
    main()
