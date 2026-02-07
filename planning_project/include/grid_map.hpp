#ifndef PLANNING_PROJECT_GRID_MAP_HPP
#define PLANNING_PROJECT_GRID_MAP_HPP

#include <vector>
#include <string>
#include <cstdint>
#include <geometry_msgs/Polygon.h>
#include "config.hpp"

namespace planning {

enum class CellState : uint8_t {
    FREE = 0,      // Cell fully in free space
    OCCUPIED = 1,  // Cell fully inside obstacle
    MIXED = 2      // Cell partially overlaps obstacle (boundary)
};

/**
 * @brief A rectangular cell with bounds and state.
 * Used for storing subdivided cells in approximate cell decomposition.
 */
struct Cell {
    double x0, y0;  // bottom-left corner
    double x1, y1;  // top-right corner
    CellState state;
    int depth;      // subdivision depth (0 = original grid cell)

    double centerX() const { return (x0 + x1) / 2.0; }
    double centerY() const { return (y0 + y1) / 2.0; }
    double width() const { return x1 - x0; }
    double height() const { return y1 - y0; }
};

/**
 * @brief 2D occupancy grid for approximate cell decomposition.
 */
class GridMap {
public:
    /**
     * @brief Construct grid from map borders.
     *
     * Computes bounding box from borders and creates grid with configured resolution.
     *
     * @param borders The (original, not shrunk) map border polygon
     * @param resolution Cell size in meters (default from config)
     */
    GridMap(const geometry_msgs::Polygon& borders, double resolution);
    GridMap(const geometry_msgs::Polygon& borders);

    // Grid dimensions
    int width() const { return width_; }
    int height() const { return height_; }
    double resolution() const { return resolution_; }

    // World bounds
    double minX() const { return min_x_; }
    double minY() const { return min_y_; }
    double maxX() const { return max_x_; }
    double maxY() const { return max_y_; }

    // Coordinate conversion
    bool worldToGrid(double wx, double wy, int& gx, int& gy) const;
    void gridToWorld(int gx, int gy, double& wx, double& wy) const;

    // Cell access
    CellState getCell(int gx, int gy) const;
    void setCell(int gx, int gy, CellState state);
    bool isFree(int gx, int gy) const;
    bool isOccupied(int gx, int gy) const;
    bool isMixed(int gx, int gy) const;
    bool isInBounds(int gx, int gy) const;

    /**
     * @brief Mark cells based on obstacle polygons.
     *
     * Cells fully inside obstacles -> OCCUPIED
     * Cells intersecting obstacle boundary -> MIXED
     * Other cells remain FREE
     *
     * @param obstacles Vector of inflated obstacle polygons
     */
    void markObstacles(const std::vector<geometry_msgs::Polygon>& obstacles);

    /**
     * @brief Mark cells outside the border polygon.
     *
     * Cells fully outside borders -> OCCUPIED
     * Cells intersecting border boundary -> MIXED
     *
     * @param shrunk_borders The shrunk (inward offset) border polygon
     */
    void markOutsideBorders(const geometry_msgs::Polygon& shrunk_borders);

    /**
     * @brief Get count of cells by state.
     */
    void getCellCounts(int& free_count, int& occupied_count, int& mixed_count) const;

    /**
     * @brief Refine MIXED cells by recursive subdivision.
     *
     * For each MIXED cell, subdivides into 4 quadrants and stores them as separate cells.
     * Continues recursively until max_depth or cell is fully FREE/OCCUPIED.
     * After max_depth, remaining MIXED cells are marked OCCUPIED (conservative).
     *
     * After calling this, use getLeafCells() to get all cells for pathfinding.
     *
     * @param obstacles Inflated obstacle polygons (for reclassification)
     * @param shrunk_borders Shrunk border polygon (for reclassification)
     * @param max_depth Maximum subdivision depth (e.g., 4)
     */
    void refineMixedCells(const std::vector<geometry_msgs::Polygon>& obstacles,
                          const geometry_msgs::Polygon& shrunk_borders,
                          int max_depth);

    /**
     * @brief Get all leaf cells after subdivision.
     *
     * Returns all FREE and OCCUPIED cells (no MIXED after refinement).
     * These are the cells to use for pathfinding.
     */
    const std::vector<Cell>& getLeafCells() const { return leaf_cells_; }

    /**
     * @brief Getters for map properties.
     */
    double getResolution() const { return resolution_; }
    std::array<double, 4> getBounds() const { return {min_x_, min_y_, max_x_, max_y_}; }
    std::pair<int, int> getGridSize() const { return {width_, height_}; }
    bool isRefined() const { return cells_refined_; }

    /**
     * @brief Save map to file for visualization.
     *
     * Saves grid map data (leaf cells with state, bounds, depth) to a JSON file.
     * File can be read by Python scripts for plotting.
     *
     * @param filename Path to output file (e.g., "results/map.json")
     * @return true if successful, false otherwise
     */
    bool saveToFile(const std::string& filename) const;

private:
    double resolution_;
    double min_x_, min_y_, max_x_, max_y_;
    int width_, height_;
    std::vector<std::vector<CellState>> grid_;

    // Leaf cells after subdivision (for pathfinding)
    std::vector<Cell> leaf_cells_;
    bool cells_refined_ = false;

    void computeBounds(const geometry_msgs::Polygon& borders);
    void initializeGrid();

    // Get cell corner coordinates
    void getCellCorners(int gx, int gy,
                        double& x0, double& y0,
                        double& x1, double& y1) const;

    // Classify cell vs polygon relationship
    CellState classifyCellVsPolygon(int gx, int gy,
                                     const geometry_msgs::Polygon& polygon) const;

    // Classify arbitrary rectangle vs polygon
    CellState classifyRectVsPolygon(double x0, double y0, double x1, double y1,
                                     const geometry_msgs::Polygon& polygon) const;

    // Classify rectangle against all obstacles and borders
    CellState classifyRect(double x0, double y0, double x1, double y1,
                           const std::vector<geometry_msgs::Polygon>& obstacles,
                           const geometry_msgs::Polygon& shrunk_borders) const;

    // Recursive subdivision helper - populates leaf_cells_
    void subdivideCell(double x0, double y0, double x1, double y1,
                       const std::vector<geometry_msgs::Polygon>& obstacles,
                       const geometry_msgs::Polygon& shrunk_borders,
                       int depth, int max_depth);
};

// Geometry helpers
bool pointInPolygon(double px, double py, const geometry_msgs::Polygon& polygon);
bool segmentsIntersect(double ax, double ay, double bx, double by,
                       double cx, double cy, double dx, double dy);
bool polygonIntersectsRect(const geometry_msgs::Polygon& polygon,
                           double x0, double y0, double x1, double y1);

}  // namespace planning

#endif  // PLANNING_PROJECT_GRID_MAP_HPP
