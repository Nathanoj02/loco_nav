#include "grid_map.hpp"
#include <algorithm>
#include <cmath>
#include <sstream>
#include <iostream>

namespace planning {

// ============================================================================
// Geometry helpers
// ============================================================================

bool pointInPolygon(double px, double py, const geometry_msgs::Polygon& polygon) {
    // Ray casting algorithm
    int n = polygon.points.size();
    if (n < 3) return false;

    bool inside = false;
    for (int i = 0, j = n - 1; i < n; j = i++) {
        double xi = polygon.points[i].x, yi = polygon.points[i].y;
        double xj = polygon.points[j].x, yj = polygon.points[j].y;

        if (((yi > py) != (yj > py)) &&
            (px < (xj - xi) * (py - yi) / (yj - yi) + xi)) {
            inside = !inside;
        }
    }
    return inside;
}

bool segmentsIntersect(double ax, double ay, double bx, double by,
                       double cx, double cy, double dx, double dy) {
    // Check if segment AB intersects segment CD
    auto ccw = [](double ax, double ay, double bx, double by, double cx, double cy) {
        return (cy - ay) * (bx - ax) > (by - ay) * (cx - ax);
    };

    return (ccw(ax, ay, cx, cy, dx, dy) != ccw(bx, by, cx, cy, dx, dy)) &&
           (ccw(ax, ay, bx, by, cx, cy) != ccw(ax, ay, bx, by, dx, dy));
}

bool pointInRect(double px, double py, double x0, double y0, double x1, double y1) {
    return px >= x0 && px <= x1 && py >= y0 && py <= y1;
}

bool polygonIntersectsRect(const geometry_msgs::Polygon& polygon,
                           double x0, double y0, double x1, double y1) {
    int n = polygon.points.size();
    if (n < 3) return false;

    // Rectangle edges
    double rect_edges[4][4] = {
        {x0, y0, x1, y0},  // bottom
        {x1, y0, x1, y1},  // right
        {x1, y1, x0, y1},  // top
        {x0, y1, x0, y0}   // left
    };

    // Check if any polygon edge intersects any rectangle edge
    for (int i = 0; i < n; ++i) {
        int j = (i + 1) % n;
        double px0 = polygon.points[i].x, py0 = polygon.points[i].y;
        double px1 = polygon.points[j].x, py1 = polygon.points[j].y;

        for (int k = 0; k < 4; ++k) {
            if (segmentsIntersect(px0, py0, px1, py1,
                                  rect_edges[k][0], rect_edges[k][1],
                                  rect_edges[k][2], rect_edges[k][3])) {
                return true;
            }
        }
    }

    return false;
}

bool polygonInsideRect(const geometry_msgs::Polygon& polygon,
                       double x0, double y0, double x1, double y1) {
    // Check if any polygon vertex is inside the rectangle
    // (handles case where small obstacle is entirely inside a large cell)
    for (const auto& pt : polygon.points) {
        if (pointInRect(pt.x, pt.y, x0, y0, x1, y1)) {
            return true;
        }
    }
    return false;
}

// ============================================================================
// GridMap implementation
// ============================================================================

GridMap::GridMap(const geometry_msgs::Polygon& borders, double resolution)
    : resolution_(resolution) {
    computeBounds(borders);
    initializeGrid();
}

GridMap::GridMap(const geometry_msgs::Polygon& borders)
    : GridMap(borders, getConfig().grid_resolution) {}

void GridMap::computeBounds(const geometry_msgs::Polygon& borders) {
    if (borders.points.empty()) {
        min_x_ = min_y_ = 0;
        max_x_ = max_y_ = 1;
        return;
    }

    min_x_ = max_x_ = borders.points[0].x;
    min_y_ = max_y_ = borders.points[0].y;

    for (const auto& pt : borders.points) {
        min_x_ = std::min(min_x_, static_cast<double>(pt.x));
        max_x_ = std::max(max_x_, static_cast<double>(pt.x));
        min_y_ = std::min(min_y_, static_cast<double>(pt.y));
        max_y_ = std::max(max_y_, static_cast<double>(pt.y));
    }

    // Add small padding to ensure border cells are included
    double pad = resolution_ * 0.5;
    min_x_ -= pad;
    min_y_ -= pad;
    max_x_ += pad;
    max_y_ += pad;
}

void GridMap::initializeGrid() {
    width_ = static_cast<int>(std::ceil((max_x_ - min_x_) / resolution_));
    height_ = static_cast<int>(std::ceil((max_y_ - min_y_) / resolution_));

    // Ensure minimum size
    width_ = std::max(width_, 1);
    height_ = std::max(height_, 1);

    // Initialize all cells as FREE
    grid_.assign(height_, std::vector<CellState>(width_, CellState::FREE));
}

bool GridMap::worldToGrid(double wx, double wy, int& gx, int& gy) const {
    gx = static_cast<int>((wx - min_x_) / resolution_);
    gy = static_cast<int>((wy - min_y_) / resolution_);
    return isInBounds(gx, gy);
}

void GridMap::gridToWorld(int gx, int gy, double& wx, double& wy) const {
    // Return cell center
    wx = min_x_ + (gx + 0.5) * resolution_;
    wy = min_y_ + (gy + 0.5) * resolution_;
}

CellState GridMap::getCell(int gx, int gy) const {
    if (!isInBounds(gx, gy)) return CellState::OCCUPIED;
    return grid_[gy][gx];
}

void GridMap::setCell(int gx, int gy, CellState state) {
    if (isInBounds(gx, gy)) {
        grid_[gy][gx] = state;
    }
}

bool GridMap::isFree(int gx, int gy) const {
    return getCell(gx, gy) == CellState::FREE;
}

bool GridMap::isOccupied(int gx, int gy) const {
    return getCell(gx, gy) == CellState::OCCUPIED;
}

bool GridMap::isMixed(int gx, int gy) const {
    return getCell(gx, gy) == CellState::MIXED;
}

bool GridMap::isInBounds(int gx, int gy) const {
    return gx >= 0 && gx < width_ && gy >= 0 && gy < height_;
}

void GridMap::getCellCorners(int gx, int gy,
                              double& x0, double& y0,
                              double& x1, double& y1) const {
    x0 = min_x_ + gx * resolution_;
    y0 = min_y_ + gy * resolution_;
    x1 = x0 + resolution_;
    y1 = y0 + resolution_;
}

CellState GridMap::classifyCellVsPolygon(int gx, int gy,
                                          const geometry_msgs::Polygon& polygon) const {
    double x0, y0, x1, y1;
    getCellCorners(gx, gy, x0, y0, x1, y1);

    // Check all four corners of cell vs polygon (obstacle)
    bool corners[4];
    corners[0] = pointInPolygon(x0, y0, polygon);
    corners[1] = pointInPolygon(x1, y0, polygon);
    corners[2] = pointInPolygon(x1, y1, polygon);
    corners[3] = pointInPolygon(x0, y1, polygon);

    int inside_count = corners[0] + corners[1] + corners[2] + corners[3];

    if (inside_count == 4) {
        return CellState::OCCUPIED;  // Cell fully inside obstacle
    } else if (inside_count == 0) {
        // All cell corners outside obstacle - but check:
        // 1. Obstacle edge might cross the cell
        // 2. Obstacle might be entirely inside the cell
        if (polygonIntersectsRect(polygon, x0, y0, x1, y1) ||
            polygonInsideRect(polygon, x0, y0, x1, y1)) {
            return CellState::MIXED;
        }
        return CellState::FREE;  // Truly free
    } else {
        return CellState::MIXED;  // Partially inside
    }
}

void GridMap::markObstacles(const std::vector<geometry_msgs::Polygon>& obstacles) {
    for (const auto& obstacle : obstacles) {
        if (obstacle.points.size() < 3) continue;

        // Compute obstacle bounding box for efficiency
        double obs_min_x = obstacle.points[0].x, obs_max_x = obstacle.points[0].x;
        double obs_min_y = obstacle.points[0].y, obs_max_y = obstacle.points[0].y;
        for (const auto& pt : obstacle.points) {
            obs_min_x = std::min(obs_min_x, static_cast<double>(pt.x));
            obs_max_x = std::max(obs_max_x, static_cast<double>(pt.x));
            obs_min_y = std::min(obs_min_y, static_cast<double>(pt.y));
            obs_max_y = std::max(obs_max_y, static_cast<double>(pt.y));
        }

        // Convert to grid coordinates (with margin)
        int gx_min, gy_min, gx_max, gy_max;
        worldToGrid(obs_min_x - resolution_, obs_min_y - resolution_, gx_min, gy_min);
        worldToGrid(obs_max_x + resolution_, obs_max_y + resolution_, gx_max, gy_max);

        gx_min = std::max(0, gx_min);
        gy_min = std::max(0, gy_min);
        gx_max = std::min(width_ - 1, gx_max);
        gy_max = std::min(height_ - 1, gy_max);

        // Check cells in bounding box
        for (int gy = gy_min; gy <= gy_max; ++gy) {
            for (int gx = gx_min; gx <= gx_max; ++gx) {
                CellState current = getCell(gx, gy);
                if (current == CellState::OCCUPIED) continue;  // Already occupied

                CellState vs_obstacle = classifyCellVsPolygon(gx, gy, obstacle);

                if (vs_obstacle == CellState::OCCUPIED) {
                    setCell(gx, gy, CellState::OCCUPIED);
                } else if (vs_obstacle == CellState::MIXED) {
                    // Only upgrade FREE to MIXED, don't downgrade OCCUPIED
                    if (current == CellState::FREE) {
                        setCell(gx, gy, CellState::MIXED);
                    }
                }
            }
        }
    }
}

void GridMap::markOutsideBorders(const geometry_msgs::Polygon& shrunk_borders) {
    if (shrunk_borders.points.size() < 3) return;

    for (int gy = 0; gy < height_; ++gy) {
        for (int gx = 0; gx < width_; ++gx) {
            CellState current = getCell(gx, gy);
            if (current == CellState::OCCUPIED) continue;

            double x0, y0, x1, y1;
            getCellCorners(gx, gy, x0, y0, x1, y1);

            // Check corners - for borders, INSIDE polygon means FREE
            bool corners[4];
            corners[0] = pointInPolygon(x0, y0, shrunk_borders);
            corners[1] = pointInPolygon(x1, y0, shrunk_borders);
            corners[2] = pointInPolygon(x1, y1, shrunk_borders);
            corners[3] = pointInPolygon(x0, y1, shrunk_borders);

            int inside_count = corners[0] + corners[1] + corners[2] + corners[3];

            if (inside_count == 0) {
                // All corners outside borders
                if (polygonIntersectsRect(shrunk_borders, x0, y0, x1, y1)) {
                    setCell(gx, gy, CellState::MIXED);
                } else {
                    setCell(gx, gy, CellState::OCCUPIED);
                }
            } else if (inside_count < 4) {
                // Partially outside
                if (current == CellState::FREE) {
                    setCell(gx, gy, CellState::MIXED);
                }
            }
            // If all 4 corners inside, cell remains FREE (or MIXED from obstacles)
        }
    }
}

void GridMap::getCellCounts(int& free_count, int& occupied_count, int& mixed_count) const {
    free_count = occupied_count = mixed_count = 0;
    for (int gy = 0; gy < height_; ++gy) {
        for (int gx = 0; gx < width_; ++gx) {
            switch (grid_[gy][gx]) {
                case CellState::FREE: ++free_count; break;
                case CellState::OCCUPIED: ++occupied_count; break;
                case CellState::MIXED: ++mixed_count; break;
            }
        }
    }
}

std::string GridMap::getSummary() const {
    int free_count, occupied_count, mixed_count;
    getCellCounts(free_count, occupied_count, mixed_count);

    std::ostringstream ss;
    ss << "GridMap: " << width_ << "x" << height_
       << " cells (resolution: " << resolution_ << "m)\n"
       << "World bounds: [" << min_x_ << ", " << min_y_ << "] to ["
       << max_x_ << ", " << max_y_ << "]\n"
       << "Cells: " << free_count << " FREE, "
       << occupied_count << " OCCUPIED, "
       << mixed_count << " MIXED";
    return ss.str();
}

void GridMap::printASCII(int max_display_width) const {
    // Compute downsampling factor if needed (0 = no limit)
    int step = 1;
    if (max_display_width > 0 && width_ > max_display_width) {
        step = (width_ + max_display_width - 1) / max_display_width;
    }

    int display_width = (width_ + step - 1) / step;
    int display_height = (height_ + step - 1) / step;

    std::cout << getSummary() << "\n";
    if (step > 1) {
        std::cout << "(Downsampled " << step << "x for display)\n";
    }
    std::cout << "\n";

    // Print from top to bottom (high Y to low Y)
    for (int dy = display_height - 1; dy >= 0; --dy) {
        for (int dx = 0; dx < display_width; ++dx) {
            int gx = dx * step;
            int gy = dy * step;

            // For downsampled display, show worst case in the block
            CellState state = CellState::FREE;
            for (int offy = 0; offy < step && (gy + offy) < height_; ++offy) {
                for (int offx = 0; offx < step && (gx + offx) < width_; ++offx) {
                    CellState s = getCell(gx + offx, gy + offy);
                    if (s == CellState::OCCUPIED) {
                        state = CellState::OCCUPIED;
                    } else if (s == CellState::MIXED && state == CellState::FREE) {
                        state = CellState::MIXED;
                    }
                }
            }

            switch (state) {
                case CellState::FREE: std::cout << '.'; break;
                case CellState::OCCUPIED: std::cout << '#'; break;
                case CellState::MIXED: std::cout << '~'; break;
            }
        }
        std::cout << '\n';
    }
    std::cout << std::flush;
}

CellState GridMap::classifyRectVsPolygon(double x0, double y0, double x1, double y1,
                                          const geometry_msgs::Polygon& polygon) const {
    // Check all four corners of rect vs polygon (obstacle)
    bool corners[4];
    corners[0] = pointInPolygon(x0, y0, polygon);
    corners[1] = pointInPolygon(x1, y0, polygon);
    corners[2] = pointInPolygon(x1, y1, polygon);
    corners[3] = pointInPolygon(x0, y1, polygon);

    int inside_count = corners[0] + corners[1] + corners[2] + corners[3];

    if (inside_count == 4) {
        return CellState::OCCUPIED;  // Rect fully inside obstacle
    } else if (inside_count == 0) {
        // All rect corners outside obstacle - but check:
        // 1. Obstacle edge might cross the rect
        // 2. Obstacle might be entirely inside the rect
        if (polygonIntersectsRect(polygon, x0, y0, x1, y1) ||
            polygonInsideRect(polygon, x0, y0, x1, y1)) {
            return CellState::MIXED;
        }
        return CellState::FREE;
    } else {
        return CellState::MIXED;
    }
}

CellState GridMap::classifyRect(double x0, double y0, double x1, double y1,
                                 const std::vector<geometry_msgs::Polygon>& obstacles,
                                 const geometry_msgs::Polygon& shrunk_borders) const {
    // First check against borders (must be INSIDE borders to be FREE)
    bool border_corners[4];
    border_corners[0] = pointInPolygon(x0, y0, shrunk_borders);
    border_corners[1] = pointInPolygon(x1, y0, shrunk_borders);
    border_corners[2] = pointInPolygon(x1, y1, shrunk_borders);
    border_corners[3] = pointInPolygon(x0, y1, shrunk_borders);

    int inside_border_count = border_corners[0] + border_corners[1] +
                               border_corners[2] + border_corners[3];

    if (inside_border_count == 0) {
        // Fully outside borders
        if (polygonIntersectsRect(shrunk_borders, x0, y0, x1, y1)) {
            return CellState::MIXED;
        }
        return CellState::OCCUPIED;
    } else if (inside_border_count < 4) {
        // Partially outside borders
        return CellState::MIXED;
    }

    // All corners inside borders - now check obstacles
    for (const auto& obstacle : obstacles) {
        if (obstacle.points.size() < 3) continue;

        CellState vs_obs = classifyRectVsPolygon(x0, y0, x1, y1, obstacle);
        if (vs_obs == CellState::OCCUPIED) {
            return CellState::OCCUPIED;
        } else if (vs_obs == CellState::MIXED) {
            return CellState::MIXED;
        }
    }

    return CellState::FREE;
}

void GridMap::subdivideCell(double x0, double y0, double x1, double y1,
                             const std::vector<geometry_msgs::Polygon>& obstacles,
                             const geometry_msgs::Polygon& shrunk_borders,
                             int depth, int max_depth) {
    CellState state = classifyRect(x0, y0, x1, y1, obstacles, shrunk_borders);

    // If FREE or OCCUPIED, store as leaf cell
    if (state == CellState::FREE || state == CellState::OCCUPIED) {
        leaf_cells_.push_back({x0, y0, x1, y1, state, depth});
        return;
    }

    // MIXED cell - check if we can subdivide further
    if (depth >= max_depth) {
        // At max depth, treat MIXED as OCCUPIED (conservative)
        leaf_cells_.push_back({x0, y0, x1, y1, CellState::OCCUPIED, depth});
        return;
    }

    // Subdivide into 4 quadrants
    double mx = (x0 + x1) / 2.0;
    double my = (y0 + y1) / 2.0;

    subdivideCell(x0, y0, mx, my, obstacles, shrunk_borders, depth + 1, max_depth);  // bottom-left
    subdivideCell(mx, y0, x1, my, obstacles, shrunk_borders, depth + 1, max_depth);  // bottom-right
    subdivideCell(x0, my, mx, y1, obstacles, shrunk_borders, depth + 1, max_depth);  // top-left
    subdivideCell(mx, my, x1, y1, obstacles, shrunk_borders, depth + 1, max_depth);  // top-right
}

void GridMap::refineMixedCells(const std::vector<geometry_msgs::Polygon>& obstacles,
                                const geometry_msgs::Polygon& shrunk_borders,
                                int max_depth) {
    // Clear any existing leaf cells
    leaf_cells_.clear();

    int mixed_count = 0;

    // Process all grid cells
    for (int gy = 0; gy < height_; ++gy) {
        for (int gx = 0; gx < width_; ++gx) {
            double x0, y0, x1, y1;
            getCellCorners(gx, gy, x0, y0, x1, y1);

            CellState state = getCell(gx, gy);

            if (state == CellState::MIXED) {
                // Subdivide MIXED cells recursively
                ++mixed_count;
                subdivideCell(x0, y0, x1, y1, obstacles, shrunk_borders, 0, max_depth);
            } else {
                // FREE and OCCUPIED cells are stored directly as leaf cells
                leaf_cells_.push_back({x0, y0, x1, y1, state, 0});
            }
        }
    }

    cells_refined_ = true;

    // Count results
    int free_count = 0, occupied_count = 0;
    for (const auto& cell : leaf_cells_) {
        if (cell.state == CellState::FREE) ++free_count;
        else ++occupied_count;
    }

    std::cout << "Approximate Cell Decomposition complete:\n"
              << "  Original grid: " << width_ << "x" << height_ << " = " << (width_ * height_) << " cells\n"
              << "  MIXED cells subdivided: " << mixed_count << " (max depth=" << max_depth << ")\n"
              << "  Leaf cells: " << leaf_cells_.size() << " total ("
              << free_count << " FREE, " << occupied_count << " OCCUPIED)" << std::endl;
}

std::vector<const Cell*> GridMap::getFreeCells() const {
    std::vector<const Cell*> free_cells;
    for (const auto& cell : leaf_cells_) {
        if (cell.state == CellState::FREE) {
            free_cells.push_back(&cell);
        }
    }
    return free_cells;
}

}  // namespace planning
