#include "geometry_utils.hpp"
#include <cmath>

namespace planning {

namespace {

struct Vec2 {
    double x, y;

    Vec2(double x = 0, double y = 0) : x(x), y(y) {}

    Vec2 operator+(const Vec2& other) const { return Vec2(x + other.x, y + other.y); }
    Vec2 operator-(const Vec2& other) const { return Vec2(x - other.x, y - other.y); }
    Vec2 operator*(double s) const { return Vec2(x * s, y * s); }

    double length() const { return std::sqrt(x * x + y * y); }

    Vec2 normalized() const {
        double len = length();
        if (len < 1e-9) return Vec2(0, 0);
        return Vec2(x / len, y / len);
    }

    // Perpendicular vector (rotated 90 degrees CCW)
    Vec2 perp() const { return Vec2(-y, x); }
};

// Compute intersection of two lines defined by point + direction
// Line 1: p1 + t * d1
// Line 2: p2 + s * d2
// Returns the intersection point
Vec2 lineIntersection(const Vec2& p1, const Vec2& d1, const Vec2& p2, const Vec2& d2) {
    double cross = d1.x * d2.y - d1.y * d2.x;

    if (std::abs(cross) < 1e-9) {
        // Lines are parallel, return midpoint as fallback
        return Vec2((p1.x + p2.x) / 2, (p1.y + p2.y) / 2);
    }

    Vec2 diff = p2 - p1;
    double t = (diff.x * d2.y - diff.y * d2.x) / cross;

    return p1 + d1 * t;
}

// Compute signed area of polygon (positive = CCW, negative = CW)
double polygonSignedArea(const geometry_msgs::Polygon& polygon) {
    double area = 0.0;
    size_t n = polygon.points.size();
    for (size_t i = 0; i < n; ++i) {
        size_t j = (i + 1) % n;
        area += polygon.points[i].x * polygon.points[j].y;
        area -= polygon.points[j].x * polygon.points[i].y;
    }
    return area / 2.0;
}

}  // anonymous namespace

geometry_msgs::Polygon offsetPolygon(const geometry_msgs::Polygon& polygon, double offset) {
    geometry_msgs::Polygon result;

    size_t n = polygon.points.size();
    if (n < 3) {
        return result;  // Not a valid polygon
    }

    // Determine winding order: CCW = positive area, CW = negative area
    double signed_area = polygonSignedArea(polygon);
    // perp() gives LEFT normal. For CCW polygons, left = inward, so negate to expand outward
    // For CW polygons, left = outward, so keep positive to expand
    double actual_offset = (signed_area > 0) ? -offset : offset;

    // Compute offset edges and find their intersections
    std::vector<Vec2> offsetPoints;

    for (size_t i = 0; i < n; ++i) {
        // Current vertex and neighbors
        size_t prev = (i + n - 1) % n;
        size_t next = (i + 1) % n;

        Vec2 p0(polygon.points[prev].x, polygon.points[prev].y);
        Vec2 p1(polygon.points[i].x, polygon.points[i].y);
        Vec2 p2(polygon.points[next].x, polygon.points[next].y);

        // Edge directions
        Vec2 d1 = (p1 - p0).normalized();
        Vec2 d2 = (p2 - p1).normalized();

        // Outward normals (perpendicular, pointing outward for CCW polygon)
        Vec2 n1 = d1.perp();
        Vec2 n2 = d2.perp();

        // Offset edge start points (using actual_offset which accounts for winding)
        Vec2 offset1 = p0 + n1 * actual_offset;
        Vec2 offset2 = p1 + n2 * actual_offset;

        // Find intersection of the two offset edges
        Vec2 intersection = lineIntersection(offset1, d1, offset2, d2);
        offsetPoints.push_back(intersection);
    }

    // Convert back to geometry_msgs::Polygon
    result.points.resize(offsetPoints.size());
    for (size_t i = 0; i < offsetPoints.size(); ++i) {
        result.points[i].x = offsetPoints[i].x;
        result.points[i].y = offsetPoints[i].y;
        result.points[i].z = polygon.points[i].z;
    }

    return result;
}

geometry_msgs::Polygon circleToPolygon(
    double center_x,
    double center_y,
    double radius,
    int num_vertices) {

    geometry_msgs::Polygon result;
    result.points.resize(num_vertices);

    for (int i = 0; i < num_vertices; ++i) {
        double angle = 2.0 * M_PI * i / num_vertices;
        result.points[i].x = center_x + radius * std::cos(angle);
        result.points[i].y = center_y + radius * std::sin(angle);
        result.points[i].z = 0.0;
    }

    return result;
}

bool isCircleObstacle(const obstacles_msgs::ObstacleMsg& obstacle) {
    // Circle obstacles have radius > 0 and typically 1 point (center)
    // or sometimes the polygon is empty and only radius + center are used
    return obstacle.radius > 0.0;
}

geometry_msgs::Polygon inflateObstacle(
    const obstacles_msgs::ObstacleMsg& obstacle,
    double inflation) {

    if (isCircleObstacle(obstacle)) {
        // For circles: get center and inflate radius directly
        double cx = 0.0, cy = 0.0;

        if (!obstacle.polygon.points.empty()) {
            // Center is the first (and usually only) point
            cx = obstacle.polygon.points[0].x;
            cy = obstacle.polygon.points[0].y;
        }

        double inflated_radius = obstacle.radius + inflation;
        return circleToPolygon(cx, cy, inflated_radius);
    } else {
        // For polygons: use edge-normal offset method
        return offsetPolygon(obstacle.polygon, inflation);
    }
}

geometry_msgs::Polygon inflateObstacle(const obstacles_msgs::ObstacleMsg& obstacle) {
    return inflateObstacle(obstacle, getConfig().totalInflation());
}

std::vector<geometry_msgs::Polygon> inflateObstacles(
    const obstacles_msgs::ObstacleArrayMsg& msg,
    double inflation) {

    std::vector<geometry_msgs::Polygon> result;
    result.reserve(msg.obstacles.size());

    for (const auto& obs : msg.obstacles) {
        result.push_back(inflateObstacle(obs, inflation));
    }

    return result;
}

std::vector<geometry_msgs::Polygon> inflateObstacles(
    const obstacles_msgs::ObstacleArrayMsg& msg) {
    return inflateObstacles(msg, getConfig().totalInflation());
}

geometry_msgs::Polygon shrinkBorders(
    const geometry_msgs::Polygon& borders,
    double shrink_amount) {
    // Shrinking borders = negative offset
    // For a CCW polygon representing the boundary, negative offset shrinks it
    return offsetPolygon(borders, -shrink_amount);
}

geometry_msgs::Polygon shrinkBorders(const geometry_msgs::Polygon& borders) {
    return shrinkBorders(borders, getConfig().totalInflation());
}

}  // namespace planning
