#pragma once

#include <vector>
#include <Eigen/Dense>

namespace motion_planner {
    /// Tolerance for floating-point comparisons.
    inline constexpr double kEpsilon = 1e-10;

    /// A line segment defined by two endpoints.
    struct LineSegment {
        Eigen::Vector2d start;
        Eigen::Vector2d end;
    };

    /// A circle defined by a center point and radius.
    struct Circle {
        Eigen::Vector2d center;
        double radius;

        /// @throws std::invalid_argument if radius <= 0.
        Circle(const Eigen::Vector2d& center, double radius);
    };

    /// A convex polygon defined by an ordered list of vertices.
    /// Vertices are stored in counter-clockwise order.
    struct ConvexPolygon {
        std::vector<Eigen::Vector2d> vertices;

        /// Enforces CCW winding and validates convexity.
        /// @throws std::invalid_argument if fewer than 3 vertices, collinear, or non-convex.
        ConvexPolygon(std::vector<Eigen::Vector2d> vertices);
    };

    /// Returns the 2D cross product (z-component) of two vectors.
    inline double cross2d(const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
        return a.x() * b.y() - b.x() * a.y();
    }
} // namespace motion_planner
