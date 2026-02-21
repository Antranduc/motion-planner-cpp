#include "motion_planner/geometry.h"

#include <algorithm>
#include <cmath>

namespace motion_planner {

Circle::Circle(const Eigen::Vector2d& center, double radius)
    : center(center), radius(radius) {
    if (radius <= 0) {
        throw std::invalid_argument("Circle radius must be positive.");
    }
}

ConvexPolygon::ConvexPolygon(std::vector<Eigen::Vector2d> verts) 
    : vertices(std::move(verts)) {
    int num_vertices = vertices.size();

    if (num_vertices < 3) {
        throw std::invalid_argument("Inputed positions do not form a polygon.");
    }
    
    // Checking if vertices are counter-clockwise using signed area.
    double signed_area = 0;
    for (int i = 0; i < num_vertices; i++) {
        int j = (i + 1) % num_vertices;
        signed_area += cross2d(vertices[i], vertices[j]);
    }
    signed_area *= 0.5;

    if (std::abs(signed_area) < kEpsilon) {
        throw std::invalid_argument("Inputted positions are collinear.");
    } else if (signed_area < 0) {
        std::reverse(vertices.begin(), vertices.end());
    }

    // Enforcing convex polygon.
    Eigen::Vector2d prev_edge = vertices[0] - vertices[num_vertices - 1];
    for (int i = 0; i < num_vertices; i++) {
        int j = (i + 1) % num_vertices;
        Eigen::Vector2d curr_edge = vertices[j] - vertices[i];
        if (cross2d(prev_edge, curr_edge) < kEpsilon){
            throw std::invalid_argument("Inputed positions do not form a convex polygon.");
        }
        prev_edge = curr_edge;
    }
}

} // namespace motion_planner
