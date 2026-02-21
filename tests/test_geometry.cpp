#include "motion_planner/geometry.h"

#include <gtest/gtest.h>

TEST(LineSegment, Construction) {
    motion_planner::LineSegment line = motion_planner::LineSegment{{0, 3}, {-1, -5}};
    EXPECT_TRUE(line.start.isApprox(Eigen::Vector2d(0, 3)));
    EXPECT_TRUE(line.end.isApprox(Eigen::Vector2d(-1, -5)));
}

TEST(Circle, Construction) {
    motion_planner::Circle circle = motion_planner::Circle({0, 0}, 5);
    EXPECT_TRUE(circle.center.isApprox(Eigen::Vector2d(0, 0)));
    EXPECT_EQ(circle.radius, 5);
}

TEST(Circle, InvalidRadius) {
    EXPECT_THROW(motion_planner::Circle({0, 0}, -4), std::invalid_argument);
}

TEST(ConvexPolygon, Construction) {
    motion_planner::ConvexPolygon poly = motion_planner::ConvexPolygon(
            std::vector<Eigen::Vector2d>{{5, 5}, {0, 5}, {0, 0}, {5, 0}});
    EXPECT_TRUE(poly.vertices[0].isApprox(Eigen::Vector2d(5, 5)));
    EXPECT_TRUE(poly.vertices[1].isApprox(Eigen::Vector2d(0, 5)));
    EXPECT_TRUE(poly.vertices[2].isApprox(Eigen::Vector2d(0, 0)));
    EXPECT_TRUE(poly.vertices[3].isApprox(Eigen::Vector2d(5, 0)));
}

TEST(ConvexPolygon, ClockWiseConstruction) {
    motion_planner::ConvexPolygon poly = motion_planner::ConvexPolygon(
            std::vector<Eigen::Vector2d>{{5, 5}, {5, 0}, {0, 0}, {0, 5}});
    EXPECT_TRUE(poly.vertices[0].isApprox(Eigen::Vector2d(0, 5)));
    EXPECT_TRUE(poly.vertices[1].isApprox(Eigen::Vector2d(0, 0)));
    EXPECT_TRUE(poly.vertices[2].isApprox(Eigen::Vector2d(5, 0)));
    EXPECT_TRUE(poly.vertices[3].isApprox(Eigen::Vector2d(5, 5)));
}

TEST(ConvexPolygon, CollinearConstruction) {
    EXPECT_THROW(motion_planner::ConvexPolygon(std::vector<Eigen::Vector2d>{{0,0}, {5,5}, {10, 10}}), 
                std::invalid_argument);
}

TEST(ConvexPolygon, ConcaveConstruction) {
    EXPECT_THROW(motion_planner::ConvexPolygon(std::vector<Eigen::Vector2d>{{0, 0}, {10, 0}, {1, 1}, {0, 10}}),
                std::invalid_argument);
}

TEST(ConvexPolygon, NotPolygonConstruction) {
    EXPECT_THROW(motion_planner::ConvexPolygon(std::vector<Eigen::Vector2d>{{0, 0}}), std::invalid_argument);
}
