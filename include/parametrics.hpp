#include "main.h"
#pragma once

class Coord {
    public:
        double x, y;

        Coord(double x, double y)
            : x(x),
              y(y) {}

        Coord operator+(const Coord& other) const;
        Coord operator-(const Coord& other) const;
        double operator*(const Coord& other) const;
        Coord operator*(const float& scalar) const;
        Coord operator/(const float& scalar) const;
        Coord &operator+=(const Coord &other);
        Coord lerp(Coord other, float t) const;
        double distance(const Coord& other) const;
        double angle(const Coord& other) const;
        Coord rotate(double angle) const;
};

class Pose : public Coord {
    public:
    double theta;

    Pose(double x, double y, double theta) : Coord(x, y), theta(theta) {}
    
    // Constructor to convert pxl::Coord to Pose
    Pose(const Coord &other) : Coord(other), theta() {}
   
    double getCurvature(Pose& pose, Pose& other);
};