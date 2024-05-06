#include "main.h"
#pragma once

class Coord {
    public:
        float x, y;

        Coord(float x, float y)
            : x(x),
              y(y) {}

        Coord operator+(const Coord& other) const;
        Coord operator-(const Coord& other) const;
        float operator*(const Coord& other) const;
        Coord operator*(const float& scalar) const;
        Coord operator/(const float& scalar) const;
        Coord &operator+=(const Coord &other);
        Coord lerp(Coord other, float t) const;
        float distance(const Coord& other) const;
        float angle(const Coord& other) const;
        Coord rotate(float angle) const;
};

class Pose : public Coord {
    public:
    float theta;

    Pose(float x, float y, float theta) : Coord(x, y), theta(theta) {}
    
    // Constructor to convert pxl::Coord to Pose
    Pose(const Coord &other) : Coord(other), theta() {}
   
    float getCurvature(Pose& pose, Pose& other);
};