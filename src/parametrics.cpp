#include "parametrics.hpp"
#include "util.hpp"

Coord Coord::operator+(const Coord &other) const { return Coord(x + other.x, y + other.y); }

Coord Coord::operator-(const Coord &other) const { return Coord(x - other.x, y - other.y); }

float Coord::operator*(const Coord &other) const { return this->x * other.x + this->y * other.y; }

Coord Coord::operator*(const float &scalar) const { return Coord(this->x * scalar, this->y * scalar); }

Coord Coord::operator/(const float &scalar) const { return Coord(x / scalar, y / scalar); }

Coord &Coord::operator+=(const Coord &other) {
    this->x += other.x;
    this->y += other.y;
    return *this;
}

Coord Coord::lerp(Coord other, float t) const {
    return Coord(this->x + (other.x - this->x) * t, this->y + (other.y - this->y) * t);
}

float Coord::distance(const Coord &other) const { return std::hypot(this->x - other.x, this->y - other.y); }

float Coord::angle(const Coord &other) const { return std::atan2(other.y - this->y, other.x - this->x); }

Coord Coord::rotate(float angle) const {
    return Coord(this->x * std::cos(angle) - this->y * std::sin(angle),
                 this->x * std::sin(angle) + this->y * std::cos(angle));
}


float getCurvature(Pose pose, Pose other) {
    // calculate whether the pose is on the left or right side of the circle
    float side = sgn(std::sin(pose.theta) * (other.x - pose.x) - std::cos(pose.theta) * (other.y - pose.y));
    // calculate center point and radius
    float a = -std::tan(pose.theta);
    float c = std::tan(pose.theta) * pose.x - pose.y;
    float x = std::fabs(a * other.x + other.y + c) / std::sqrt((a * a) + 1);
    float d = std::hypot(other.x - pose.x, other.y - pose.y);

    // return curvature
    return side * ((2 * x) / (d * d));
}