#include "Points.hpp"

#include <cmath>
#include <map>
#include <random>
#include <string>
#include <utility>
#include <algorithm>
#include <vector>

// Default constructor
Points::Points() : mX(0.0), mY(0.0), mHead(0.0) {}

// Constructor x, y, h
Points::Points(double x, double y, double head) : mX{x}, mY{y}, mHead{head} {}

// Constructor x, y
Points::Points(double x, double y) : mX{x}, mY{y}, mHead{0.0} {}

// Returns x
double Points::getX() const {
    return mX;
}

// Returns y
double Points::getY() const {
    return mY;
}

// Returns h
double Points::getH() const {
    return mHead;
}

// Sets x
void Points::setX(double x) {
    mX = x;
}

// Sets y
void Points::setY(double y) {
    mY = y;
}

// Sets h
void Points::setH(double head) {
    mHead = head;
}

// Returns direction between two vectors
double Points::directionBn(const Points &second) const {
    double secondX = second.getX();
    double secondY = second.getY();

    double mag1 = std::hypot(mX, mY);
    double mag2 = std::hypot(secondX, secondY);
    if (mag1 == 0.0 || mag2 == 0.0)  {
        return VECTOR_ZERO;
    }

    double cosTheta = (mX * secondX + mY * secondY) / 
        (std::hypot(mX, mY) * std::hypot(secondX, secondY));
    cosTheta = std::clamp(cosTheta, -1.0, 1.0);
    return std::acos(cosTheta);
}

// Returns angle above x axis of a vector
double Points::angleAboveX() const {
    return atan2(mY, mX);
}
