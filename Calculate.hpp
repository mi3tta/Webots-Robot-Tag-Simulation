#pragma once

#include "Coords.hpp"

class Calculate {
public:
    Calculate();

    double calcDistanceEpucks(double x1, double y1, double x2, 
            double y2) const;
    int calcChaserTargetDist(const Coords& me, const Coords& otherR1, 
            const Coords& otherR2, const Coords& frozRunner) const;
};
