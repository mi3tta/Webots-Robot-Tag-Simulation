#include "Calculate.hpp"
#include "Runner.hpp"

#include <iostream>
#include <sstream>
#include <string>
#include <cmath>
#include <map>
#include <random>
#include <limits>
#include <algorithm>
#include <initializer_list>

// Calculate constructor
Calculate::Calculate() = default;

// Calculates the distance between 2 epucks, given both of their 
// x and y coordinates in relation to world
double Calculate::calcDistanceEpucks(double x1, double y1, double x2, 
        double y2) const {
    double dist {};
    dist = sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
    return dist;
}

// Returns integer corresponding to runner with largest distance from frozen
// runner. Called when first "I'm frozen" message is received by runners from
// the frozen runner to decide who should go to untag the runner 
// If runner who called this function is the one with the largest distance,
// will return 0.
int Calculate::calcChaserTargetDist(const Coords& me, const Coords& otherR1, 
            const Coords& otherR2, const Coords& frozRunner) const {
    double meDist = calcDistanceEpucks(me.x, me.y, frozRunner.x,
            frozRunner.y);
    double r1Dist = calcDistanceEpucks(otherR1.x, otherR1.y, frozRunner.x, 
            frozRunner.y);
    double r2Dist = calcDistanceEpucks(otherR2.x, otherR2.y, frozRunner.x, 
            frozRunner.y);

    if (frozRunner.x == me.x && frozRunner.y == me.y) {
        meDist = -std::numeric_limits<double>::infinity();
    } else if (frozRunner.x == otherR1.x && frozRunner.y == otherR1.y) {
        r1Dist = -std::numeric_limits<double>::infinity();
    } else if (frozRunner.x == otherR2.x && frozRunner.y == otherR2.y) {
        r2Dist = -std::numeric_limits<double>::infinity();
    }

    double maxDist = std::max({meDist, r1Dist, r2Dist});
    if (maxDist == meDist) {
        return Runner::MY_RUNNER;
    } else if (maxDist == r1Dist) {
        return Runner::RUN_1;
    } else {
        return Runner::RUN_2;
    }
}
