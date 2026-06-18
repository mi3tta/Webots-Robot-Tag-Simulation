#include "run.hpp"
#include "Runner.hpp"

#include <iostream>
#include <sstream>

void run(webots::Robot& robot) {
    Runner runner(robot);
    double timeStep{robot.getBasicTimeStep()};
    while (robot.step(timeStep) != -1) {
        runner.emitNormCoords();
        runner.emitImTagger();
        runner.readMessages();
        runner.emitFrozenPosition();
        runner.calcIfUnfreeze();
        runner.senseWall();
        runner.decideMovement();
    }
}
