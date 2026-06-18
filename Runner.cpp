#include "Runner.hpp"
#include "Messaging.hpp"
#include "Calculate.hpp"

#include <iostream>
#include <sstream>
#include <string>
#include <cmath>
#include <map>
#include <random>
#include <limits>
#include <algorithm>

// Runner constructor
Runner::Runner(webots::Robot& robot) : robot(robot),  
        mTimeStep{robot.getBasicTimeStep()},
        mLeftMotor{robot.getMotor("left wheel motor")}, 
        mRightMotor{robot.getMotor("right wheel motor")},
        mLeftSensor{robot.getPositionSensor("left wheel sensor")},
        mRightSensor{robot.getPositionSensor("right wheel sensor")},
        mReceiver{robot.getReceiver("receiver")},
        mEmitter{robot.getEmitter("emitter")},
        mMessaging(nullptr),
        mCalculate(nullptr),
        mGPS{robot.getGPS("gps")},
        mCompass{robot.getCompass("compass")},
        mLidar{robot.getLidar("lidar")}
        {
        mReceiver->setChannel(-1);
        mReceiver->enable(mTimeStep);

        mLeftSensor->enable(mTimeStep);
        mRightSensor->enable(mTimeStep);

        mEmitter->setChannel(-1);
        mEmitter->setRange(INFINITY);

        mMessaging = std::make_unique<Messaging>(this, mEmitter, mReceiver);
        mCalculate = std::make_unique<Calculate>();

        mGPS->enable(mTimeStep);
        mCompass->enable(mTimeStep);

        mLidar->enable(mTimeStep);
        mLidar->enablePointCloud();

        auto led8 = robot.getLED("led8");
        led8->set(1);
    }

//Default destructor
Runner::~Runner() = default;

// Updates Runner's Coord class with with it's x and y coordinates
// at time of call (found by GPS). Updates it's header 
// at time of call (found by compass).
void Runner::getCoords() {
    const double* gpsValues {mGPS->getValues()};
    mPosition.x = gpsValues[0];
    mPosition.y = gpsValues[1];

    const double* compassValues {mCompass->getValues()};
    mPosition.h = std::atan2(compassValues[0], compassValues[1]);
}

//Get time of simulation
double Runner::getTime() const {
    return robot.getTime();
}

// Changes freeze state of runner so runner is frozen, turns off led, calls halt
void Runner::toFreeze(double time) {
    if (!mFreezeState) {
        mFreezeState = true;
        std::cout << mFreezeState << std::endl;
        auto led8 = robot.getLED("led8");
        led8->set(0);
        halt();
    }
}

// Changes freeze state of runner to unfrozen, turns on led
void Runner::unFreeze() {
    if (mFreezeState) {
        mFreezeState = false;
        auto led8 = robot.getLED("led8");
        led8->set(1);
    }
}

// Runner calls emitNormCoords in Messaging class
void Runner::emitNormCoords() {
    double time = getTime();
    getCoords();

    mMessaging->emitNormCoords(time, mPosition);
}

// Behaviour check - if Runner is frozen, will call emitFrozenCoords()
void Runner::emitFrozenPosition() {
    if (mFreezeState) {
        emitFrozenCoords(); 
    }
}

// Runner calls emitFrozenCoords in Messaging class because it is frozen
void Runner::emitFrozenCoords() {
    double time = getTime();
    getCoords();

    mMessaging->emitFrozenCoords(time, mPosition);
}

// If Runner is going to untag the frozen runner, calls emitImTagger in
// Messaging class
void Runner::emitImTagger() {
    if (mUntag) {
        double time = getTime();
        mMessaging->emitImTagger(time);
    }
}

// Calls readMessages in Messaging class to read messages received by this 
// runner during timestep
void Runner::readMessages() {
    mMessaging->readMessages();
}

// Decides if this Runner is close enough to the frozen runner to send
// it an unfreeze message
// Uses calcDistanceEpucks in Calculate class to work out if this runner
// is within the unfreezing distance (<0.1m), and if it is sends message
void Runner::calcIfUnfreeze() {
    //no frozen runner
    if (!mHasFrozenRunner) {
        return;
    }
    //my own runner is frozen
    if (mFreezeState) {
        return;
    }

    double dist {};
    getCoords();
    dist = mCalculate->calcDistanceEpucks(mFrozRunnerCoords.x, 
            mFrozRunnerCoords.y, mPosition.x, mPosition.y);

    if (dist < UNFREEZE_DIST) {
        mEmitter->setChannel(2);
        double time = getTime();
        mMessaging->emitUnfreezeMessage(time, mFrozRunnerCoords);
        mUntag = false;
    } 
}

// Calls readLidar to determine if close enough to the wall to change mCurve
// to true and mStraight to false so robot will decide in decideMovement
// to start moving circularly along the wall.
void Runner::senseWall() {
    readLidar(BOUNDARY);
    if (mHitWall) {
        mCurve = true;
        mStraight = false;
    }
}

// Reads the runner's lidar. If the runner is within 0.06m of the wall
// as sensed by lidar, will change mHitWall state to true. Let's runner
// know it should start to move circularly along the perimater.
void Runner::readLidar(float boundary) {
    const float* ranges{mLidar->getRangeImage()};
    int numPoints{mLidar->getNumberOfPoints()};

    // Loop through all the ranges.
    for (int i{0}; i < numPoints; i++) {
        float range{ranges[i]};

        if (range <= boundary) {
            mHitWall = true;
        }
    }
}

// Decides movement of the runner based on runner's internal state
void Runner::decideMovement() {   
    // Runner has been set to tag the frozen runner. Must go straight
    // to frozen runner coordinates
    if (mUntag && !mCannotUntag) {
        getCoords();
        goTo(mUntagCoords.x, mUntagCoords.y);
    } else if (mStraight && mStartPoint.empty()) {
        // Simulation has just started. Runner must move to perimeter
        startVector();
    } else if (mStraight) {
        // Runner has already calculated perimeter point to go to
        // Just go to this point
        goTo(mStartPoint[0], mStartPoint[1]);
    } else if (mHitWall) {
        // Runner is close enough to wall to start moving around perimeter
        moveCircular();
    }
}

// Sets tag state of runner who should untag the frozen runner, and sets
// the coordinates of where it should go to untag the frozen runner.
// Calls calcChaserTargetDist helper in Calculate class 
// to identify runner furthest from frozen
// runner.
// If runner who calls this function has already been tagged, returns.
// If another runner has already been tagged, returns
// If runner is frozen, there is no frozen runner, or the coordinates of other
// runners have not been emitted yet (E.g. at the beginning of gameplay), 
// returns.
void Runner::whosClosest() {
    if (mUntag == true && mCannotUntag == false) {
        return;
    }
    if (mUntag == false && mCannotUntag == true) {
        return;
    }
    if (mFreezeState) {
        return;
    }
    if (!mHasFrozenRunner) {
        return;
    }
    if (mOtherRunnerCoords.size() < 1) {
        return;
    }

    getCoords();
    Coords& oRunner1 = mOtherRunnerCoords[0];
    Coords& oRunner2 = mOtherRunnerCoords[1];
    int runner = mCalculate->calcChaserTargetDist(mPosition, oRunner1, 
            oRunner2, mFrozRunnerCoords);

    // If runner who called this function should be tagger, below statement
    // will be true, 0 == 0
    if (runner == MY_RUNNER) {
        mUntag = true;
        mUntagCoords.x = mFrozRunnerCoords.x;
        mUntagCoords.y = mFrozRunnerCoords.y;
        mUntagCoords.h = mFrozRunnerCoords.h;
    }
}

// Proportional controller that gets runner to move to specified coordinates
// given
void Runner::goTo(double targetX, double targetY) {
    if (mFreezeState) {
        return;
    }

    getCoords();
    double xr = mPosition.x;
    double yr = mPosition.y;
    double hr = mPosition.h;
    double xt = targetX;
    double yt = targetY;
    // calculate difference in coords (x, y, header)
    double dx = xt - xr;
    double dy = yt - yr;
    double dh = std::atan2(dy, dx) - hr;
    double dd = std::hypot(dx, dy);

    // normalise
    while (dh > M_PI) {
        dh -= 2 * M_PI;
    }
    while (dh < -M_PI) {
        dh += 2 * M_PI;
    }

    // proportional controller
    double kd = 1;
    double kh = 1 * kd;
    double v = kd * dd;
    double w = kh * dh;

    // Left and right wheel velocities
    double wl = v - (AXLE_LENGTH / 2.0) * w / WHEEL_RADIUS;
    double wr = v + (AXLE_LENGTH / 2.0) * w / WHEEL_RADIUS;
    
    // Max out speeds.
    double throttle = MAX_MOTOR_SPEED / std::max(std::abs(wl), std::abs(wr));
    wl *= throttle;
    wr *= throttle;

    wl = std::clamp(wl, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);
    wr = std::clamp(wr, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);

    // Deadzone, dont move if at pos
    if (std::abs(dd) > DEAD_ZONE) {
        move(wl, wr);
    }
    else {
        halt();
    }
}

// Start vector. Makes runner move towards the perimeter of the arena
// along a vector perpendicular to vector between this runner and the chaser
// at the very start of the simulation (so between runner and world center)
void Runner::startVector() {
    if (!mStraight) {
        return;
    }
    if (mFreezeState) {
        return;
    }

    getCoords();

    double xDiff = mPosition.x - 0.0;
    double yDiff = mPosition.y - 0.0;

    double angleFromCentre = std::atan2(yDiff, xDiff);
    double angleOfTangCW = angleFromCentre - M_PI/4;

    while (angleOfTangCW < -M_PI) {
        angleOfTangCW += 2 * M_PI;
    } 
    while (angleOfTangCW > M_PI) {
        angleOfTangCW -= 2 * M_PI;
    }

    double xPerimCoords = ARENA_RADIUS * std::cos(angleOfTangCW);
    double yPerimCoords = ARENA_RADIUS * std::sin(angleOfTangCW);
    mStartPoint.push_back(xPerimCoords);
    mStartPoint.push_back(yPerimCoords);
}


// Gets runner to move in a circular fashion along the edge of the perimeter
void Runner::moveCircular() {
    if (!mCurve || mFreezeState) {
        return;
    }

    getCoords();
    double x = mPosition.x;
    double y = mPosition.y;
    double rad = std::sqrt(x*x + y*y);
    double theta = std::atan2(y, x);

    // tangent in CW direction
    double targetHeading = theta - M_PI/2;
    while (targetHeading < -M_PI) targetHeading += 2*M_PI;
    while (targetHeading >  M_PI) targetHeading -= 2*M_PI;
    double err = targetHeading - mPosition.h;
    while (err < -M_PI) {
        err += 2*M_PI;
    }
    while (err >  M_PI) {
        err -= 2*M_PI;
    }

    // control gains
    double KpTurn = 1.5; 
    double KpRad  = 0.7;

    double turn = KpTurn * err + KpRad * (ARENA_RADIUS - rad);
    turn = std::clamp(turn, -1.0, 1.0);
    double forward = 1.0;
    double left  = forward - turn;
    double right = forward + turn;

    // will normalise if any wheel is above |1|
    double maxMag = std::max(std::abs(left), std::abs(right));
    if (maxMag > 1.0) {
        left  /= maxMag;
        right /= maxMag;
    }
    // scales to max speed
    left  *= MAX_MOTOR_SPEED;
    right *= MAX_MOTOR_SPEED;

    move(left, right);
}

// Sets Runner's left and right motors to be velocities for left and right
// passed in respectively
void Runner::move(double left, double right) {
    if (!mFreezeState) {
        mLeftMotor->setPosition(INFINITY);
        mRightMotor->setPosition(INFINITY);
        mLeftMotor->setVelocity(left);
        mRightMotor->setVelocity(right);
    }
}

// Stops Runner movement
void Runner::halt() {
    mLeftMotor->setVelocity(STOP_SPEED);
    mRightMotor->setVelocity(STOP_SPEED);
}

// Clears Runner's internal mOtherRunnerCoords vector
void Runner::clearOtherRunner() {
    mOtherRunnerCoords.clear();
}

// Sets Runner internal member mCannotUntag to false
void Runner::setCannotUntagFalse() {
    mCannotUntag = false;
}

// Sets Runner internal member mCannotUntag to true
void Runner::setCannotUntagTrue() {
    mCannotUntag = true;
}

// Sets Runner internal member mHasFrozenRunner to false
void Runner::setHasFrozenRunnerFalse() {
    mHasFrozenRunner = false;
}

// Sets Runner's mHasFrozenRunner state to true
void Runner::setHasFrozenRunnerTrue() {
    mHasFrozenRunner = true;
}

// Adds a Coords class to mOtherRunnerCoords which contains another runner's 
// coordinates that have been emitted
void Runner::setOtherRunnerCoords(Coords runner) {
    mOtherRunnerCoords.push_back(runner);
}

// Updates runner's knowledge of the frozen runner's coordinates.
void Runner::setFrozRunner(Coords Froz) {
    mFrozRunnerCoords.x = Froz.x;
    mFrozRunnerCoords.y = Froz.y;
    mFrozRunnerCoords.h = Froz.h;
}

// Sets mChaserCoords member with Coords passed in
void Runner::setChaserCoords(Coords chaser) {
    mChaserCoords.x = chaser.x;
    mChaserCoords.y = chaser.y;
    mChaserCoords.h = chaser.h;
}

// Decides based on given coordinates whether runner should unfreeze itself
void Runner::decideSelfUnfreeze(Coords given) {
    getCoords();
    if (mCalculate->calcDistanceEpucks(given.x, given.y, mPosition.x,
            mPosition.y) < 0.1) {
        unFreeze();
    }  
}