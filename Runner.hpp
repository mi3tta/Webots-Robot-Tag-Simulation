#pragma once

#include <tuple>
#include <utility>
#include <vector>
#include <memory>

class Messaging;
class Calculate;

#include <webots/Motor.hpp>
#include <webots/Robot.hpp>
#include <webots/Receiver.hpp>
#include <webots/Emitter.hpp>
#include <webots/Keyboard.hpp>
#include <webots/LED.hpp>
#include <webots/GPS.hpp>
#include <webots/Compass.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Lidar.hpp>

#include "Points.hpp"
#include "Coords.hpp"

class Runner {
public:
    using pose_t = std::tuple<double, double, double>;      // x, y, h
    using velocity_t = std::tuple<double, double, double>;  // vx, vy, w

    // Robot parameters.
    static constexpr double DIAMETER = 0.071;       //m
    static constexpr double HEIGHT = 0.05;      // m
    static constexpr double WHEEL_RADIUS = 0.02;        // m
    static constexpr double AXLE_LENGTH = 0.052;        // m
    static constexpr double MAX_MOTOR_SPEED = 6.28;     // rad/s
    static constexpr double STOP_SPEED = 0.0;       // 
    static constexpr double ARENA_RADIUS = 0.7;     // m
    static constexpr float BOUNDARY = 0.06;     //m
    static constexpr double MY_RUNNER = 0;
    static constexpr double RUN_1 = 1;
    static constexpr double RUN_2 = 2;
    static constexpr double UNFREEZE_DIST = 0.1;        //m
    static constexpr double DEAD_ZONE = 0.05;       //m

    Runner(webots::Robot& robot);
    ~Runner(); 

    // getters
    void getCoords();
    double getTime() const;

    // altering freeze state
    void toFreeze(double time);
    void unFreeze();

    // emitting messages
    void emitNormCoords();
    void emitFrozenPosition();
    void emitFrozenCoords();
    void emitImTagger();

    // reading messages received
    void readMessages();

    // change freeze state with updated information from messages
    void calcIfUnfreeze();

    // movement and deciding behaviour
    void senseWall();
    void readLidar(float boundary);
    void decideMovement();
    void whosClosest();
    void goTo(double targetX, double targetY);
    void startVector();
    void moveCircular();
    void move(double left, double right);
    void halt();
        
    // getters and setters for runner class, updating internal members
    void clearOtherRunner();
    void setCannotUntagFalse();
    void setCannotUntagTrue();
    void setHasFrozenRunnerFalse();
    void setHasFrozenRunnerTrue();
    void setOtherRunnerCoords(Coords runner);
    void setFrozRunner(Coords Froz);
    void setChaserCoords(Coords chaser);
    void decideSelfUnfreeze(Coords given);
private:
    webots::Robot& robot;
    const double mTimeStep;

    // motors
    webots::Motor* const mLeftMotor;
    webots::Motor* const mRightMotor;
    webots::PositionSensor* const mLeftSensor;
    webots::PositionSensor* const mRightSensor;

    // communication nodes
    webots::Receiver* const mReceiver;
    webots::Emitter* const mEmitter;
        
    // classes
    std::unique_ptr<Messaging> mMessaging;
    std::unique_ptr<Calculate> mCalculate;

    // navigation nodes
    webots::GPS* const mGPS {};
    webots::Compass* const mCompass {};
    webots::Lidar* const mLidar {};
    // for lidar
    std::vector<std::pair<double, double>> mPointCloud {};

    // Coords
    std::vector<double> mStartPoint {};
    Coords mPosition {};
    Coords mChaserCoords {};
    Coords mFrozRunnerCoords {};
    std::vector<Coords> mOtherRunnerCoords {};
    Coords mUntagCoords {};

    // internal flags for runner states
    bool mStraight {true};
    bool mCurve {false};
    bool mHitWall {false};
    bool mFreezeState {false};
    bool mHasFrozenRunner {false};
    bool mHasOtherRunners {false};
    bool mUntag {false};
    bool mCannotUntag{false};
};