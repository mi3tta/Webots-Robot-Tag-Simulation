#include "Messaging.hpp"
#include "Runner.hpp" 
#include <iostream>

//Messaging constructor
Messaging::Messaging(Runner* owner, webots::Emitter* emitter,
        webots::Receiver* receiver) : 
        mOwner(owner), mEmitter(emitter), mReceiver(receiver) {}

// Reads all messages received by runner during one timestep.
// If the message recieved in type
// "normal", calls gotNormMessage
// "frozen", calls gotRunnerFrozenMessage
// "freeze", calls gotFreezeMessage
// "warning", calls gotWarningMessage
// "unfreeze", readUnfreezeMessage
// "tagging", calls setCannotUntagTrue
// If no tagging message or frozen message was called, this means no 
// runners are frozen or going to untag a frozen runner in this timestep
// This information is sent to the chaser for it to use to update any 
// behaviour it needs based off this information. Sent to chaser by calling
// setCannotUntagTrue, setCannotUntagFalse, setHasFrozenRunnerFalse
void Messaging::readMessages() {
    mOwner->clearOtherRunner();
    bool gotTagMessage {false};
    bool gotFrozenMessage {false};

    while (mReceiver->getQueueLength() > 0) {
        std::string message{
            static_cast<const char*>(mReceiver->getData()),
            static_cast<size_t>(mReceiver->getDataSize())
        };
        //std::cout << message << std::endl;
        
        std::stringstream sStream(message);
        std::string output;
        double time;
        std::string messageType;

        std::getline(sStream, output, ':');
        time = std::stod(output);
        std::getline(sStream, messageType, ':');

        if (messageType == "normal") {
            gotNormMessage(sStream, output);
        } else if (messageType == "frozen") {
            gotFrozenMessage = true;
            gotRunnerFrozenMessage(sStream, output);
        } else if (messageType == "freeze") {
            gotFreezeMessage(time);
        } else if (messageType == "warning") {
            gotWarningMessage(sStream, output);
        } else if (messageType == "unfreeze") {
            readUnfreezeMessage(sStream, output);
        } else if (messageType == "tagging") {
            gotTagMessage = true;
            mOwner->setCannotUntagTrue();
        }
        mReceiver->nextPacket();
    }

    if (!gotTagMessage) {
        mOwner->setCannotUntagFalse();
    }
    if (!gotFrozenMessage) {
        mOwner->setHasFrozenRunnerFalse();
    }
}

// Emits Runner's normal coordinates (x, y, heading) on channel 2 
// by passing to Runner's emitter
void Messaging::emitNormCoords(double time, const Coords& pos) {
    mEmitter->setChannel(2);

    std::stringstream ss {};
    ss << time << ":normal:" << pos.x << "," << pos.y << "," << pos.h;
    
    const std::string message {ss.str()};
    mEmitter->send(message.c_str(), message.size());
}

// Emits Runner's frozen coordinates (x, y, heading) on channel 2 
// by passing to Runner's emitter
void Messaging::emitFrozenCoords(double time, const Coords& frozRunnerPos) {
    mEmitter->setChannel(2);

    std::stringstream ss {};
    ss << time << ":frozen:" << frozRunnerPos.x << "," << frozRunnerPos.y << "," 
        << frozRunnerPos.h;

    const std::string message {ss.str()};
    mEmitter->send(message.c_str(), message.size());
}

// If Runner is going to untag the frozen runner, emits that it is the tagger
// this ensures the other runner will no longer try and calculate if it should
// be the tagger (because as tagger moves closer, will be a point where
// it is not furthest away from frozen runner, causing other runner to think
// it should be the new tagger when it should not be)
void Messaging::emitImTagger(double time) {
    mEmitter->setChannel(2);

    std::stringstream ss {};
    ss << time << ":tagging";

    const std::string message {ss.str()};
    mEmitter->send(message.c_str(), message.size());
}

// Emits an unfreeze message with frozen runner's coordinates.
void Messaging::emitUnfreezeMessage(double time, const Coords& frozenRunner) {
    mEmitter->setChannel(2);
    
    std::stringstream ss {};
    ss << time << ":unfreeze:" << frozenRunner.x << "," << frozenRunner.y 
        << "," << frozenRunner.h;

    const std::string message {ss.str()};
    mEmitter->send(message.c_str(), message.size());
}

// Helper function, extracts runner's coordinates during timestep and saves
// in a Coords class, and pushes into mOtherRunnersCoords to store for 
// Runner
void Messaging::gotNormMessage(std::stringstream& sStream, 
        std::string& output) {
    double x;
    double y;
    double h;
    std::getline(sStream, output, ',');
    x = std::stod(output);

    std::getline(sStream, output, ',');
    y = std::stod(output);

    std::getline(sStream, output, ',');
    h = std::stod(output);

    Coords runner{x, y, h};
    mOwner->setOtherRunnerCoords(runner);
}

// Helper function, extracts coordinates of frozen runner emitted by frozen
// runner, and updates these coordinates appropriately in runner class
// Gets runner to setHasFrozenRunnerTrue(), then gets runner to call 
// whosClosest() and calcIfUnfreeze() to decide it's behaviours accordingly
// based on information extracted in this function.
void Messaging::gotRunnerFrozenMessage(std::stringstream& sStream, 
        std::string& output) {
    double x;
    double y;
    double h;
    std::getline(sStream, output, ',');
    x = std::stod(output);

    std::getline(sStream, output, ',');
    y = std::stod(output);

    std::getline(sStream, output, ',');
    h = std::stod(output);

    Coords froz{x, y, h};
    mOwner->setFrozRunner(froz);
    mOwner->setHasFrozenRunnerTrue();
    mOwner->whosClosest();
    mOwner->calcIfUnfreeze();
}

// Helper gets Runner to call toFreeze to update its own state if
// received freeze message
void Messaging::gotFreezeMessage(double time) {
    mOwner->toFreeze(time);
}

// Helper function, extracts chaser's coordinates during timestep and saves
// in a Coords class, and calls setChaserCoords passing in the Coords
// for Runner to update its record of chaser's current coordinates
void Messaging::gotWarningMessage(std::stringstream& sStream, 
        std::string& output) {
    double x {};
    double y {};
    double h {};
    std::getline(sStream, output, ',');
    x = std::stod(output);

    std::getline(sStream, output, ',');
    y = std::stod(output);

    std::getline(sStream, output, ',');
    h = std::stod(output);

    Coords chaser{x, y, h};
    mOwner->setChaserCoords(chaser);
}

// Helper function, extracts unfreeze coordinates (e.g. if another runner
// has decided they are near enough to the frozen runner to unfreeze it)
// passed in from another runner.
// Passes these to the Runner by calling decideSelfUnfreeze function,
// to let it decide further behaviour with this information
void Messaging::readUnfreezeMessage(std::stringstream& sStream, 
        std::string& output) {
    double x;
    double y;
    double h;
    std::getline(sStream, output, ',');
    x = std::stod(output);

    std::getline(sStream, output, ',');
    y = std::stod(output);

    std::getline(sStream, output, ',');
    h = std::stod(output);

    Coords read{x, y, h};
    mOwner->decideSelfUnfreeze(read);
}
