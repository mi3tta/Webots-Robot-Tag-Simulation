#pragma once

#include <string>
#include <sstream>

#include <webots/Emitter.hpp>
#include <webots/Receiver.hpp>

#include "Coords.hpp"
#include "Runner.hpp"


class Runner;

class Messaging {
public:
    Messaging(Runner* owner, webots::Emitter* emitter, 
            webots::Receiver* receiver);

    void readMessages();

    void emitNormCoords(double time, const Coords& pos);
    void emitFrozenCoords(double time, const Coords& pos);
    void emitImTagger(double time);
    void emitUnfreezeMessage(double time, const Coords& frozenRunner);

private:
    void gotNormMessage(std::stringstream& sStream, std::string& output);
    void gotRunnerFrozenMessage(std::stringstream& sStream, std::string& output);
    void gotFreezeMessage(double time);
    void gotWarningMessage(std::stringstream& sStream, std::string& output);
    void readUnfreezeMessage(std::stringstream& sStream, std::string& output);
    Runner* const mOwner;
    webots::Emitter* const mEmitter;
    webots::Receiver* const mReceiver;
};

    