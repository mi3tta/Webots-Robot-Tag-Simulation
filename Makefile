# Edit CXX_SOURCES as required.

CXX_SOURCES = runner_controller.cpp run.cpp Runner.cpp Points.cpp Coords.cpp Messaging.cpp Calculate.cpp
CFLAGS = -std=c++17 -D_USE_MATH_DEFINES

### Do not modify: this includes Webots global Makefile.include
null :=
space := $(null) $(null)
WEBOTS_HOME_PATH?=$(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))
include $(WEBOTS_HOME_PATH)/resources/Makefile.include