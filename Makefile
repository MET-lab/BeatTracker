###############################################################
#
# Purpose: Makefile for "DanceSynth"
# Author.: Mark Koh
# Version: 0.1
# Date: 1/12/2014
#
###############################################################

TARGET=BeatTrackerApp

LFLAGS += libportaudio.a -lpthread -lportaudio -lm
CXX = gcc
CXXFLAGS += -g

OBJECTS = BeatTracker.c BeatTrackerMath.c InitializeBeatVariables.c main.c

all: $(TARGET)
	
$(TARGET): $(OBJECTS)
	$(CXX) -o $(TARGET) $(CXXFLAGS) $(OBJECTS) $(LFLAGS)
	chmod 755 $(TARGET)

run: $(TARGET)
	./$(TARGET)

clean:
	rm -rf $(TARGET) *.o
