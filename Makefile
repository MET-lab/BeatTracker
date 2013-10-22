all:
	gcc -o BeatTrackerApp BeatTracker.c BeatTrackerMath.c InitializeBeatVariables.c main.c portaudio.h  -lpthread -lportaudio -lm 

