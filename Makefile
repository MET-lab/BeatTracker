all:
	gcc -o BeatTrackerApp BeatTracker.c BeatTrackerMath.c InitializeBeatVariables.c main.c libportaudio.a -lpthread -lportaudio

