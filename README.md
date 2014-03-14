BeatTracker
===========

##Description
This git project holds David Grunberg's BeatTracker implemented for use with Mark Koh's DARwIn-OP Dance Synthesis project.  The BeatTracker is written in C and utilizes the PortAudio library for streaming the audio in (see below for more info).

##Running the BeatTracker
You may compile the program using make:
```
$ make
``` 
and run it on its own as
```
$ make run
```
This compiles to ./BeatTrackerApp

##Current state (and where to go from here)
###Mark, 3/14/2014
#####Code status
So, right now the BeatTracker is in an alright place.  Since I started on the project, I've made significant improvements to the client-server implementation as well as cleaned up the code a ton.  The program is now actually somewhat reasonable to read.  However, since DARwIn has been unable to track audio due to a broken audio port, I've been hesitant to remove the code that I believe is unused.  I've done as good as I can with it, and maked down anything else that I *think* can be removed, but didn't want to touch without being able to see the effects on the actual performance of the tracker.
#####Line in vs Mic input (the holdup)
As far as further development of the BeatTracker, right now the only way to listen to audio is through the line-in signal in the back of DARwIn.  Really, where we want to be is able to use the mic on the front of DARwIn's face to listen to audio.  I can get this microphone to work with the paex_record example program supplied with PortAudio simply by changing the inputDevice ID from 'PaGetDefaultInputDevice()' to '1', but when I try to do this in the BeatTracker it causes all sorts of issues during runtime.
Where I'd like to move from here is to get that working, and while DARwIn is beatTracking, instead of making his speaker click, have his head light flash to a different color when we recieve a beat (although this should really happen when we recieve a beat in the DanceSynth).

##Notes about PortAudio
Please not that to run this you will need to have PortAudio installed. If you don't have it, go to http://www.portaudio.com/ and get it.
This program uses the WireCallback functionality supplied by PortAudio.  That is, anytime the program recieves data from the line in, the WireCallback function is called, which checks if we're at a beat, and if so, sends a UDP packet to the DanceSynth server.

##Contact
Any questions, please contact Mark Koh(mkoh@drexel.edu) or David(dkg34@drexel.edu).
