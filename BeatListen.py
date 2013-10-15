#!/usr/bin/env python
#Ros Imports
import roslib; roslib.load_manifest('maestro')
import rospy
from hubomsg.msg import PythonMessage
#UDP socket client
import socket 


if __name__ == '__main__':
    #ros setup
    print "Initializing Node"
    rospy.init_node("BeatListener")
    pub = rospy.Publisher('Maestro/Control', PythonMessage)
    #handles udp setup
    print "Initializing USP"
    UDP_IP = "127.0.0.1"
    UDP_PORT = 9930 
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) 
    sock.bind((UDP_IP, UDP_PORT)) 
    #Infinite loop that tries to receive data from UDP
    try:
        while True:
            data, addr = sock.recvfrom(70) #Does not continue until it receives a message, hopefully every beat
            print "received message:", data
            msg = PythonMessage("RSP", "position", "-.2", "")
            pub.publish(msg)
    except Exception, e:
        raise e
