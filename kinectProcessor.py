#!/usr/bin/env python
from freenect import sync_get_depth as get_depth, sync_get_video as get_video
import struct
import freenect
import Create2_TetheredDrive as robotLib
import numpy as np
import cv
import sys, glob # for listing serial ports
from Tkinter import *
import tkMessageBox
import tkSimpleDialog
try:
    import serial
except ImportError:
    # tkMessageBox.showerror('Import error', 'Please install pyserial.')
    print("'Import error: Please install pyserial.")
    raise
connection = None
VELOCITYCHANGE = 200
ROTATIONCHANGE = 300
distanceThreshold = 100
whiteThreshold = 255

class TetheredApp(Tk):


    def __init__(self):
        Tk.__init__(self)

# sendCommandASCII takes a string of whitespace-separated, ASCII-encoded base 10 values to send
    def sendCommandASCII(self, command):
            cmd = ""
            for v in command.split():
                cmd += chr(int(v))

            self.sendCommandRaw(cmd)

    def callMovementCommand(self,rVel,lVel):
        cmd = struct.pack(">Bhh", 145, rVel, lVel) #command for robot full stop
        self.sendCommandRaw(cmd)
        self.callbackKeyLastDriveCommand = cmd

#TODO add a method that automatically adjusts tilt of kinect
    def doloop(self):
        global depth, rgb
        while True:
            (depth, _), (rgb, _) = get_depth(), get_video()

            d3 = np.dstack((depth, depth, depth)).astype(np.uint8)

            redAvg = 0
            greenAvg = 0
            blueAvg = 0
            distanceAvg = 0

            for i in range(len(d3)):
                redAvg += d3[0][i][0]
                greenAvg += d3[0][i][1]
                blueAvg += d3[0][i][2]

            redAvg = redAvg / len(d3)
            greenAvg = greenAvg / len(d3)
            blueAvg = blueAvg / len(d3)
            distanceAvg = (redAvg + blueAvg + greenAvg) / 3

            cv.ShowImage('both', cv.fromarray(np.array(d3[::2, ::2, ::-1])))
            self.sendCommandASCII('128') # sets mode to passive
            self.sendCommandASCII('131') # sets mode to safe

            if(distanceAvg >= 255 or distanceAvg <= 10):
                self.sendCommandASCII('140 3 1 64 16 141 3') #beep
                self.callMovementCommand(0,-100)

            elif (distanceAvg > distanceThreshold):
                self.sendCommandASCII('140 3 1 64 16 141 3') #beep
                self.callMovementCommand(100,100)

            else:
                self.callMovementCommand(0,0) #stop
                self.callMovementCommand(0,100) #Right turn



            cv.WaitKey(10)

    def getSerialPorts(self):
        """Lists serial ports
        From http://stackoverflow.com/questions/12090503/listing-available-com-ports-with-python

        :raises EnvironmentError:
            On unsupported or unknown platforms
        :returns:
            A list of available serial ports
        """
        if sys.platform.startswith('win'):
            ports = ['COM' + str(i + 1) for i in range(256)]

        elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
            # this is to exclude your current terminal "/dev/tty"
            ports = glob.glob('/dev/tty[A-Za-z]*')

        elif sys.platform.startswith('darwin'):
            ports = glob.glob('/dev/tty.*')

        else:
            raise EnvironmentError('Unsupported platform')

        result = []
        for port in ports:
            try:
                s = serial.Serial(port)
                s.close()
                result.append(port)
            except (OSError, serial.SerialException):
                pass
        return result

    def connect(self):
        global connection

        if connection is not None:
            print "Already connected"
            return

        try:
            ports = self.getSerialPorts()
            print(ports)
            port = ports[0]
        except EnvironmentError:
            print "ENVIRONMENT ERROR!!"

        if port is not None:
            print "Trying " + str(port) + "... "
            try:
                connection = serial.Serial(port, baudrate=115200, timeout=1)
                print "Connected!"

            except:
                print "Failed."


 # sendCommandRaw takes a string interpreted as a byte array
    def sendCommandRaw(self, command):
        global connection

        try:
            if connection is not None:
                 connection.write(command)
            # else:
            #     # tkMessageBox.showerror('Not connected!', 'Not connected to a robot!')
            #     print ""
        except serial.SerialException:
            print "Lost connection"
                # tkMessageBox.showinfo('Uh-oh', "Lost connection to the robot!")
            connection = None

            print ' '.join([ str(ord(c)) for c in command ])
            self.text.insert(END, ' '.join([ str(ord(c)) for c in command ]))
            self.text.insert(END, '\n')
            self.text.see(END)

np.set_printoptions(threshold='nan')
distanceThreshold = 100
if __name__ == "__main__":
    robot = TetheredApp()
    robot.connect()
    robot.doloop()









