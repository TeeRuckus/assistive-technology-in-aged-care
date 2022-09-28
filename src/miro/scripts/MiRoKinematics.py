import statistics as st
from simple_pid import PID
import rospy
import miro2 as miro
import os
import numpy as np
import math
import time
#TODO: confirm if you can achieve the same with the time module as well
import datetime as dt
import sys
import csv
import geocoder
import ssl
import smtplib

from fallen_analyser.msg import coords
from std_msgs.msg import Float32MultiArray, UInt16MultiArray, UInt32MultiArray
from std_msgs.msg import UInt8, UInt16, UInt32, Bool
from geometry_msgs.msg import Twist, TwistStamped
from sensor_msgs.msg import JointState, Imu
from Errors import *
from  miro2.lib.RobotInterface import RobotInterface
from email.message import EmailMessage

NODE_NAME = "MiRoKinematics"
BUFFER_MAX = 5
#TODO: you will need to make your own sound directories which will have your own sounds
HELP_SIGNAL = ""

#TODO: you will need to change this to something like 10 seconds
#get help after 4 seconds
HELP_TIMER = 8
#TODO: the problem with this is because it's too big
#SONAR_MAX = 0.50
SONAR_MAX = 0.25

#setting it to 2 minutes for testing purposes 
#HELP_TIMER = 120

#TODO: when you will initialise the package, you'll need to make sure that MiRo's eyelids are going to be open
#TODO: When the head is being moved, you want to ignore the incoming coordinates from the pose_fallen node
#TODO: you'll need to transform the use of lists into using arrays for faster access time


class MiRoKinematics:

    def __init__(self):
        self.__verbose = False
        self.__headMoving = False
        #if resident has already fallen, ignore messages from resident/fallen/
        #topic, and initiate the MiRo fallen interaction
        self.__fallenState = False
        self.__miroFinishedSpeaking = False
        self.__kinHead = JointState()
        self.__kinHead.position = [0.0, math.radians(34.0), 0.0, 0.0]
        self.__startTime = 0.0
        #self.__kinHead.name = ["TILT", "LIFT", "YAW", "PITCH"]
        self.__facePos = (0,0)
        self.__ledTimer = 0
        self.__sensorInfo = None
        self.__miroStopped = False
        self.__LEDToggle = True
        self.__residentOkay = False
        rospy.init_node(NODE_NAME, anonymous=True)
        self.__coordsRight = False

        topicBaseName = "/" + os.getenv("MIRO_ROBOT_NAME")

        #SUBSCRIBERS
        #subscribing to sonar and cliff sensors of MiRo
        self.sensorPackage = rospy.Subscriber(topicBaseName + "/sensors/package",
            miro.msg.sensors_package, self.callBackSensorPackage, queue_size=1,
            tcp_nodelay=True)

        rospy.Subscriber("resident/fallen/", Bool, self.callBackHasFallen)
        rospy.Subscriber("resident/coords/", coords, self.callbackCoords)
        rospy.Subscriber("resident/startTimer/", Bool, self.callBackTimer)
        rospy.Subscriber("resident/residentOkay/", Bool,
                self.callBackResidentOkay)

        #PUBLISHERS
        self.pubIllum = rospy.Publisher(topicBaseName + "/control/illum",
                UInt32MultiArray, queue_size=0)

        topic = topicBaseName + "/control/kinematic_joints"
        self.__pubKin = rospy.Publisher(topic, JointState, queue_size=0)

        topic = topicBaseName + "/control/cmd_vel"
        self.__pubWheels = rospy.Publisher(topic, TwistStamped, queue_size=0)

        self.__pubWarningSignal = rospy.Publisher("resident/warningSignal/",
                Bool, queue_size=0)

        self.__pubMiroSpeak = rospy.Publisher("resident/miroSpeak/", Bool,
                queue_size=0)

        self.__pubMiroIntro = rospy.Publisher("resident/miroIntro/", Bool,
                queue_size=0)




        #TODO: you will add some code here to make sure that the LED lights are switched off

    @property
    def verbose(self):
        return self.__verbose

    #TODO: you will need to make this consistent with all the accessors
    def getSonarReadings(self):
        """
        IMPORT:
        EXPORT:

        PURPOSE:
        """

        sonarReading = None

        if not self.__sensorInfo is None:

            currSensorInfo = self.__sensorInfo
            #clearing sensor information not to clutter memory and 
            #get convoluted readings
            self.__sensorInfo = None
            sonarReading = currSensorInfo.sonar.range

        return sonarReading

    @verbose.setter
    def verbose(self, newVerbose):
        self.__verbose = self.__validateVerbose(newVerbose)

    def moveHeadCords(self):
        """
        ASSERTION: Moves head to given xCord and yCord, if points are wthin
        MiRo's workspace
        """
        timeHeadDelay = 0

        while not rospy.core.is_shutdown():
            if self.__verbose:
                rospy.loginfo_once("Moving head given subscribed coordinates")

            timeHeadDelay = time.time() - self.__startTime


            rospy.loginfo("x: %s " % self.__facePos[0])
            self.__kinHead.position[miro.constants.JOINT_YAW] = \
                self.moveHead2XCoord(self.__facePos[0])

            self.checkHeadWorkspaceYaw()
            self.checkHeadWorkSpaceLift()
            self.checkHeadWorkSpacePitch()
            rospy.loginfo("yaw: %s" % self.__kinHead.position[miro.constants.JOINT_YAW])


            #only publish commands when the head is stationary
            self.__pubKin.publish(self.__kinHead)



            #don't want to change the position wile the head is moving 
            #if abs(self.__facePos[0]) > 100 and not self.__headMoving:
            """
            if abs(self.__facePos[0]) > 100 and not self.__headMoving == True:
                print("x: ", self.__facePos[0])
                self.__kinHead.position[miro.constants.JOINT_YAW] += \
                    self.moveHead2XCoord(self.__facePos[0])

                print("yaw: ", self.__kinHead.position[miro.constants.JOINT_YAW])
                self.__headMoving = True
                self.__startTime = time.time()



            if abs(self.__facePos[1]) > 30 and not self.__headMoving == True:
                self.__kinHead.position[miro.constants.JOINT_LIFT] += \
                    self.moveHead2YCoord(self.__facePos[1])

                self.__headMoving = True
                self.__startTime = time.time()

            if not self.__headMoving == True:
                self.__headMoving = True
                self.__startTime = time.time()

            if self.__headMoving == True:
                pass


            if timeHeadDelay > 1:
                self.__headMoving = False
                #resetting the delay back to 0
                timeHeadDelay = 0

            if self.__verbose:
                rospy.loginfo("LIFT: %.2f , PITCH: %.2f , YAW: %.2f" %
                        (self.__kinHead.position[miro.constants.JOINT_TILT],
                            self.__kinHead.position[miro.constants.JOINT_PITCH],
                            self.__kinHead.position[miro.constants.JOINT_YAW]))
            """



            #TODO: you will need to remember the sleep time for this
            #time.sleep(0.02)
            time.sleep(0.5)

    def medianKernel(self, dataPts, newPt):
        """
        IMPORT:
        EXPORT:

        PURPOSE:
        """

        dataPts = np.roll(dataPts, -1)
        size = len(dataPts) - 1

        #replacing the last element with new data
        dataPts[size] = newPt
        median = np.median(dataPts)

        return median, dataPts

    def respondFallen(self):
        """
        PURPOSE:
        """

        sleepTime = 0.02
        frequency = 1 / 0.02
        helpTimerStart = 0.0
        elapsedTime = 0.0
        elapsedHelpTime = 0.0

        #initial variables associated with the PID controller for the drive sub-system
        control = 1
        startTime = time.time()
        elapsedTime = 0.0
        reading = 0.0

        #creating a log file, so we can observe how MiRo stops
        fileName = "/home/parallels/Desktop/Thesis/data/sonar_pid_control/data.csv"
        headers = ["Time (secs)", "Control variable"]
        self.createFile(fileName, headers)

        pid = self.generatePID()
        #TODO: do you really need this sleep timer here at all?
        time.sleep(0.02)

        speakToggle = False
        emailSent = False

        #code for filtering points on the data 
        kernel = np.zeros(25)

        while not rospy.core.is_shutdown():
            #when resident has fallen over
            self.logDataFile(fileName, headers, control, startTime)
            if self.__fallenState and not self.__miroStopped:
                rospy.loginfo_once("Getting to fallen resident...")
                #checking if the sonar sensor is close to something
            #if they is some sensor information, we want to determine what to do
                sonarReading = self.getSonarReadings()
                #filtering the sonar sensor
                sonarReading, kernel = self.medianKernel(kernel, sonarReading)

                print("SONAR: %s " % sonarReading)
                self.activateWheels(control)
                control = self.determinePIDControl(pid, sonarReading, control)
                #self.logDataFile(fileName, headers, control, startTime)

                #if sonarReading <= 0.4:
               # if sonarReading <= SONAR_MAX:

                #when PID control has reached the set point
                if control == 0:
                    #print("YES I AM inside")
                    self.__miroStopped = True
                    #helpTimerStart = time.time()

                        #starting the timer

            #print("STATUS: %s " % self.__miroFinishedSpeaking)
            if not self.__miroFinishedSpeaking:
                #print("START TIMER PARTNER")
                helpTimerStart = time.time()


            #if miro has stopped and the person has fallen, want to approach person
            if self.__fallenState and self.__miroStopped:
                #you want to turn on the LED lights to a particular color
                rospy.loginfo_once("Social Interaction MiRO")
                #TODO: I am currently here at the moment. You have to change this to intro
                #TODO: you will have to come back and toggle MiRo's intro
                speakToggle = self.toggleIntro(speakToggle)

                #once MiRo has finished speaking, we want to listen

                #TODO: you will have to make a toggle variable for this, so miro will speak only once
                if self.__miroFinishedSpeaking:
                    miroSpeak = Bool()
                    miroSpeak.data = True
                    self.__pubMiroSpeak.publish(miroSpeak)
                    #waiting for this command to actually register
                    time.sleep(0.1)
                    miroSpeak.data = False
                    self.__pubMiroSpeak.publish(miroSpeak)

                #checking if the resident is okay
                self.residentOkayHaptic()

                #only start the timer once Miro has finished talking

                #only count the time if the resident is not okay
                if not(self.__residentOkay) and self.__miroFinishedSpeaking:
                    elapsedHelpTime = time.time() - helpTimerStart
                    #print("CURRENT TIME: %s " % elapsedHelpTime)

                warningSignal = Bool()
                #waiting for 10 seconds
                if elapsedHelpTime >= HELP_TIMER and not(self.__residentOkay):
                    rospy.loginfo_once("The resident is NOT okay please help ...")
                    red = np.array([255, 0, 0])
                    self.activateSOSLED(sleepTime, frequency, red, 255)
                    self.getAttention()
                    warningSignal.data = True
                    self.__pubWarningSignal.publish(warningSignal)

                    #ensuring that an e-mail is only sent once 
                    if not emailSent:
                        emailSent = True
                        location = self.getLocation()
                        self.sendEmail(location)

                #print("RESIDENT STATUS: %s " % self.__residentOkay)
                if self.__residentOkay:
                    #re-setting the timers, and variables used 
                    elapsedTime = 0.0
                    helpTimerStart = 0.0
                    rospy.loginfo_once("Resident is okay now")
                    warningSignal.data = False
                    self.__pubWarningSignal.publish(warningSignal)
                    self.turnLEDOff()
                    sys.exit()

                    #TODO: you will need to figure out a better sequence to this
                    #if (self.residentOkayHaptic()):
                        #rospy.loginfo("I am switching off now :)")
                        #sys.exit()


            time.sleep(sleepTime)

    def toggleIntro(self, inCondition):
        """
        IMPORT: Boolean
        EXPORT: NONE

        PURPOSE: To tell MiRo to speak introducer itself only once.
        """
        miroSpeak = Bool()

        if not inCondition:
            miroSpeak.data = True
            self.__pubMiroIntro.publish(miroSpeak)
            #waiting for this command to actually register
            time.sleep(0.1)
            miroSpeak.data = False
            self.__pubMiroIntro.publish(miroSpeak)
            #switching this off afterwards, so MiRo doesn't talk again
            inCondition = True

        return inCondition

    def moveHead2XCoord(self, dx):
        """
        ASSERTION:
        """
        retValue = 0

        """
        if self.__facePos[0] > 200:
            retValue =  math.radians((dx)*-0.1)
        elif self.__facePos[0] < 100:
            #retValue = math.radians((dx)*0.1)
            retValue = math.radians(10.0)
        """

        if self.__coordsRight:
            #equation to get results from right camera
            retValue =  math.radians((dx)*-0.1)
        #I want to explicitly look for false value, and do nothing for None
        elif self.__coordsRight == False:
            #equation to get results from left camera
            retValue +=  miro.constants.YAW_RAD_MAX - math.radians((dx)*0.1)

        #moving the head on the left hemisphere of the head

        #the middle of 
        """
        self.__startTime = time.time()
        self.__headMoving = True
        """
        return retValue
        #return math.radians((dx)*0.05)

    def rotateBodyRight(self):
        """
        ASSERTION: Rotates Miro to the right 1 degree at a time
        """

        if self.__verbose:
            rospy.loginfo("rotating Miro's body right")

        msgWheels = TwistStamped()
        msgWheels.twist.angular.z = 1.0
        self.__pubWheels.publish(msgWheels)
        #giving time for the command to fully execute
        time.sleep(0.01)

        return 1

    def rotateBodyLeft(self):
        """
        ASSERTION: Rotates Miro to the left 1 degree at a time
        """

        if self.__verbose:
            rospy.loginfo("rotating MiRo's body left")

        msgWheels = TwistStamped()
        msgWheels.twist.angular.z = -1.0
        self.__pubWheels.publish(msgWheels)

        time.sleep(0.01)

        return 1

    def getAttention(self):
        """
        PURPOSE:
        """

        #setting it to spin in clockwise direction
        spin = -1.0
        msgWheels = TwistStamped()
        #msgWheels.twist.linear.x = miro.constants.WHEEL_MAX_SPEED_M_PER_S*0.5
        msgWheels.twist.linear.x = 0.0
        #TODO: you can add timers on this variable to make it rotate for a period of time
        v = 1.0
        msgWheels.twist.angular.z = v * 6.2832 * spin
        self.__pubWheels.publish(msgWheels)

    def wagTail(self):
        """
        ASSERTION: Wags MiRo's tail for a set time to communicate happiness.
        """
        #TODO: come back and finish this  off

        if self.__verbose:
            rospy.loginfo("Wagging MiRo's tail")

    def residentOkayHaptic(self):
        """
        ASSERTION: Will return true if the body of Miro is touched
        """

        touched = False
        if not self.__sensorInfo is None:
            headSensor =  self.__sensorInfo.touch_head.data
            #accounting for the sensors which are already pressed
            bodySensor = self.__sensorInfo.touch_body.data - 14208

            if self.__verbose:
                rospy.loginfo("headSensor: %s bodySensor: %s " % (headSensor, 
                    bodySensor))

            if (headSensor > 0) | (bodySensor > 0):
                self.__residentOkay = True
                touched = True
                rospy.loginfo("You touched me ...")

        return touched

        #grabbing the head sensor data


    def getLocation(self):
        """
        IMPORT:
        EXPORT:

        PURPOSE:
        """

        g = geocoder.ip('me')

        locInfo = {
                "latlng" : g.latlng,
                "post_code" : g.postal,
                "street" : g.street,
                "state" : g.state,
                "country" : g.country,
                "city" : g.city,
                "number" : g.housenumber
                }

        return locInfo

    def sendEmail(self, location):
        """
        IMPORT:
        EXPORT:

        PURPOSE:
        """


        emailSender = "consquentimiro@gmail.com"
        emailPassword = "qahjnpcnmbfqsrrc"
        #you can put the e-mail address of the age-care here 
        emailReceiver = "19476700@student.curtin.edu.au"

        #getting the current time of where the resident has fallen
        currentTime = dt.datetime.now()

        subject = "FALLEN RESIDENT %s %s" % (currentTime.date(),
                str(currentTime.time()).split(".")[0])
        body  = "ALERT Resident has fallen over: \n \n" + \
                "\tLocation: %s " % location["latlng"] + \
                "\n\t Address: %s %s %s, %s" % (location["number"],
                        location["street"], location["city"], location["state"]) + \
                "\n\t Post Code: %s " % location["post_code"] + \
            "\n \n Kind Regards, \n \n MiRo"

        em = EmailMessage()

        em['From'] = emailSender
        em['To'] = emailReceiver
        em['Subject'] = subject
        em.set_content(body)

        context = ssl.create_default_context()

        with smtplib.SMTP_SSL('smtp.gmail.com', 465, context=context) as smtp:
            #logging into the g-mail account
            smtp.login(emailSender, emailPassword)
            smtp.sendmail(emailSender, emailReceiver, em.as_string())
    def turnLEDOn(self, rgb, bright):
        """
        PURPOSE
        """

        frontLeft, midLeft, rearLeft, frontRight, midRight, rearRight = range(6)
        msgIllum = UInt32MultiArray()
        msgIllum.data = [0, 0, 0, 0, 0, 0]

        value = self.__generateIllum(rgb, bright)

        #turning front section of lights on
        msgIllum.data[frontLeft] = value
        msgIllum.data[frontRight] = value

        #turning midsection of lights on
        msgIllum.data[midLeft] = value
        msgIllum.data[midRight] = value

        #turning rear section of lights on
        msgIllum.data[rearLeft] = value
        msgIllum.data[rearRight] = value

        self.pubIllum.publish(msgIllum)

    def activateSOSLED(self, sleep, frequency, color, bright):
        """
        ASSERTION: MiRO will continuously turn on the LED on and off with
        intervals of 1 second given the specified color and brightness
        """
        self.__ledTimer += 1


        if ((self.__ledTimer % frequency) == 0) and self.__LEDToggle:
            self.turnLEDOn(np.array([255,0,0]), 255)
            self.__LEDToggle = False
        else:
            self.turnLEDOff()
            self.__LEDToggle = True

    def turnLEDOff(self):
        """
        ASSERTION: Turns all sections of MiRO
        """
        msgIllum = UInt32MultiArray()
        msgIllum.data = [0, 0, 0, 0, 0, 0]
        self.pubIllum.publish(msgIllum)

    def createFile(self, fileName, inFieldNames):
        """
        IMPORT:
        EXPORT:

        PURPOSE:
        """
        with open(fileName, "w") as outStrm:
            csvWriter = csv.DictWriter(outStrm, fieldnames=inFieldNames)
            csvWriter.writeheader()

    def writeInfo(self, fileName, headers, xData, yData):
        """
        IMPORT:
        EXPORT:

        PURPOSE:
        """

        with open(fileName, 'a') as outStream:
            csvWriter = csv.DictWriter(outStream, fieldnames=headers)

            info = {
                    headers[0] : xData,
                    headers[1] : yData
                }

            csvWriter.writerow(info)

    #breaking up PID stuff into functions, so it will be easier to input into the classes defined above

    def generatePID(self, P=-3, I=0, D=-0.03):
        """
        IMPORT:
        EXPORT:

        PURPOSE:
        """
        pid = PID(P,I,D, setpoint=SONAR_MAX)
        #restraining the outputs between 0 and 1, so we don't blow up motors
        pid.output_limits = (0,1)

        return pid

    def createRecordDataFile(self, P, I, D):
        """
        IMPORT:
        EXPORT:

        PURPOSE:
        """
        pidSetting = "P=" + str(P) + ",I=" + str(I) + ",D=" + str(D) + ".csv"
        fileName = "/home/parallels/Desktop/Thesis/data/sonar_pid_control/data" + pidSetting
        headers = ["Time (secs)", "Control variable"]
        #creating the file to place the data inside of
        createFile(fileName, headers)

    #TODO: you will need to refactor this so you can change the file name and the headers when you call the function
    def createLogDataFile(self):
        """
        IMPORT:
        EXPORT:

        PURPOSE:
        """

        headers = ["Time (secs)", "Control variable"]
        #fixed location to log data, as accompanying function will read data from here 
        fileName = "/home/parallels/Desktop/Thesis/data/sonar_pid_control/data.csv"
        self.createFile(fileName, headers)

    def determinePIDControl(self, pid, reading, prevControl):
        """
        IMPORT:
        EXPORT:

        PURPOSE:
        """

        #if we can't determine a new control variable, we want to use previous one
        control = prevControl

        if reading:
            #calculating the new output form the PID according to the systems current value
            if reading < float("inf"):
                #print("reading: %s " % reading)
                #me just trying to understand the control variable a little bit
                #better for myself
                control = pid(reading)
                print("Sonar reading: %s" % reading)
                print("Control: %s " % (control))
                #print("Control variable: % s" % control)

        return control

    def logDataFile(self, fileName, headers, inX, startTime):
        """
        IMPORT:
        EXPORT:

        PURPOSE:
        """
        elapsedTime = time.time() - startTime
        self.writeInfo(fileName, headers, elapsedTime, inX)

        return elapsedTime

    def activateWheels(self, control):
        """
        IMPORT:
        EXPORT:

        PURPOSE:
        """

        msgWheels = TwistStamped()
        #this is the maximum forward speed of MiRo
        msgWheels.twist.linear.x = miro.constants.WHEEL_MAX_SPEED_M_PER_S * control
        msgWheels.twist.angular.z = 0.0
        self.__pubWheels.publish(msgWheels)

    #TODO: I honestly don't know what this function is actually going to do
    def wiggle(self, v, n, m):
        """
        ASSERTION: Wiggles MiRo's tail within the tail's work space
        """
        #TODO: come back and finish this  off
        v = v + float(n) / float(m)
        if v > 2.0:
            v -= 2.0
        elif v > 1.0:
            v = 2.0 - v
        return v

        #stop node from being excited until node has being stopped

    def moveHead2YCoord(self, dy):
        """
        ASSERTION:
        """
        self.__startTime = time.time()
        self.__headMoving = True
        #THIS IS GOING TO BE WITH LIFT
        return math.radians((abs(dy)-20)*np.sign(dy)*0.3)

    def checkHeadWorkSpaceLift(self):
        """
        ASSERTION: Will stop MiRo's head from moving past it's workspace in the
        lift direction.
        """

        workspaceOkay = True

        if self.__kinHead.position[miro.constants.JOINT_LIFT] >= miro.constants.LIFT_RAD_MAX:
            self.__kinHead.position[miro.constants.JOINT_LIFT] = \
            miro.constants.LIFT_RAD_MAX - BUFFER_MAX
            workspaceOkay = False
            self.__headMoving = True

            if self.__verbose:
                rospy.loginfo("Miro has reached lift max")

        if self.__kinHead.position[miro.constants.JOINT_LIFT] <= miro.constants.LIFT_RAD_MIN:
            self.__kinHead.position[miro.constants.JOINT_TILT] = \
            miro.constants.LIFT_RAD_MIN + BUFFER_MAX
            workspaceOkay = False
            self.__headMoving = True

            if self.__verbose:
                rospy.loginfo("Miro has reached lift minimum")


        return workspaceOkay

    def checkHeadWorkSpacePitch(self):
        """
        ASSERTION: Will stop MiRo's head from moving past it's workspace in the
        pitch direction.
        """

        workspaceOkay = True

        if self.__kinHead.position[miro.constants.JOINT_PITCH] >= miro.constants.PITCH_RAD_MAX:
            self.__kinHead.position[miro.constants.JOINT_PITCH] = \
            miro.constants.PITCH_RAD_MAX - BUFFER_MAX

            workspaceOkay = False

            if self.__verbose:
                rospy.loginfo("Miro has reached pitch's max")

        if self.__kinHead.position[miro.constants.JOINT_PITCH] <= miro.constants.PITCH_RAD_MIN:
            self.__kinHead.position[miro.constants.JOINT_PITCH] = \
            miro.constants.PITCH_RAD_MIN + BUFFER_MAX

            workspaceOkay = False

            if self.__verbose:
                rospy.loginfo("Miro has reached pitch's min")

        return workspaceOkay

    def checkHeadWorkspaceYaw(self):
        """
        ASSERTION: Will stop MiRo's head from past it's workspace in the yaw
        direction.
        """
        workspaceOkay = True

        if self.__kinHead.position[miro.constants.JOINT_YAW] >= miro.constants.YAW_RAD_MAX:
            self.__kinHead.position[miro.constants.JOINT_YAW] = \
            miro.constants.YAW_RAD_MAX - BUFFER_MAX

            workspaceOkay = False
            self.__headMoving = True
            if self.__verbose:
                rospy.loginfo("Miro has reached yaw's max")

        if self.__kinHead.position[miro.constants.JOINT_YAW] <= miro.constants.YAW_RAD_MIN:
            self.__kinHead.position[miro.constants.JOINT_YAW] = \
                miro.constants.YAW_RAD_MIN + BUFFER_MAX


            workspaceOkay = False
            self.__headMoving = True
            if self.__verbose:
                rospy.loginfo("Miro has reached yaw's minimum")

        return workspaceOkay

    def callbackCoords(self, data):
        """
        ASSERTION: This will unpack subscribed data from resident/coords/ topic and
        assign data to the facePos class field
        """
        #add another point to the path of residents head
        self.__facePos = (int(data.xCord.data), int(data.yCord.data))
        self.__coordsRight = bool(data.rightCam.data)

        #rospy.loginfo("Coords Right: %s " % self.__coordsRight)

        if self.__verbose:
            rospy.loginfo("received Coordinates: (%.3f, %.3f)" %
                    (data.xCord.data, data.yCord.data))

    def callBackHasFallen(self, data):
        """
        ASSERTION: This function will unpack subscribed data from resident/fallen/
        topic and assign the fallenState class field to True
        """

        #if the fallen state has already being activated and hasn't be cleared,
        #we don't want to accept any new incoming data
        if not self.__fallenState:
            self.__fallenState = bool(data.data)

        if self.__verbose:
            rospy.loginfo("Resident hasn't fallen: %s " % data.data)

    def callBackSensorPackage(self, msg):
        self.__sensorInfo = msg

    def callBackTimer(self, data):
        """
        IMPORT:
        EXPORT:

        PURPOSE:
        """
        self.__miroFinishedSpeaking = bool(data.data)

    def callBackResidentOkay(self, data):
        """
        IMPORT:
        EXPORT:

        PURPOSE:
        """
        self.__residentOkay = bool(data.data)


    def callbackKin(self, msg):
        """
        """
        #TODO: you'll need to have some protection, so that if it's not called it will do nothing
        self.__kinHead = msg.position

    #TODO: come back and expand on this idea and see what can come out of it
    def cleanUp(self):
        self.turnLEDOff()
        print("you have shut me down mate")

    def __validateVerbose(self, inVerbose):
        if type(inVerbose) is not(bool):
            MiRoError("Verbose muse be a boolean either true or false")

        return inVerbose

    def __generateIllum(self, rgb, bright):
        """
        ASSERTION: Generates MiRo specific code to produce color given rgb import
        with specified brightness by the bright import.
        """
        red = rgb[0]
        green = rgb[1]
        blue = rgb[2]
        return (int(bright) << 24) | (red << 16) | (green << 8) | blue


def SMA(dataPts, newPt):
    """
    IMPORT:
    EXPORT:

    PURPOSE:
    """

    dataPts = np.roll(dataPts, -1)
    size = len(dataPts) - 1

    #replacing the last element with new data
    dataPts[size] = newPt
    average = np.mean(dataPts)

    return average, dataPts

def medianKernel(dataPts, newPt):
    """
    IMPORT:
    EXPORT:

    PURPOSE:
    """

    dataPts = np.roll(dataPts, -1)
    size = len(dataPts) - 1

    #replacing the last element with new data
    dataPts[size] = newPt
    median = np.median(dataPts)

    return median, dataPts
    #you can us numpy's median function

def modeKernel(dataPts, newPt):
    """
    IMPORT:
    EXPORT:

    PURPOSE:
    """

    dataPts = np.roll(dataPts, -1)
    size = len(dataPts) - 1

    #replacing the last element with new data
    dataPts[size] = newPt
    mode = st.mode(dataPts)

    return mode, dataPts

def testSonarSensor():
    """
    IMPORT:
    EXPORT:

    PURPOSE:
    """

    #TODO: you will need to come back to this, and implement it after you have integrated it into your main program


if __name__ == "__main__":
    miroRobot = MiRoKinematics()
    #miroRobot.verbose = True
    #TODO: write in your report how hard this was to actually implement it in code
    #miroRobot.moveHeadCords()

    #TODO: you will need to uncomment this, and explore this a little bit later
    miroRobot.respondFallen()

    #testing if I can control the wheels from here or not 


    #TODO: code used for testing the filters for the sonar sensor, make sure to put this in a function later on
    """
    fileNameBase = "/home/parallels/Desktop/Thesis/data/sonar_filtering/"
    headers = ["Time (secs)", "Sonar Distance (m)"]


    fileName = fileNameBase + "Raw Data.csv"
    miroRobot.createFile(fileName, headers)
    #SMA - Simple Moving Average
    #miroRobot.createFile(fileNameBase + "SMA.csv", headers)
    startTime = time.time()

    #initialising the kernels to use

    #looking at the last 10 data points
    kernelSize = 30 # a proof of concept
    kernel = np.zeros(kernelSize)
    elapsedTime = 0

    while not rospy.core.is_shutdown():
        reading = miroRobot.getSonarReadings()

        miroRobot.activateWheels(1)

        if reading:
            if reading < float("inf"):
                #rounded data to 2 decimal place
                reading = round(reading, 2)
                #reading = medianKernel(kernel, reading)
                #reading, kernel = modeKernel(kernel, reading)

                elapsedTime = miroRobot.logDataFile(fileName,
                        headers,
                        reading,
                        startTime)


        #only recording data for 10 seconds
        if elapsedTime >= 3:
            sys.exit()



        #time.sleep(0.02)
        time.sleep(0.02)
        """

    rospy.on_shutdown(miroRobot.cleanUp)
    RobotInterface.disconnect
