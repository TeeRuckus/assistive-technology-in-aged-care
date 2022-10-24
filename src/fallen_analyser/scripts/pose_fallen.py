import mediapipe as mp
import numpy as np
import cv2
import rospy
import sys
import os
from Errors import *
import time
from fallen_analyser.msg import coords
import miro2 as miro
import glob
import csv

from  miro2.lib.RobotInterface import RobotInterface
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, Float32MultiArray, String
from cv_bridge import CvBridge, CvBridgeError

VISIBILITY_THRESHOLD = 0.4
#FALEN_DISTANCE_THRESHOLD = 240
#FALEN_DISTANCE_THRESHOLD = 300
#FALEN_DISTANCE_THRESHOLD = 350
#FALEN_DISTANCE_THRESHOLD = 290
FALEN_DISTANCE_THRESHOLD = 250
NODE_NAME = "pose_fallen"
FALLEN_COUNT_THRESHOLD = 6
SAVE_PATH_L = "/home/parallels/Desktop/Thesis/data/video/raw_frames/left_cam/"
SAVE_PATH_R = "/home/parallels/Desktop/Thesis/data/video/raw_frames/right_cam/"


#TODO: You'll need to test if the conversation with constants have worked here
#TODO: You'll need to do the counts of the falls for the pose algorithm as well

class PoseFallen():
    def __init__(self,args, dectConf=0.1, trackConf=0.1):
        self.__mpDrawing = mp.solutions.drawing_utils
        self.__imageStitcher = None
        self.__inputCamera = [None] * 3
        self.__mpHolistic = mp.solutions.holistic
        self.__imageConverter = CvBridge()
        self.__dectConf = dectConf
        self.__trackConf = trackConf
        self.__cvBridge = CvBridge()
        #to store images for left, right, and stitched images
        self.__miroImgs = [None]*3
        self.__verbose = False
        self.__bbox = False
        self.__pose = False
        self.__mode = self.__setMode(args)
        #counter to make sure that the person in the frame has fallen, and it's not 
        #a false negative of fallen person
        self.__fallenCounter = 0
        self.__leftCamFallen = False
        self.__rightCamFallen = False
        self.__coordsCamRight = False

        #things for the face detection
        self.__mpFaceDetection = mp.solutions.face_detection

        #subscribing to the required nodes and the data
        if self.__verbose:
            rospy.loginfo("Creating main node %s" % NODE_NAME)

        rospy.init_node(NODE_NAME, anonymous=True)
        topicBaseName = "/" + os.getenv("MIRO_ROBOT_NAME")


        #TODO: You'll need to come back and delete this CODE
        camLTopic = topicBaseName + "/sensors/caml/compressed"
        camRTopic = topicBaseName + "/sensors/camr/compressed"
        #TODO: you'll have to come back and see if this is going to be necessary

        #SUBSCRIBERS

        if self.__verbose:
            rospy.loginfo("subscribing to left and right camera ... ")

        self.__camL = rospy.Subscriber(camLTopic, CompressedImage,
                self.callbackCamL, queue_size=1, tcp_nodelay=True)

        self.__camR = rospy.Subscriber(camRTopic, CompressedImage,
                self.callbackCamR, queue_size=1, tcp_nodelay=True)

        if self.__verbose:
            rospy.loginfo("Creating Publisher, to publish coordinates")

        #PUBLISHERS

        #TODO: uncomment this out, this was just for testing purposes
        self.__coordsPub = rospy.Publisher("resident/coords/",
                coords, queue_size=10)
        #TODO: play with the queue size and see if you will get  better performance
        self.__fallenPublisher = rospy.Publisher("resident/fallen/", Bool,
                queue_size=0)

        #TODO Come back and play with the quality of the camera
        #publisher for camera quality
        self.__pubCmd = rospy.Publisher(topicBaseName + "/control/command", String, queue_size=0)


    @property
    def imageStitcher(self):
        """
        PURPOSE: A getter for the imageStitcher class field
        """
        return self.__imageStitcher

    @property
    def mode(self):
        """
        PURPOSE: A getter for the mode class field
        """
        return self.__mode

    @property
    def inputCamera(self):
        """
        PURPOSE: A getter for the inputCamera class field
        """
        return self.__inputCamera

    def findPose(self, img, draw=True):
        """
        PURPOSE: To implement mediapipe on the found image, and trying
        to find the pose on the found image at the current moment
        """

        results = None
        with self.__mpHolistic.Holistic(
                min_detection_confidence=self.__dectConf,
                min_tracking_confidence=self.__trackConf) as holistic:

            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            results = holistic.process(img)

            if results:
                if draw:
                    self.__mpDrawing.draw_landmarks(img, 
                            results.pose_landmarks,
                            self.__mpHolistic.POSE_CONNECTIONS, 
                            self.__mpDrawing.DrawingSpec(color=(66,117,245), thickness=2, circle_radius=2),
                            self.__mpDrawing.DrawingSpec(color=(230,66,245), thickness=2, circle_radius=2))


        return results, cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

    def torsoInView(self, results):
        """
        """
        inView = False

        landmarkFound = results.pose_landmarks

        #checking if any landmarks are found in the image
        if landmarkFound:
            landmark = landmarkFound.landmark
            leftShoulder = landmark[self.__mpHolistic.PoseLandmark.LEFT_SHOULDER.value].visibility
            rightShoulder = landmark[self.__mpHolistic.PoseLandmark.RIGHT_SHOULDER.value].visibility
            leftHip = landmark[self.__mpHolistic.PoseLandmark.LEFT_HIP.value].visibility
            rightHip = landmark[self.__mpHolistic.PoseLandmark.RIGHT_HIP.value].visibility


            if (leftShoulder >= VISIBILITY_THRESHOLD) & (rightShoulder >= VISIBILITY_THRESHOLD) \
                & (leftHip > VISIBILITY_THRESHOLD) & (rightHip > VISIBILITY_THRESHOLD):
                    inView = True

        return inView

    def noseHipInView(self, results):
        """
        """
        inView = False

        landMarkFound = results.pose_landmarks

        if landMarkFound: 
            landmark = landMarkFound.landmark
            nose = landmark[self.__mpHolistic.PoseLandmark.NOSE.value].visibility
            hip = landmark[self.__mpHolistic.PoseLandmark.RIGHT_HIP.value].visibility

            if (nose >= VISIBILITY_THRESHOLD) & (hip >= VISIBILITY_THRESHOLD):
                inView = True

        return inView

    def orientationOfTorso(self, results, imgShape):
        """

        """
        fallen = None
        bbox = []

        landmark = results.pose_landmarks

        #checking if any landmarks are found in the image
        if landmark:
            fallen = False
            landmark = landmark.landmark

            lShoulder = landmark[self.__mpHolistic.PoseLandmark.LEFT_SHOULDER.value]
            rShoulder = landmark[self.__mpHolistic.PoseLandmark.RIGHT_SHOULDER.value]
            lHip = landmark[self.__mpHolistic.PoseLandmark.LEFT_HIP.value]
            rHip = landmark[self.__mpHolistic.PoseLandmark.RIGHT_HIP.value]

            #normalising the x and y direction, so it's not a value from [0,1]
            #but instead a value in relation to the dimensions of the image
            lShoulderCords = (lShoulder.x * imgShape[1] , lShoulder.y * imgShape[0])
            rShoulderCords = (rShoulder.x * imgShape[1], rShoulder.y * imgShape[0])
            lHipCords =  (lHip.x * imgShape[1], lHip.y * imgShape[0])
            rHipCords = (rHip.x * imgShape[1] , rHip.y * imgShape[0])

            bbox.append(lShoulderCords[0])
            bbox.append(lShoulderCords[1])
            bbox.append(rHipCords[0])
            bbox.append(rHipCords[1])

            xmin, ymin, xmax, ymax = bbox[0], bbox[1], bbox[2], bbox[3]
            xSide = xmax - xmin
            ySide = ymax - ymin


            if abs(xSide) > abs(ySide):
                fallen = True

        return fallen , bbox

    def getDistance(self, results, imgShape):
        """
        PURPOSE: Gets distance of resident between their nose and knees. This
        distance is monitored to ensure resident has fallen if, distance below
        programmed threshold the resident has fallen and needs assistance.
        """

        distance = None
        landmarkFound = results.pose_landmarks

        #checking  if any landmarks are found in the image 
        if landmarkFound:
            landmark = landmarkFound.landmark
            head = landmark[self.__mpHolistic.PoseLandmark.NOSE.value]
            #we're going to assume that the left and right hip are going to be
            #mediapipe returns width and height numbers between [0,1]. Therefore,
            #normalising width and height to size of image
            headHeight = head.y * imgShape[0]

            #will only need to get distance of head, as measurements in 
            #mediapipe are relative to the hip joints
            distance = np.sqrt(pow(headHeight,2))

        return distance

    def hasFallenDistance(self, distance):
        """
        """

        fallen = False
        if distance >= FALEN_DISTANCE_THRESHOLD:
            fallen = True


        return fallen

    def getImage(self):
        """
        PURPOSE: To subscribe to the stereo cameras of MiRo, and to get the 
        necessary image file 
        """

    def publishFallen(self, fallenTrigger):
        """
        PURPOSE: To publish whether a resident has fallen or not from the
        previous frames of the algorithm
        """
        #TODO: you will need to build up some forgiveness in this algorithm for dropped out frame rates

        #to force a fall on this controller. Uncomment for testing
        """
        fallenTrigger = True
        self.__leftCamFallen = True
        self.__rightCamFallen = True
        """

        fallenStatus = Bool()
        rospy.loginfo("The current count: %s" % self.__fallenCounter)
        if fallenTrigger:
            self.__fallenCounter += 1
            if self.__fallenCounter >= FALLEN_COUNT_THRESHOLD:
                rospy.loginfo("THE PERSON HAS DEFINITELY FALLEN AND I SHOULD DO SOMETHING ABOUT IT")
                fallenStatus.data = True
                self.__fallenPublisher.publish(fallenStatus)

        #only reset if both cameras don't have a fallen resident in their frame
        if not(self.__leftCamFallen) and not(self.__rightCamFallen):
            fallenStatus.data = False
            self.__fallenPublisher.publish(fallenStatus)
            self.__fallenCounter = 0

    def callbackCam(self, rosImg, indx):
        """
        PURPOSE: To grab image data from a camera specified by index
        """

        if len(rosImg.data) == 0:
            print("dropped empty camera frame")
            return

        try:
            #converting ros image to raw openCV file 
            img = self.__imageConverter.compressed_imgmsg_to_cv2(rosImg, "rgb8")
            self.__inputCamera[indx] = img


        except CvBridgeError as e:
            print("callbackCam Error: ", e)

    def callbackCamL(self, rosImg):
        """
        PURPOSE: To obtain images from the left stereo camera of MiRO
        """
        self.callbackCam(rosImg, miro.constants.CAM_L)

    def callbackCamR(self, rosImg):
        """
        PURPOSE: To obtain images from the right stereo camera of MiRo
        """
        self.callbackCam(rosImg, miro.constants.CAM_R)

    def getVideoFeed(self):
        """
        PURPOSE: Responsible from getting raw video data from the stereo camera 
        of MiRo.
        """
        # state
        channelsToProcess = [0, 1]

        if not self.__imageStitcher is None:
            channelsToProcess = [2]

        outFile = [None, None, None]
        outCount = [0] * len(outFile)
        t0 = time.time()
        camNames = ['left', 'right', 'stitched']
        frameCountRight = 0
        frameCountLeft = 0


        #main loop for getting data from the stereo cameras of MiRo
        while not rospy.core.is_shutdown():

            #setting the resolution MiRo should be displaying its data
            #Here is a list of all possible resolutions: Param 2 is the frame rates
            #1280 x 720: param 1: 720w, param 2: 15
            #960 x 720: param 1: 720s, param 2: 15 
            #640 x  360 param 1: 360w, param 2: 15
            #480 x 360 param 1: 360s, param 2: 15
            #320 x 240 param 1 240s
            #320 x 180 param 1: 180w
            #240 x 180 param 1: 180s
            #if we're required to stitch the images
            cmd  = "frame=360w@15"
            self.__pubCmd.publish(cmd)

            if not self.__imageStitcher is None:
                #performing stitching process of the given images 

                #if both of the cameras are going to have images in them
                if not self.__inputCamera[0] is None and not self.__inputCamera[1] is None:
                    imgs = [self.__inputCamera[0], self.__inputCamera[1]]
                    self.__inputCamera[2] = cv2.hconcat(imgs)
                    self.__inputCamera[0] = None
                    self.__inputCamera[1] = None

            #only publish the images, if they is going to be an image to 
            #publish, and the only channels to process is either left or right
            for ii in channelsToProcess:
                #getting the current image
                img = self.__inputCamera[ii]


                #ensuring that we actually have an image, and not a dropped frame
                if not img is None:
                    #clearing the current image, as we're about to process
                    #the current image
                    self.__inputCamera[ii] = None


                    #code to save each of the individual frames to memory
#                    temp = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
#                    if ii == 0:
#                        frameCountLeft += 1
#                        cv2.imwrite(SAVE_PATH_L + "img_frame_" + str(frameCountLeft) + ".jpg"
#                                , temp)
#                    else:
#                        frameCountRight += 1
#                        cv2.imwrite(SAVE_PATH_R + "img_frame_" + str(frameCountRight) + ".jpg"
#                                , temp)


                    #getting results from the current frame
                    results, img = self.findPose(img)

                    _, img = self.blurFace(img)

                    #TODO: You will need to make an algorithm which will get the coordinates where the resident is the closest to the middle of the robot
                    #resultsPub, imgPub = self.findPose(
                            #self.__inputCamera[miro.constants.CAM_R])

                    #TODO: you will need to figure out how to actually make this work
#                    resultsPubL, imgPubL = self.findPose(
#                            self.__inputCamera[miro.constants.CAM_L])
#
#                    resultsPubR, imgPubR  = self.findPose(
#                            self.__inputCamera[miro.constants.CAM_R])
#
#
#                    #TODO: you will need to put this mess into a function
#                    self.selectEyeCoords(resultsPubL, imgPubL, resultsPubR,
#                            imgPubR)

                    if self.__mode == "show":

                        #TODO: you will need to move all these things in a function
                        if self.__bbox:
                            if  self.__verbose:
                                rospy.loginfo_once("Showing Bounding box mode")
                            img, hasFallen = self.showBBox(results, img)
                            self.selectFrameFallen(ii, hasFallen)

                        #TODO: you will need to move all these things in a function
                        elif self.__pose:
                            if self.__verbose:
                                rospy.loginfo_once("Showing pose from media pipe")
                            img, hasFallen = self.showPose(results, img)
                            self.selectFrameFallen(ii, hasFallen)

                        #correcting the color of the image
                        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                        cv2.imshow("Video from MiRo: " + camNames[ii], img)
                        cv2.waitKey(1)

                    #add more modes here for the program

                    #publishing the found node

            #processing frames at 50Hz. You can try 0.01HZ
            time.sleep(0.02)

        for ii in range(len(outFile)):
            pass

    def writeFile(self, fileName, data):
        """
        IMPORT:
        EXPORT:

        PURPOSE:
        """

        with open(fileName, "w") as outStrm:
            outStrm.write(data)

    def blurFace(self, img):
        """
        IMPORT:
        EXPORT:

        PURPOSE:
        """
        result = None
        height, width, _ = img.shape
        #TODO: I have left it on default settings for now, you will need to come back and adjust this for MiRo
        with self.__mpFaceDetection.FaceDetection( model_selection=1,
                min_detection_confidence=0.35) as faceDetection:


            #improving performance by marking the image as not writeable to
            #pass by reference
            img.flags.writeable = False
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            results = faceDetection.process(img)

            if results:
                #drawing the face detection annotations on the image
                if results.detections:
                    img.flags.writeable = True
                    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

                    bboxes = self.getFaceBoundingBox(results, img.shape)

                    #going through each bounding box which was found
                    for box in bboxes:
                        #points used to apply a bounding box mask over image
                        startPoint = (box[0], box[1])
                        endPoint = (box[0] + box[2], box[1] + box[3])

                        #creating bask so we can blur or pxelate the image
                        mask = np.zeros((height, width), np.uint8)

                        #blurring the image
                        imgCopy = img.copy()
                        #TODO: you can investigate the impacts of this on performance
                        imgCopy = cv2.blur(imgCopy, (100, 100))
                        cv2.rectangle(mask, startPoint, endPoint, 255, -1)
                        masked = cv2.bitwise_and(imgCopy, imgCopy, mask=mask)

                        #extract background
                        bgMask = cv2.bitwise_not(mask)
                        bg = cv2.bitwise_and(img, img, mask=bgMask)

                        #returning the blurred image to the user
                        retImage = cv2.add(bg, masked)
                #if we didn't detect anything just return original image
                else:
                    retImage = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        return results, retImage

    def getFaceBoundingBox(self, results, imgShape):
        """
        IMPORT:
        EXPORT:

        PURPOSE:
        """
        bboxes = []
        imgHeight, imgWidth, _ = imgShape

        #we want to go through each face which was found in the image and get the bounding box
        for detection in results.detections:
            bboxFeatures = detection.location_data.relative_bounding_box
            xmin = int(bboxFeatures.xmin * imgWidth)
            ymin = int(bboxFeatures.ymin * imgHeight)
            width = int(bboxFeatures.width * imgWidth)
            height = int(bboxFeatures.height * imgHeight)

            bboxes.append(np.array([xmin, ymin, width, height], np.int32))

        return bboxes

    def selectEyeCoords(self, resultsPubL, imgPubL, resultsPubR, imgPubR):
        """
        PURPOSE:
        """
        noseL = None
        noseR = None

        if resultsPubL.pose_landmarks:
            noseL = resultsPubL.pose_landmarks.landmark[self.__mpHolistic.PoseLandmark.NOSE.value]

        if resultsPubR.pose_landmarks:
            noseR = resultsPubR.pose_landmarks.landmark[self.__mpHolistic.PoseLandmark.NOSE.value]


        #only going to publish coordinates from the right eye
        coordsFound = self.getNoseCordinates(resultsPubR,
            imgPubR.shape)
        self.__coordsCamRight = True

        #if they're going to be present in both frames
        if noseL and noseR:
            #swap cameras if the visibility is better in the other lens
            if noseL.visibility > noseR.visibility:
                self.__coordsCamRight = False
                coordsFound = self.getNoseCordinates(resultsPubL,
                    imgPubL.shape)

                #in only going to be present in the left camera only
        if noseL and not  noseR:
            self.__coordsCamRight = False
            coordsFound = self.getNoseCordinates(resultsPubL,
                imgPubL.shape)


        #gaurding against publish none messages
        if coordsFound:
            self.publishCoords(coordsFound)

        if coordsFound and self.__verbose:
            rospy.loginfo("Coordinates: %s %s" % (coordsFound[0],
                coordsFound[1]))

    def publishCoords(self, coordsFound):
        """
        PURPOSE
        """
        pubCoords = coords()
        #isRightCam = Bool()
        #isRightCam.data = self.__coordsCamRight

        pubCoords.xCord.data = coordsFound[0]
        pubCoords.yCord.data = coordsFound[1]
        pubCoords.rightCam.data = self.__coordsCamRight
        self.__coordsPub.publish(pubCoords)

    def selectFrameFallen(self, ii, hasFallen):
        """
        PURPOSE:
        """
        if hasFallen:
            self.toggleCameraStatesON(ii)
        else:
            self.toggleCameraStatesOFF(ii)
        self.chooseCameraFallen(hasFallen)

    def chooseCameraFallen(self, hasFallen):
        """
        ASSERTION: the camera which has a fallen resident will be chosen. If
        both of the cameras have a fallen resident, then the algorithm will
        default to also choose the one which has more of the person in its
        current frame of view
        """
        #they're  4 possible permutation hence, we will need to deal with all 
        #four of them
        if self.__rightCamFallen and self.__leftCamFallen:
            #TODO: you will need to implement a function which will pick the better frame betwen the left and the right, for now, I am going to stick to right camera
            rospy.loginfo("BOTH CAMERAS")
            self.publishFallen(hasFallen)
        elif self.__leftCamFallen and not self.__rightCamFallen:
            rospy.loginfo("LEFT CAMERA")
            self.publishFallen(hasFallen)
        elif not self.__leftCamFallen and self.__rightCamFallen:
            rospy.loginfo("RIGHT CAMERA")
            self.publishFallen(hasFallen)
        else:
            rospy.loginfo("No cameras")
            self.publishFallen(hasFallen)

    def toggleCameraStatesON(self, camIndx):
        """
        ASSERTION: Will turn the leftCamFallen class field to true for an index
        of 0, and it will turn the rightCamFallen class field to true for an
        index of 1
        """
        if camIndx == miro.constants.CAM_L:
            self.__leftCamFallen = True
        #otherwise, it will be the right camera
        else:
            self.__rightCamFallen = True

    def toggleCameraStatesOFF(self, camIndx):
        """
        ASSERTION: Will turn the leftCamFallen class field to false for an index
        of 0, and it will turn the rightCamFallen class field to false for an
        index of 1
        """

        if camIndx == miro.constants.CAM_L:
            self.__leftCamFallen = False
        #otherwise, it will be the right camera
        else:
            self.__rightCamFallen = False

    def getNoseCordinates(self, results, imgShape):
        """
        ASSERTION: MiRo will always know the location of the face of the
        the resident
        """
        coords = None
        #by the looks of it, it's going to be just s tuple, and this tuple is 
        #literally going to be the x and y variables of the coordinates which you 
        #want to find and use in the image which you'll have

        landmarkFound = results.pose_landmarks

        if landmarkFound:
            landmark = landmarkFound.landmark
            head = landmark[self.__mpHolistic.PoseLandmark.NOSE.value]
            coords = (head.x * imgShape[1], head.y * imgShape[0])

        return coords

    def showBBox(self, results, img):
        """
        PURPOSE: To show the results for the bounding box algorithm from MiRo's 
        stereo cameras
        """

        inView = self.torsoInView(results)
        color = None
        text = ""
        orientation = None

        if inView:
            text = "Person in view"
            color = (0,255,0)
            orientation, bbox = self.orientationOfTorso(results, img.shape)


            if bbox:
                cv2.rectangle(img, (int(bbox[0]), int(bbox[1])),
                        (int(bbox[2]), int(bbox[3])), (0,255,0),
                        thickness=2, lineType=cv2.LINE_AA)

                if orientation:
                    text = "Fallen Resident"
                    color = (255, 0, 0)
                    cv2.rectangle(img, (int(bbox[0]), int(bbox[1])),
                            (int(bbox[2]), int(bbox[3])), (0,0,255),
                        thickness=2, lineType=cv2.LINE_AA)

        img = cv2.putText(img, text, (100,100), cv2.FONT_HERSHEY_SIMPLEX,1, color,
                2, cv2.LINE_AA)

        return img, orientation

    def showPose(self, results, img):
        """
        PURPOSE: To show the results fr the mediapipe algorithm from MiRo's 
        stereo cameras
        """

        inView = self.torsoInView(results)

        color = None
        fallen = False

        text = ""

        if inView:
            text = "Person is in View"
            color = (0,255,0)
            distance = self.getDistance(results, img.shape)
            fallen = self.hasFallenDistance(distance)

            if fallen:
                text = "Fallen Resident"
                color = (255, 0, 0)

        img = cv2.putText(img, text, (100,100), cv2.FONT_HERSHEY_SIMPLEX,1, color,
                2, cv2.LINE_AA)

        return img, fallen

    def __setMode(self, args):
        """
        PURPOSE: A wrapper to the setter property, so we can use function internally,
        and as a user
        """
        #TODO: Try to re-factor this so you can use the argparser module for it to be cleaner
        retArg = None
        if len(args) == 0:
            self.MiRoError("Please provide arguments into programme:\n" +
                    "\t show: show video (eye cameras) as it arrives from platform\n"+
                    "\t --stitch stitch stereo images into one image")
        else:
            for arg in args:
                #TODO: expand this for more modes of the programme
                if arg in ["show"]:
                    #self.__mode = self.__validateMode(arg)
                    #TODO: You'll need to come back and check the logic of this file
                    retArg = self.__validateMode(arg)

                if arg == "--verbose":
                    self.__verbose = True

                if arg == "--stitch":
                    self.__imageStitcher = True

                if arg == "bounding_box":
                    self.__bbox = True

                if arg == "pose":
                    self.__pose = True


        #self.__validateMode(self.__mode)

        return  retArg

    def MiRoError(self, mssg):
        rospy.logerr(mssg)
        sys.exit(0)

    def __validateMode(self, inMode):
        """
        PURPOSE: TO ensure that the current mode which has being set for the
        programme is not going to be none
        """

        if inMode == None:
            MiRoError("Stereo Camera mode is not set")

        return inMode

#--------------METHODS USED FOR TESTING THIS CLASS -----------------------------
    def testMiRoVariables(self, inPathRight, inPathLeft):
        """
        IMPORT:
        EXPORT:

        PURPOSE:
        """

        #the tuning variables on the values I am running the tests on

        trackingConfidence = 0.4

        #the default value for the tracking confidence
        detectionConfidence = 0.5

        #getting all the jpeg files
        jpegFilesLeft = glob.glob(inPathLeft + "*.jpg")
        jpegFilesRight = glob.glob(inPathRight + "*.jpg")




        steps = [ii * 0.1 for ii in range (2, 12, 2)]
        #limiting the values to one decimal place to correctly match up folders
        steps = ["%.1f" % ii for ii in steps]
        for value in steps:
            lSavePath = "/home/parallels/Desktop/Thesis/data/video/pose_detection/mediapipe_adjustments/left_cam/detection_confidence/" + value + "/"
            rSavePath = "/home/parallels/Desktop/Thesis/data/video/pose_detection/mediapipe_adjustments/right_cam/detection_confidence/" + value + "/"

            value = float(value)

            #default detection confidence
            #self.__dectConf = 0.5
            #self.__trackConf = value
            self.__dectConf = value
            self.__trackConf = 0.5
            print(value)
            #processing all the left frames
            self.processImagesPose(jpegFilesLeft, lSavePath)

            #processing all the right frames
            self.processImagesPose(jpegFilesRight, rSavePath)



        #processing all the right frames

    def processImagesPose(self, fileList, savePath):
        """
        IMPORT:
        EXPORT:

        PURPOSE:
        """

        ii = 0
        for imgPath in fileList:

            currImg = cv2.imread(imgPath)
            imgName = imgPath.split("/")[-1]

            #I am just curious to see the forms which results will take up,
            #and what it wll be 
            results, img = self.findPose(currImg)
            #just annotating the image
            img, hasFallen = self.showPose(results, img)

            #determining if landmarks were found in the image or not 

            landMarkFound = results.pose_landmarks

            if landMarkFound:
                #saving the images to the successful folder of the algorithm
                cv2.imwrite(savePath + imgName, img)
                self.writeFile(savePath + imgName[:-4] +".txt", "")
            else:
                cv2.imwrite(savePath + "/fail/" + imgName, img)
                self.writeFile(savePath + "/fail/" + imgName[:-4] +".txt", "")

    def testResolution(self):
        """
        IMPORT:
        EXPORT:

        PURPOSE:
        """

        #setting the tracking and detection confidence to defaults as 
        #recommended by the mediapipe documentation
        lcBase = "/home/parallels/Desktop/Thesis/data/video/pose_detection/resolution_adjustments/left_cam/"


        self.__dectConf = 0.5
        self.__trackConf = 0.5

        resolutions = ["1280x720", "640x360", "320x180"]

        for ii in resolutions:
            print(ii)
            sampledFiles = glob.glob(lcBase + ii + "/sampled/" + "*.jpg")
            savePath = lcBase + ii + "/"
            self.processImagesPose(sampledFiles, savePath)

    def createPerformanceData(self, inPathRight, inPathLeft):
        """
        IMPORT:
        EXPORT:

        PURPOSE:
        """
        loopCounter = 0
        elapsedTime = 0

        #performance results file name
        fileNameBase = "/home/parallels/Desktop/Thesis/data/video/performance/"
        fileNameRaw = fileNameBase + "Just fall.csv"

        headers = ["loop step", "time (secs)"]

        self.createFile(fileNameRaw, headers)
        #getting all the image files
        jpegFilesLeft = glob.glob(inPathLeft + "*.jpg")
        jpegFilesRight = glob.glob(inPathRight + "*.jpg")
        startTime = time.time()

        for leftPath, rightPath in zip(jpegFilesLeft, jpegFilesRight):

            leftImg = cv2.imread(leftPath)
            rightImg = cv2.imread(rightPath)

            #getting the results from the current frame
            resultsLeft, imgLeft = self.findPose(leftImg)
            resultsRight, imgRight = self.findPose(rightImg)



            #the blurring algorithm here
            #_, imgLeft = self.blurFace(imgLeft)
            #_, imgRight = self.blurFace(imgRight)


            #pose algorithm
            imgleft, hasFallenLeft = self.showPose(resultsLeft, imgLeft)
            imgRight, hasFallenRight = self.showPose(resultsRight, imgRight)


            self.selectFrameFallen(0, True)
            self.selectFrameFallen(1, False)


            imgLeft = cv2.cvtColor(imgLeft, cv2.COLOR_BGR2RGB)
            imgRight = cv2.cvtColor(imgRight, cv2.COLOR_BGR2RGB)


            #changing the color
            cv2.imshow("Current Left", imgLeft)
            cv2.imshow("Current Right", imgRight)

            elapsedTime = time.time() - startTime
            self.writeInfo(fileNameRaw, headers, loopCounter, elapsedTime)
            loopCounter += 1


            time.sleep(0.02)

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

    def testResolutionPerformance(self):
        """
        IMPORT:
        EXPORT:

        PURPOSE:
        """

        loopCounter = 0
        elapsedTime = 0

        size = "320x180"
        #performance results save location 
        fileNameBase = "/home/parallels/Desktop/Thesis/data/video/performance/resolution/"

        fileName = fileNameBase + size + ".csv"

        headers = ["loop step", "time (secs)"]

        self.createFile(fileName, headers)

        fileNameSaved = "/home/parallels/Desktop/Thesis/data/video/pose_detection/resolution_adjustments/left_cam/" + str(size) + "/sampled/"


        #getting the images file
        jpegFiles = glob.glob(fileNameSaved + "*.jpg")

        startTime = time.time()
        for imgPath in jpegFiles:
            leftImg = cv2.imread(imgPath)
            rightImg = cv2.imread(imgPath)
            #getting the results from the current frame
            resultsLeft, imgLeft = self.findPose(leftImg)
            resultsRight, imgRight = self.findPose(rightImg)

            #the blurring algorithm here
            _, imgLeft = self.blurFace(imgLeft)
            _, imgRight = self.blurFace(imgRight)

            #pose algorithm
            imgleft, hasFallenLeft = self.showPose(resultsLeft, imgLeft)
            imgRight, hasFallenRight = self.showPose(resultsRight, imgRight)

            self.selectFrameFallen(0, True)
            self.selectFrameFallen(1, False)
            #changing the color
            imgLeft = cv2.cvtColor(imgLeft, cv2.COLOR_BGR2RGB)
            imgRight = cv2.cvtColor(imgRight, cv2.COLOR_BGR2RGB)

            cv2.imshow("Current Left", imgLeft)
            cv2.imshow("Current Right", imgRight)

            elapsedTime = time.time() - startTime
            self.writeInfo(fileName, headers, loopCounter, elapsedTime)
            loopCounter += 1

            time.sleep(0.02)





if __name__ == "__main__":
    mode = ""
    algo = ""
    mode = rospy.get_param("viewMode")
    algo = rospy.get_param("typeAlgo")
    print("Started pose_fallen package ... ")

    if len(mode) != 0:
        print("Using launch file...")
        rospy.loginfo("Using launch file ...")
        main = PoseFallen([mode, algo])
        main.getVideoFeed()
    else:
        print("Not using a launch file ... ")
        rospy.loginfo("Not using launch file ...")
        main = PoseFallen(sys.argv[1:])
        main.getVideoFeed()
    #starting the  video feed and analysing if person has fallen in frames


    #just testing if the testing functions will work for our cases 
    """
    main = PoseFallen(["show", "pose"])
    #main.testMiRoVariables(SAVE_PATH_L, SAVE_PATH_R)
    #main.createPerformanceData(SAVE_PATH_L, SAVE_PATH_R)
    #main.testResolution()
    main.testResolutionPerformance()
    RobotInterface.disconnect
    """
