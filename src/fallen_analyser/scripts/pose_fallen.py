import mediapipe as mp
import numpy as np
import cv2
import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, Float32MultiArray
from cv_bridge import CvBridge, CvBridgeError
import sys
import os
from Errors import *
import time
from fallen_analyser.msg import coords
import miro2 as miro


VISIBILITY_THRESHOLD = 0.4
#FALEN_DISTANCE_THRESHOLD = 240
#FALEN_DISTANCE_THRESHOLD = 300
#FALEN_DISTANCE_THRESHOLD = 350
#FALEN_DISTANCE_THRESHOLD = 290
FALEN_DISTANCE_THRESHOLD = 250
NODE_NAME = "pose_fallen"

#TODO: You'll need to test if the conversation with constants have worked here

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


        #subscribing to the required nodes and the data
        if self.__verbose:
            rospy.loginfo("Creating main node %s" % NODE_NAME)

        rospy.init_node(NODE_NAME, anonymous=True)
        topicBaseName = "/" + os.getenv("MIRO_ROBOT_NAME")


        #TODO: You'll need to come back and delete this CODE
        camLTopic = topicBaseName + "/sensors/caml/compressed"
        camRTopic = topicBaseName + "/sensors/camr/compressed"
        #TODO: you'll have to come back and see if this is going to be necessary

        if self.__verbose:
            rospy.loginfo("subscribing to left and right camera ... ")

        self.__camL = rospy.Subscriber(camLTopic, CompressedImage,
                self.callbackCamL, queue_size=1, tcp_nodelay=True)

        self.__camR = rospy.Subscriber(camRTopic, CompressedImage,
                self.callbackCamR, queue_size=1, tcp_nodelay=True)

        if self.__verbose:
            rospy.loginfo("Creating Publisher, to publish coordinates")

        self.__coordsPub = rospy.Publisher("resident/coords/",
                coords, queue_size=10)

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


    def publishFallen(self):
        """
        PURPOSE: To publish whether a resident has fallen or not in the current
        fames
        """

        #TODO: COME BACK TO THIS, and see how you're going to publish this
        #message
        hasFallen = False
        pub = rospy.Publisher(NODE_NAME, Bool,queue_size=10)


        pub.publish(hasFallen)



    def hasFallenOrientation(self, orientation):
        """
        """


    def callbackCam(self, rosImg, indx):
        """
        PURPOSE: To grab image data from a camera specified by index
        """
        try:
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


        #main loop for getting data from the stereo cameras of MiRo
        while not rospy.core.is_shutdown():
            #if we're required to stitch the images
            if not self.__imageStitcher is None:
                #performing stitching process of the given images 

                #if both of the cameras are going to have images in them
                if not self.__inputCamera[0] is None and not self.__inputCamera[1] is None:
                    imgs = [self.__inputCamera[0], self.__inputCamera[1]]
                    self.__inputCamera[2] = cv2.hconcat(imgs)
                    self.__inputCamera[0] = None
                    self.__inputCamera[1] = None

            #only publish the images, if they is going to be an image to 
            #publish

            for ii in channelsToProcess:
                #getting the current image
                img = self.__inputCamera[ii]

                #TODO: make this line more readable
                #if the image is present
                if not img is None:
                    #clearing the current image, as we're about to process
                    #the current image
                    self.__inputCamera[ii] = None

                    #getting results from the current frame
                    results, img = self.findPose(img)

                    #TODO: you'll need to double check with Tele if this is going to be acceptable
                    resultsPub, imgPub = self.findPose(
                            self.__inputCamera[miro.constants.CAM_R])

                    #only going to publish coordinates from the left eye
                    coordsFound = self.getNoseCordinates(resultsPub,
                        imgPub.shape)

                    #gaurding against publish none messages
                    if coordsFound:
                        pubCoords = coords()
                        pubCoords.xCord.data = coordsFound[0]
                        pubCoords.yCord.data = coordsFound[1]
                        self.__coordsPub.publish(pubCoords)

                    if coordsFound and self.__verbose:
                        rospy.loginfo("Coordinates: %s %s" % (coordsFound[0],
                            coordsFound[1]))

                    #print("mode, ", self.__mode)
                    if self.__mode == "show":

                        if self.__bbox:
                            if  self.__verbose:
                                rospy.loginfo_once("Showing Bounding box mode")
                            img = self.showBBox(results, img)
                        elif self.__pose:
                            if self.__verbose:
                                rospy.loginfo_once("Showing pose from media pipe")
                            img = self.showPose(results, img)

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

        return img


    def showPose(self, results, img):
        """
        PURPOSE: To show the results fr the mediapipe algorithm from MiRo's 
        stereo cameras
        """

        inView = self.torsoInView(results)

        color = None

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

        return img

    def __setMode(self, args):
        """
        PURPOSE: A wrapper to the setter property, so we can use function internally,
        and as a user
        """
        #TODO: Try to re-factor this so you can use the argparser module for it to be cleaner
        retArg = None
        if len(args) == 0:
            MiRoError("Please provide arguments into programme:\n" +
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

    def __validateMode(self, inMode):
        """
        PURPOSE: TO ensure that the current mode which has being set for the
        programme is not going to be none
        """

        if inMode == None:
            MiRoError("Stereo Camera mode is not set")

        return inMode


#the code of the main loop which is needed for this node
if __name__ == "__main__": 
    rospy.loginfo("Started pose fallen node")

    #starting the  video feed and analysing if person has fallen in frames
    main = PoseFallen(sys.argv[1:])
    main.getVideoFeed()



