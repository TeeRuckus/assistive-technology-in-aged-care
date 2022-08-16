"""
FILE NAME: Video.py
AUTHOR: Tawana Kwaramba (19476700)
LAST MODIFIED: N.D

PURPOSE: This is file is to contain all the functionality for getting a video
feed from MiRo's stereo cameras.
"""
#TODO: you will need to comment on where some of this code has came from
import rospy
import cv2
import time
import sys
import os
import numpy as np

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage
from Errors import *

NODE_NAME = "Miro_Video"

#TODO: you will need to add in the mode which you will be running the programme 
#with
#TODO: you will need to create the accessors and the mutators  for each class
#field which you will have

class MiRoVideo():

    def __init__(self, args):
        self.__inputCamera = [None] * 3
        self.__imageStitcher = None
        self.__mode = self.__setMode(args)
        #a converter from the ROS application to OpenCV
        self.__imageConverter = CvBridge()
        self.__pubVideoL = rospy.Publisher("/videoL/image/compressed",
                CompressedImage, queue_size=10)
        self.__pubVideoR = rospy.Publisher("/videoR/image/compressed",
                CompressedImage, queue_size=10)
        #self.__pubVideoL = rospy.Publisher("videoL", String, queue_size=1)
        #self.__pubVideoR = rospy.Publisher("videoR", String, queue_size=1)

        #Creating the node to support this class currently
        topicBaseName = "/" + os.getenv("MIRO_ROBOT_NAME")

        #the nodes which this class will need to subscribe too

        rospy.init_node(NODE_NAME, anonymous=True)
        self.__subCamL = rospy.Subscriber(topicBaseName + "/sensors/caml/compressed",
                                CompressedImage, self.callbackCamL, queue_size=1, tcp_nodelay=True)

        self.__subCamR = rospy.Subscriber(topicBaseName + "/sensors/camr/compressed",
                                CompressedImage, self.callbackCamR, queue_size=1, tcp_nodelay=True)



    @property
    def pubVideoL(self):
        """
        PURPOSE: A getter for the pubVideoL class field
        """
        return  self.__pubVideoL

    @property
    def pubVideoR(self):
        """
        PURPOSE: A getter for the pubVideoL class field
        """
        return  self.__pubVideoR

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

    @property
    def imageStitcher(self):
        """
        PURPOSE: A getter for the imageStitcher class field
        """
        return self.__imageStitcher

    @pubVideoL.setter
    def pubVideoL(self, args):
        """
        PURPOSE: A setter for the class field pubVideoL
        """
        #TODO: you'll have to figure out how to throw errors properly in ros

    @pubVideoR.setter
    def pubVideoR(self,args):
        """
        PURPOSE: A setter for the class field pubVideoR
        """
        #TODO: you'll have to figure out how to throw errors properly in ros


    @mode.setter
    def mode(self, args):
        #going through the arguments which might have being set in the programme

        self.__setMode(args)



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
        self.callbackCam(rosImg, 0)


    def callbackCamR(self, rosImg):
        """
        PURPOSE: To obtain images from the right stereo camera of MiRo
        """
        self.callbackCam(rosImg, 1)

    def getVideoFeed(self):
        """
        PURPOSE: to get a live video feed from MiRo's stereo cameras, and output
        the video to the screen. Additionally, this should also publish the 
        frames which MiRo is receiving from the video feed
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
            #for the left image
            """
            if self.__inputCamera[0] is not None:
                #creating the compressed image to be published to the network
                msg = CompressedImage()
                msg.header.stamp = rospy.Time.now()
                msg.format = "jpeg"
                #I AM HERE, AND DOING TUTORIAL

                npArr = np.fromstring(self.__inputCamera[0], np.uint8)
                print("="*80)
                print(npArr)
                print("="*80)
                imgNP = cv2.imdecode(npArr, cv2.IMREAD_COLOR)
                print("="*80)
                print(imgNP)
                print(npArr)
                print("="*80)
                msg.data = np.array(cv2.imencode(".jpg", imgNP)[1]).tostring()
                #msg.data = self.__inputCamera[0]
                #self.__pubVideoL.publish("left_video", "rgb8",
                        #self.__inputCamera[0])

                rospy.loginfo("PUblishing left video message")
                rospy.loginfo(msg)

                self.__pubVideoL.publish(msg)

            #for the right image
            if self.__inputCamera[1] is not None:
                #creating the compressed image to be published to the network
                self.__pubVideoR.publish("right_video", "rgb8", 
                        self.__inputCamera[1])
            """

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


                    #print("mode, ", self.__mode)
                    if self.__mode == "show":

                        #correcting the color of the image
                        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                        cv2.imshow("Video from MiRo: " + camNames[ii], img)
                        cv2.waitKey(1)

                    #add more modes here for the program

                    #publishing the found node

            #processing frames at 50Hz
            time.sleep(0.02)

        for ii in range(len(outFile)):
            pass


    #PRIVATE METHODS

    def __validateMode(self, inMode):
        """
        PURPOSE: TO ensure that the current mode which has being set for the
        programme is not going to be none
        """

        if inMode == None:
            MiRoError("Stereo Camera mode is not set")

        return inMode

    def __setMode(self, args):
        """
        PURPOSE: A wrapper to the setter property, so we can use function internally,
        and as a user
        """
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
                    retArg = self.__validateMode(arg)
                if arg == "--stitch":
                    self.__imageStitcher = True

        #self.__validateMode(self.__mode)

        return  retArg






#testing if the streaming of the video will work on MiRo 

if __name__ == "__main__":
    #hard coding the arguments for now, to see test if the connections will work properly
    #this should be publishing a compressed video now
    main = MiRoVideo(sys.argv[1:])
    main.getVideoFeed()

