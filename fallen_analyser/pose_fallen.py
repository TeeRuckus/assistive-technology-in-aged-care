import mediapipe as mp
import numpy as np
import cv2

VISIBILITY_THRESHOLD = 0.4
#FALEN_DISTANCE_THRESHOLD = 240
#FALEN_DISTANCE_THRESHOLD = 300
#FALEN_DISTANCE_THRESHOLD = 350
#FALEN_DISTANCE_THRESHOLD = 290
FALEN_DISTANCE_THRESHOLD = 250

class PoseFallen():

    def __init__(self, dectConf=0.1, trackConf=0.1):
        self.mpDrawing = mp.solutions.drawing_utils
        self.mpHolistic = mp.solutions.holistic
        self.dectConf = dectConf
        self.trackConf = trackConf


    def findPose(self, img, draw=True):
        """
        PURPOSE: To implement mediapipe on the found image, and trying
        to find the pose on the found image at the current moment
        """

        results = None
        with self.mpHolistic.Holistic(
                min_detection_confidence=self.dectConf,
                min_tracking_confidence=self.trackConf) as holistic:

            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            results = holistic.process(img)

            if results:
                if draw:
                    self.mpDrawing.draw_landmarks(img, 
                            results.pose_landmarks,
                            self.mpHolistic.POSE_CONNECTIONS, 
                            self.mpDrawing.DrawingSpec(color=(66,117,245), thickness=2, circle_radius=2),
                            self.mpDrawing.DrawingSpec(color=(230,66,245), thickness=2, circle_radius=2))


        return results, cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

    def torsoInView(self, results):
        """
        """
        inView = False

        landmarkFound = results.pose_landmarks

        #checking if any landmarks are found in the image
        if landmarkFound:
            landmark = landmarkFound.landmark
            leftShoulder = landmark[self.mpHolistic.PoseLandmark.LEFT_SHOULDER.value].visibility
            rightShoulder = landmark[self.mpHolistic.PoseLandmark.RIGHT_SHOULDER.value].visibility
            leftHip = landmark[self.mpHolistic.PoseLandmark.LEFT_HIP.value].visibility
            rightHip = landmark[self.mpHolistic.PoseLandmark.RIGHT_HIP.value].visibility


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
            nose = landmark[self.mpHolistic.PoseLandmark.NOSE.value].visibility
            hip = landmark[self.mpHolistic.PoseLandmark.RIGHT_HIP.value].visibility

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

            lShoulder = landmark[self.mpHolistic.PoseLandmark.LEFT_SHOULDER.value]
            rShoulder = landmark[self.mpHolistic.PoseLandmark.RIGHT_SHOULDER.value]
            lHip = landmark[self.mpHolistic.PoseLandmark.LEFT_HIP.value]
            rHip = landmark[self.mpHolistic.PoseLandmark.RIGHT_HIP.value]

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

            #getting the euclidean distance of each side of rectangle 
            #we're going to determine orientation by which side is longer in 
            #calculated bounding box. If y-side is longer person hasn't fallen, 
            #if x side is longer then the person has fallen

            #ySide = np.sqrt(pow(lShoulderCords[1],2) + pow(lHipCords[1],2))
            #xSide = np.sqrt(pow(lShoulderCords[0],2) + pow(rShoulderCords[0],2))

            """
            xSides = [lShoulderCords[0], rShoulderCords[0], lHipCords[0],
                    rHipCords[0]]

            ySides = [lShoulderCords[1], rShoulderCords[1], lHipCords[1],
                    rHipCords[1]]
            xSideMax = max(xSides)
            xSideMin = min(xSides)
            ySideMax = max(ySides)
            ySideMin = min(ySides)

            xSide = xSideMax - xSideMin
            ySide = ySideMax = ySideMin
            """



            print("ySide: ", ySide)
            print("xSide ", xSide)



            if xSide > ySide:
                fallen = True

        return fallen , bbox


    def getDistance(self, results, imgShape):
        """
        """

        distance = None
        landmarkFound = results.pose_landmarks

        #checking  if any landmarks are found in the image 
        if landmarkFound:
            landmark = landmarkFound.landmark
            head = landmark[self.mpHolistic.PoseLandmark.NOSE.value]
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


    def hasFallenOrientation(self, orientation):
        """
        """
