import mediapipe as mp
import numpy as np
import cv2

VISIBILITY_THRESHOLD = 0.4

class PoseFallen():

    def __init__(self, dectConf=0.5, trackConf=0.5):
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

        #landmark = results.pose_landmarks.landmark

        landmark = results.pose_landmarks

        if landmark:
            landmark = results.pose_landmarks.landmark
            leftShoulder = landmark[self.mpHolistic.PoseLandmark.LEFT_SHOULDER.value].visibility
            rightShoulder = landmark[self.mpHolistic.PoseLandmark.RIGHT_SHOULDER.value].visibility
            leftHip = landmark[self.mpHolistic.PoseLandmark.LEFT_HIP.value].visibility
            rightHip = landmark[self.mpHolistic.PoseLandmark.RIGHT_HIP.value].visibility


            if (leftShoulder >= VISIBILITY_THRESHOLD) & (rightShoulder >= VISIBILITY_THRESHOLD) \
                & (leftHip > VISIBILITY_THRESHOLD) & (rightHip > VISIBILITY_THRESHOLD):
                    inView = True

        return inView


