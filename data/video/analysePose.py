import glob
import cv2
import sys

LEFT_CAM_BASE = "/home/parallels/Desktop/Thesis/data/video/pose_detection/resolution_adjustments/left_cam/"

RIGHT_CAM_BASE = "/home/parallels/Desktop/Thesis/data/video/pose_detection/mediapipe_adjustments/right_cam/"

LEFT_RAW_IMAGES = "/home/parallels/Desktop/Thesis/data/video/raw_frames/left_cam/"

LEFT_CAM_BASE_RES = "/home/parallels/Desktop/Thesis/data/video/pose_detection/resolution_adjustments/left_cam/"

def main():
    """
    IMPORT:
    EXPORT:

    PURPOSE:
    """
    #to re-sample the images to different sizes
    #reSampleImages(640, 360)
    #reSampleImages(320, 180)
    #preLabelDataRes()

def preLabelData():
    """
    IMPORT:
    EXPORT:

    PURPOSE:
    """
    ltcBase = RIGHT_CAM_BASE + "tracking_confidence/"
    dtcBase = RIGHT_CAM_BASE + "detection_confidence/"


    #trying to access all the directories at once
    steps = [ii * 0.1 for ii in range(2,12, 2)]
    steps = ["%.1f" % ii for ii in steps]

    for value in steps:
        ltc = ltcBase + value + "/"
        ltcFail = ltcBase + value + "/fail/"


        ltc = glob.glob(ltc + "*.txt")
        ltcFail = glob.glob(ltcFail + "*.txt")

        writeFile(ltc, "1")
        writeFile(ltcFail, "0")

    for value in steps:
        dtc = dtcBase + value + "/"
        dtcFail = dtcBase + value + "/fail/"

        dtc = glob.glob(dtc + "*.txt")
        dtcFail = glob.glob(dtcBase + "*.txt")


        writeFile(dtc, "1")
        writeFile(dtcFail, "0")

def preLabelDataRes():
    """
    IMPORT:
    EXPORT:

    PURPOSE:
    """

    resolutions = ["1280x720", "640x360", "320x180"]

    for ii in resolutions:
        #grabbing the current files in the directory
        print(ii)
        currFiles = glob.glob(LEFT_CAM_BASE_RES + ii + "/" + "*.txt")
        currFilesFail = glob.glob(LEFT_CAM_BASE + ii + "/fail/" + "*.txt")


        writeFile(currFiles, "1")
        writeFile(currFilesFail, "0")


def reSampleImages(width, height):
    """
    IMPORT:
    EXPORT:

    PURPOSE:
    """


    folderName = str(width) + "x" + str(height) + "/sampled/"
    rawFiles = glob.glob(LEFT_RAW_IMAGES + "*.jpg")
    dim = (width, height)


    ii = 0
    for imgPath in rawFiles:
        img = cv2.imread(imgPath)
        imgName = imgPath.split("/")[-1]
        resized = cv2.resize(img, dim, interpolation = cv2.INTER_AREA)
        cv2.imwrite(LEFT_CAM_BASE + folderName + imgName, resized)

def writeFile(fileNames, data):
    """
    IMPORT:
    EXPORT:

    PURPOSE:
    """

    for name in fileNames:
        with open(name, "w") as outStrm:
            outStrm.write(data)

if __name__ == "__main__":
    main()
