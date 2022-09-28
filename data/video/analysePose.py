import glob

LEFT_CAM_BASE = "/home/parallels/Desktop/Thesis/data/video/pose_detection/mediapipe_adjustments/left_cam/"

RIGHT_CAM_BASE = "/home/parallels/Desktop/Thesis/data/video/pose_detection/mediapipe_adjustments/right_cam/"

def main():
    """
    IMPORT:
    EXPORT:

    PURPOSE:
    """

    #preLabelData()



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
