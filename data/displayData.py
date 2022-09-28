import pandas as pd
import matplotlib.pyplot as plt
import os
import glob
import sys

plt.style.use("ggplot")

def getCsvFiles(dirName):
    """
    IMPORT:
    EXPORT:

    PURPOSE:
    """
    files = os.listdir(dirName)
    print(files[1].split(".")[-1])
    #making sure to only include directories
    csvFIles = [x for x in files if x.split(".")[-1] == "csv"]
    print(csvFIles)

    return csvFIles


def displayPID():
    """
    IMPORT:
    EXPORT:

    PURPOSE:
    """
    dirName = "sonar_pid_control/I_tuning/"
    files = getCsvFiles(dirName)

    x = []
    y = []
    legendList = []

    for ii in files:
        data = pd.read_csv(dirName + ii)
        x = data["Time (secs)"]
        y = data["Control variable"]
        #labels for the proportional values
        #plt.plot(x,y, label=ii.split(",")[0][4:])
        #labels for the derivative of PID controller
        #plt.plot(x,y, label="P=-3, " + ii.split(",")[2][:-4])
        #labels for all parameters
        plt.plot(x,y, label="P=-3, " + ii[4:-4])
    plt.legend(loc="upper right")
    plt.xlabel("Time (Secs)")
    plt.ylabel("PID Control Variable (%)")
    plt.title("PID Control Variable vs. Time")
    plt.savefig("sonar_pid_control/Integral Tuning ones firs five")
    plt.show()

def displaySonar():
    """
    IMPORT:
    EXPORT:

    PURPOSE:
    """
    dirName = "sonar_filtering/Driving results/SMA/"
    files = getCsvFiles(dirName)

    x = []
    y = []
    legendList = []

    for ii in files:
        data = pd.read_csv(dirName + ii)
        x = data["Time (secs)"]
        y = data["Sonar Distance (m)"]
        plt.plot(x,y, label=ii[:-4])
    plt.legend(loc="upper left")
    plt.xlabel("Time (Secs)")
    plt.ylabel("Sonar Distance (m)")
    plt.title("Simple Moving Average Tests")
    plt.savefig("sonar_filtering/Simple Moving Average Results")
    plt.show()

def displayPurePerformance():
    """
    IMPORT:
    EXPORT:

    PURPOSE:
    """

    dirName = "video/performance/"

    files = glob.glob(dirName + "*.csv")


    for ii in files:
        data = pd.read_csv(ii)
        x = data["loop step"]
        y = data["time (secs)"]
        labelName = ii.split("/")[-1][:-4]
        plt.plot(x, y, label=labelName)

    plt.legend(loc="upper left")
    plt.xlabel("Algorithm Step")
    plt.ylabel("Time (Secs)")
    plt.title("Algorithm Performance Comparison")
    plt.savefig("video/Algorithm Performance Comparison")
    plt.show()

def displayResolutionPerformance():
    """
    IMPORT:
    EXPORT:

    PURPOSE:
    """

    dirName = "video/performance/resolution/"

    files = glob.glob(dirName + "*.csv")

    for ii in files:
        data = pd.read_csv(ii)
        x = data["loop step"]
        y = data["time (secs)"]
        labelName = ii.split("/")[-1][:-4]
        plt.plot(x, y, label=labelName)

    plt.legend(loc="upper left")
    plt.xlabel("Algorithm Step")
    plt.ylabel("Time (Secs)")
    plt.title("Resolution Size Performance Comparison")
    plt.savefig("video/Resolution Size Performance Comparison")
    plt.show()


def getMediaPipeStatsDectConf():
    """
    IMPORT:
    EXPORT:

    PURPOSE:
    """

    steps = [ii * 0.1 for ii in range (2, 12, 2)]
    #limiting the values to one decimal place to correctly match up folders
    steps = ["%.1f" % ii for ii in steps]

    for value in steps:
        performance = {}
        #file counters
        truePositives  = 0
        falsePositives  = 0

        dirNameDetected = "video/pose_detection/mediapipe_adjustments/left_cam/detection_confidence/" + value +"/"
        dirNameFailed = "video/pose_detection/mediapipe_adjustments/left_cam/detection_confidence/" + value + "/fail/"


        filesDetected = glob.glob(dirNameDetected + "*.txt")
        filesFailed = glob.glob(dirNameFailed + "*.txt")


        noDetection = len(filesFailed)

        for ii in filesDetected:

            with open(ii, "r") as inStrm:

                #they is going to be only one line in every single file
                result = inStrm.readline()
                result = int(result.strip())


                if result == 1:
                    truePositives += 1
                elif result == 2:
                    falsePositives += 1
                else:
                    print("You have labelled something wrong bro")
                    sys.exit()
        totalData = truePositives + falsePositives + noDetection
        try:
            precision = truePositives / (truePositives + falsePositives)
        except ZeroDivisionError as err:
            precision = "undefined"

        percentCorrect = truePositives / totalData


        #loading all data to dictionary so it can be written to a file 
        performance["Total Data"] = totalData
        performance["Precision"]  = precision
        performance["True Positives"] = truePositives
        performance["False Positives"] = falsePositives
        performance["No Detection"] = noDetection
        performance["Reliability"] = percentCorrect

        #save file

        saveName = "video/pose_detection/mediapipe_adjustments/left_cam/detection_confidence/" + value +".csv"
        writeDict(saveName, performance)

def getMediaPipeStatsTrackConf():
    """
    IMPORT:
    EXPORT:

    PURPOSE:
    """

    steps = [ii * 0.1 for ii in range (2, 12, 2)]
    #limiting the values to one decimal place to correctly match up folders
    steps = ["%.1f" % ii for ii in steps]

    for value in steps:
        performance = {}
        #file counters
        truePositives  = 0
        falsePositives  = 0

        dirNameDetected = "video/pose_detection/mediapipe_adjustments/left_cam/tracking_confidence/" + value +"/"
        dirNameFailed = "video/pose_detection/mediapipe_adjustments/left_cam/tracking_confidence/" + value + "/fail/"


        filesDetected = glob.glob(dirNameDetected + "*.txt")
        filesFailed = glob.glob(dirNameFailed + "*.txt")


        noDetection = len(filesFailed)

        for ii in filesDetected:

            with open(ii, "r") as inStrm:

                #they is going to be only one line in every single file
                result = inStrm.readline()
                result = int(result.strip())


                if result == 1:
                    truePositives += 1
                elif result == 2:
                    falsePositives += 1
                else:
                    print("You have labelled something wrong bro")
                    sys.exit()
        totalData = truePositives + falsePositives + noDetection
        try:
            precision = truePositives / (truePositives + falsePositives)
        except ZeroDivisionError as err:
            precision = "undefined"

        percentCorrect = truePositives / totalData


        #loading all data to dictionary so it can be written to a file 
        performance["Total Data"] = totalData
        performance["Precision"]  = precision
        performance["True Positives"] = truePositives
        performance["False Positives"] = falsePositives
        performance["No Detection"] = noDetection
        performance["Reliability"] = percentCorrect

        #save file

        saveName = "video/pose_detection/mediapipe_adjustments/left_cam/tracking_confidence/" + value +".csv"
        writeDict(saveName, performance)

def getResolutionStats():
    """
    IMPORT:
    EXPORT:

    PURPOSE:
    """

    resolutions = ["1280x720", "640x360", "320x180"]

    for ii in resolutions:

        performance = {}
        #file counters
        truePositives  = 0
        falsePositives  = 0
        dirNameDetected = "video/pose_detection/resolution_adjustments/left_cam/" + ii +"/"
        dirNameFailed = "video/pose_detection/resolution_adjustments/left_cam/" + ii + "/fail/"


        filesDetected = glob.glob(dirNameDetected + "*.txt")
        filesFailed = glob.glob(dirNameFailed + "*.txt")

        noDetection = len(filesFailed)

        for jj in filesDetected:
            with open(jj, "r") as inStrm:
                #they is going to be only one line in every single file
                result = inStrm.readline()
                result = int(result.strip())


                if result == 1:
                    truePositives += 1
                elif result == 2:
                    falsePositives += 1
                else:
                    print("You have labelled something wrong bro")
                    sys.exit()
        totalData = truePositives + falsePositives + noDetection
        try:
            precision = truePositives / (truePositives + falsePositives)
        except ZeroDivisionError as err:
            precision = "undefined"

        percentCorrect = truePositives / totalData
        #loading all data to dictionary so it can be written to a file 
        performance["Total Data"] = totalData
        performance["Precision"]  = precision
        performance["True Positives"] = truePositives
        performance["False Positives"] = falsePositives
        performance["No Detection"] = noDetection
        performance["Reliability"] = percentCorrect

        saveName = "video/pose_detection/resolution_adjustments/left_cam/" + ii +".csv"
        writeDict(saveName, performance)


def writeDict(path, data):
    """
    IMPORT:
    EXPORT:

    PURPOSE: To write data which has being stored in a dictionary to a file
    """
    with open(path, "w") as outStrm:
        for key in data.keys():
            outStrm.write("%s,%s\n"%(key,data[key]))

def createTrackingConfidencePlots():
    """
    IMPORT:
    EXPORT:

    PURPOSE:
    """

    savePath = "video/pose_detection/mediapipe_adjustments/left_cam/tracking_confidence/"
    filePath = "video/pose_detection/mediapipe_adjustments/left_cam/tracking_confidence/combined.csv"

    data = pd.read_csv(filePath)
    x = data['Tracking Confidence']
    precision  = data["Precision"]
    truePositives = data["True Positives"]
    falsePositives = data["False Positives"]
    noDetection  = data["No Detection"]
    reliability = data["Reliability"]

    plt.legend(loc="upper left")
    plt.plot(x, precision)
    plt.xlabel("Tracking Confidence")
    plt.ylabel("Precision (%)")
    plt.title("Impacts of Tracking Confidence on Precision")
    plt.savefig(savePath + 'precision')
    plt.clf()

    plt.plot(x, truePositives)
    plt.ylabel("True Positives")
    plt.xlabel("Tracking Confidence")
    plt.title("Impacts of Tracking Confidence on True Positives")
    plt.savefig(savePath + "true positives")
    plt.clf()

    plt.plot(x, falsePositives)
    plt.ylabel("False Positives")
    plt.xlabel("Tracking Confidence")
    plt.title("Impacts of Tracking Confidence on False Positives")
    plt.savefig(savePath + "false positives")
    plt.clf()

    plt.plot(x, noDetection)
    plt.ylabel("No Detection")
    plt.xlabel("Tracking Confidence")
    plt.title("Impacts of Tracking Confidence on Detection Ability")
    plt.savefig(savePath + "no detection")
    plt.clf()


    plt.plot(x, reliability)
    plt.ylabel("Reliability (%)")
    plt.title("Impacts of Tracking Confidence on Reliability")
    plt.savefig(savePath + "reliability")

def createDetectionConfidencePlots():
    """
    IMPORT:
    EXPORT:

    PURPOSE:
    """

    savePath = "video/pose_detection/mediapipe_adjustments/left_cam/detection_confidence/"
    filePath = "video/pose_detection/mediapipe_adjustments/left_cam/detection_confidence/combined.csv"

    data = pd.read_csv(filePath)
    x = data['Detection Confidence']
    precision  = data["Precision"]
    truePositives = data["True Positives"]
    falsePositives = data["False Positives"]
    noDetection  = data["No Detection"]
    reliability = data["Reliability"]

    plt.legend(loc="upper left")
    plt.plot(x, precision)
    plt.xlabel("Detection Confidence")
    plt.ylabel("Precision (%)")
    plt.title("Impacts of Detection Confidence on Precision")
    plt.savefig(savePath + 'precision')
    plt.clf()

    plt.plot(x, truePositives)
    plt.ylabel("True Positives")
    plt.xlabel("Detection Confidence")
    plt.title("Impacts of Detection Confidence on True Positives")
    plt.savefig(savePath + "true positives")
    plt.clf()

    plt.plot(x, falsePositives)
    plt.ylabel("False Positives")
    plt.xlabel("Detection Confidence")
    plt.title("Impacts of Detection Confidence on False Positives")
    plt.savefig(savePath + "false positives")
    plt.clf()

    plt.plot(x, noDetection)
    plt.ylabel("No Detection")
    plt.xlabel("Detection Confidence")
    plt.title("Impacts of Detection Confidence on Detection Ability")
    plt.savefig(savePath + "no detection")
    plt.clf()


    plt.plot(x, reliability)
    plt.ylabel("Reliability (%)")
    plt.xlabel("Tracking Confidence")
    plt.title("Impacts of Detection Confidence on Reliability")
    plt.savefig(savePath + "reliability")


def createResolutionCharts():
    """
    IMPORT:
    EXPORT:

    PURPOSE:
    """

    savePath = "video/pose_detection/resolution_adjustments/left_cam/"
    filePath = "video/pose_detection/resolution_adjustments/left_cam/combined.csv"

    data = pd.read_csv(filePath)
    x = data["Resolution"]
    precision  = data["Precision"]
    truePositives = data["True Positives"]
    falsePositives = data["False Positives"]
    noDetection  = data["No Detection"]
    reliability = data["Reliability"]

    plt.bar(x, precision)
    plt.xlabel("Resolution")
    plt.ylabel("Precision (%)")
    plt.title("Impacts of Resolution on Precision")
    plt.savefig(savePath + 'precision')
    plt.clf()

    plt.bar(x, truePositives)
    plt.ylabel("True Positives")
    plt.xlabel("Resolution")
    plt.title("Impacts of Resolution on True Positives")
    plt.savefig(savePath + "true positives")
    plt.clf()

    plt.bar(x, falsePositives)
    plt.ylabel("False Positives")
    plt.xlabel("Resolution")
    plt.title("Impacts of Resolution on False Positives")
    plt.savefig(savePath + "false positives")
    plt.clf()

    plt.bar(x, noDetection)
    plt.ylabel("No Detection")
    plt.xlabel("Resolution")
    plt.title("Impacts of Resolution on Detection Ability")
    plt.savefig(savePath + "no detection")
    plt.clf()


    plt.bar(x, reliability)
    plt.ylabel("Reliability (%)")
    plt.xlabel("Resolution")
    plt.title("Impacts of Resolution on Reliability")
    plt.savefig(savePath + "reliability")





#TODO: change the files
if __name__ == "__main__":
    #displayPID()
    #displaySonar()
    #displayPurePerformance()
    #displayResolutionPerformance()
    #getMediaPipeStatsDectConf()
    #getMediaPipeStatsTrackConf()
    #getResolutionStats()
    createTrackingConfidencePlots()
    createDetectionConfidencePlots()
    createResolutionCharts()
