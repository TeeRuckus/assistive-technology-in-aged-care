from pose_fallen import PoseFallen
import cv2
import glob

PATH = "test_data/SetElderly/"

def main():
    testImageBbox()
    #testVideoBbox()
    #testImageDist()
    #testVideo()


def testImageBbox():
    poseFallen = PoseFallen()
    testDataPath = "test_data/SetElderly/"

    img = cv2.imread(testDataPath + "img_001.png")
    results, img = poseFallen.findPose(img)
    fallen, bbox = poseFallen.orientationOfTorso(results, img.shape)

    print("Shape: ", img.shape)

    print("has fallen: ", fallen)

    cv2.rectangle(img, (int(bbox[0]), int(bbox[1])),
            (int(bbox[2]), int(bbox[3])), (0,0,255),
            thickness=2, lineType=cv2.LINE_AA)


    cv2.imshow ("fallen resident", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def testVideoBbox():
    keepOpen = True
    cap = cv2.VideoCapture(0)
    poseFallen = PoseFallen()
    while cap.isOpened and keepOpen:
        _,frame = cap.read()
        results, img = poseFallen.findPose(frame)

        orientation, bbox = poseFallen.orientationOfTorso(results, img.shape)
        if bbox:
            cv2.rectangle(img, (int(bbox[0]), int(bbox[1])),
                    (int(bbox[2]), int(bbox[3])), (0,0,255),
                    thickness=2, lineType=cv2.LINE_AA)

        cv2.imshow("Testing life vide feed", img)

        if cv2.waitKey(10) & 0xFF == ord('q'):
            keepOpen = False

    cap.release()
    cv2.destroyAllWindows()


def testImageDist():
    poseFallen = PoseFallen()
    testDataPath = "test_data/SetElderly/"
    #reading in an image from the test data 
    image = cv2.imread(testDataPath + "img_058.png") #image 2 and 58 are going to be good for testing this
    image = cv2.imread(testDataPath + "img_002.png")

    results, image = poseFallen.findPose(image)
    inView = poseFallen.noseHipInView(results)
    distance = poseFallen.getDistance(results, image.shape)
    fallen = poseFallen.hasFallenDistance(distance)
    print("Euclidean distance ", distance)

    if inView:
        print("person's torso is in view")
    else:
        print("person torso is NOT in view")

    print("has fallen ", fallen)

    cv2.imshow("fallen resident", image)


    #waiting for a key to be pressed before closing down
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def testVideoDistance():
    keepOpen = True
    cap = cv2.VideoCapture(0)
    poseFallen = PoseFallen()
    while cap.isOpened and keepOpen:
        _,frame = cap.read()
        results, img = poseFallen.findPose(frame)

        """
        #seeing if it has found any land marks
        if results.pose_landmarks:
            print(results.pose_landmarks.landmark[0])
        """

        inView = poseFallen.torsoInView(results)

        color = None
        text = ""

        if inView:
            text = "Person is in view"
            color = (0,255,0)
            distance = poseFallen.getDistance(results, img.shape)
            fallen = poseFallen.hasFallenDistance(distance)

            if fallen:
                text = "Fallen Resident"
                color = (0, 0, 255)

        img = cv2.putText(img, text, (100,100), cv2.FONT_HERSHEY_SIMPLEX,1, color,
                2, cv2.LINE_AA)

        cv2.imshow("Testing life vide feed", img)

        if cv2.waitKey(10) & 0xFF == ord('q'):
            keepOpen = False

    cap.release()
    cv2.destroyAllWindows()

def testImageData(generate=False):
    """
    PURPOSE:This function will test the fallen function given the setElderly
    data, and will write the results in relation to the following metrics.

    - True Positives (TP) - Number of falls correctly detected
    - False Negatives (FN) - Number of falls not detected
    - False positives (FP) - Number of non detected as falls
    - Precision (Pr) - Provides information about the proportion of positive fall 
    identifications that are actually falls. This is given by the following equation
    Pr  = TP / (TP + FP)
    - Recall (Re) - The proportion of falls that were identified correctly. 
    This metric is given by the following equation: 
        Re = TP / (TP + FN)
    """

    pngFiles = glob.glob(PATH + "*.png")
    txtFiles = glob.glob(PATH + "*.txt")
    distanceResults = None

    #sorting data so png files line up with their respective txt files
    pngFiles.sort()
    txtFiles.sort()
    hasFallen = fallenImage(txtFiles)
    #creating a dictionary of pictures and if they have fallen or not
    actualResults = {pngFiles[ii] : hasFallen[ii] for ii in range(len(pngFiles))}

    if generate:
        distanceResults = applyDistanceFallDetection()
    else:
        distanceResults = readToDictonary("results/distance_fall_detection/data.csv")

    writeToFile("results/distance_fall_detection/actualResults.csv", actualResults)

    calcPerformance(actualResults,distanceResults,
            "results/distance_fall_detection/results.csv")




def calcPerformance(actualDict, resultsDict, savePath):
    """
    """
    performance = {}
    truePostives = 0
    falseNegatives = 0
    falsePositives = 0
    trueNegatives = 0
    performance["Total Data points"] = len(actualDict)

    #Splitting the data to detection, and none detection
    noresultsDict = dict(filter(lambda ii: ii[1].strip() == "None",
        resultsDict.items()))
    resultsDict = dict(filter(lambda ii: ii[1].strip()!= "None",
        resultsDict.items()))

    ii = 0


    #only doing the test on the data which was able to detect a person
    for key in resultsDict.keys():
        ii += 1
        #if the resident has fallen
        if actualDict[key]:
            #if a fall has being correctly classified, count it as true positive
            if resultsDict[key].strip() == "True":
                truePostives += 1
            #falls which have not being detected, count it as a false negative
            else:
                falseNegatives += 1
        #if the resident hasn't fallen
        else:
            #resident has fallen, although algorithm detects it as a fall, count
            #as a false positive
            if resultsDict[key].strip() == "True":
                falsePositives += 1
            else:
                trueNegatives += 1

    precision = truePostives / (truePostives + falsePositives)
    recall = truePostives / (truePostives + falseNegatives)

    #re-cording how the algorithm performed with the test data provided
    performance["No Detection"] = len(noresultsDict)
    performance["True Positives (TP) "] = truePostives
    performance["False Negatives (FN)"] = falseNegatives
    performance["False positives (FP)"] = falsePositives
    performance["True Negatives (TN)"] = trueNegatives
    performance["Precision (Pr)"] = precision
    performance["Recall (Re)"] = recall

    writeToFile(savePath, performance)



def applyDistanceFallDetection():
    """
    PURPOSE: To test the fall detection method using the distance of the head to 
    the heap joint of the fallen resident, and getting the results it produces
    """
    poseFallen = PoseFallen()
    #grabbing all images from the test directory
    imgPaths = glob.glob(PATH + "*.png")
    obtainedResults = {}

    for currImgPath in imgPaths:
        print(currImgPath)
        fallen = None
        img = cv2.imread(currImgPath)
        results,_ = poseFallen.findPose(img)

        #if they has being results returned, then we want to find the calculated 
        #distance from the data
        if results.pose_landmarks:
            distance = poseFallen.getDistance(results, img.shape)
            fallen = poseFallen.hasFallenDistance(distance)

        obtainedResults[currImgPath] = str(fallen)

    writeToFile("results/distance_fall_detection/data.csv", obtainedResults)

    return obtainedResults


def applyBBoxFallDetection():
    """
    PURPOSE: TO test the fall detection method using the bounding box method, and
    trying to extract performance results from fall method.
    """
    poseFallen = PoseFallen()
    #grabbing all images from the test directory
    #grabbing all images from the test directory
    imgPaths = glob.glob(PATH + "*.png")
    obtainedResults = {}

    for currImgPath in imgPath:
        print(currImgPath)

def writeToFile(path, data):
    """
    PURPOSE: to write data which has being stored in a dictionary to a file
    """
    with open(path, "w") as outStrm:
        for key in data.keys():
            outStrm.write("%s,%s\n"%(key,data[key]))

def readToDictonary(path):
    """
    PURPOSE: To read data from a CSV file which is formatted in the following 
    manner <key for dictionary>, <value for dictionary entry>
    """

    keys = []
    values = []
    data = {}

    with open(path, "r") as inStrm:
        for data in inStrm:
            key, value = data.split(",")
            keys.append(key.strip())
            values.append(value.strip())

    data = {keys[ii] : values[ii] for ii in range(len(keys))}

    return data

def fallenImage(filePath):
    """
    PURPOSE: Opens up paths in given list to determine if image has being
    labelled as a fall or a non-fall. A non-fall file will have no data written
    to it, and a fall will have data written in it.
    """
    fallenList = []
    for currPath in filePath:
        fallen = False
        with open(currPath, "r") as inStrm:
            lines = inStrm.readlines()
            if lines:
                fallen = True
        fallenList.append(fallen)

    return fallenList


if __name__ == "__main__":
    main()
    """
    test = {}

    test["Name"] = "Tawana Kwaramba"
    test["DOB"] = "15/05/2000"
    test["Job"] = "Academic Seasonal"
    test["Deadlift"] = "200kg"
    test["car"] = None

    print("Hello mate")
    test = dict(filter(lambda ii: ii[1] != None, test.items()))


    for key, value in test.items():
        print("%s %s" %(key, value))
    """
