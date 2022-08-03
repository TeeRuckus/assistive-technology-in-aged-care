from pose_fallen import PoseFallen
import cv2
import glob

PATH = "test_data/SetElderly/"

def main():
    #testImageData(distance=True, bbox=False)
    #getFalseNegativesBbox()
    #getFalseNegativesDistance()
    #testImageBbox()
    testVideoBbox()
    #testImageDist()
    #testVideoDistance()


def testImageBbox():
    poseFallen = PoseFallen()
    testDataPath = "test_data/SetElderly/"

    img = cv2.imread(testDataPath + "img_060.png")
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

        inView = poseFallen.torsoInView(results)
        color = None
        text = ""

        if inView:
            text = "Person in view"
            color = (0,255,0)
            orientation, bbox = poseFallen.orientationOfTorso(results, img.shape)

            if bbox:
                cv2.rectangle(img, (int(bbox[0]), int(bbox[1])),
                        (int(bbox[2]), int(bbox[3])), (0,255,0),
                        thickness=2, lineType=cv2.LINE_AA)

                if orientation:
                    text = "Fallen Resident"
                    color = (0, 0, 255)
                    cv2.rectangle(img, (int(bbox[0]), int(bbox[1])),
                            (int(bbox[2]), int(bbox[3])), (0,0,255),
                        thickness=2, lineType=cv2.LINE_AA)


        img = cv2.putText(img, text, (100,100), cv2.FONT_HERSHEY_SIMPLEX,1, color,
                2, cv2.LINE_AA)

        cv2.imshow("Video Feed", img)

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


        inView = poseFallen.torsoInView(results)

        color = None
        text = ""

        if inView:
            text = "Person is in View"
            color = (0,255,0)
            distance = poseFallen.getDistance(results, img.shape)
            fallen = poseFallen.hasFallenDistance(distance)

            if fallen:
                text = "Fallen Resident"
                color = (0, 0, 255)

        img = cv2.putText(img, text, (100,100), cv2.FONT_HERSHEY_SIMPLEX,1, color,
                2, cv2.LINE_AA)

        cv2.imshow("Video Feed", img)

        if cv2.waitKey(10) & 0xFF == ord('q'):
            keepOpen = False

    cap.release()
    cv2.destroyAllWindows()

def testImageData(**kwargs):
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

    if kwargs["distance"]:
        print("applying distance fall detection on all images ...")
        distanceResults = applyDistanceFallDetection()
    else:
        print("reading data from written file distance")
        distanceResults = readToDictonary("results/distance_fall_detection/data.csv")



    if kwargs["bbox"]:
        print("applying bounding box detection on all images ...")
        bboxResults = applyBBoxFallDetection()
    else:
        print("reading data from witten file bounding box")
        bboxResults = readToDictonary("results/bounding_box_fall_detection/data.csv")

    writeToFile("results/bounding_box_fall_detection/actualResults.csv", actualResults)
    writeToFile("results/distance_fall_detection/actualResults.csv", actualResults)

    calcPerformance(actualResults,distanceResults,
            "results/distance_fall_detection/results.csv",
            "results/distance_fall_detection/falseNegativeList.txt")

    calcPerformance(actualResults, bboxResults,
            "results/bounding_box_fall_detection/results.csv",
            "results/bounding_box_fall_detection/falseNegativesList.txt")



def getFalseNegativesDistance():
    """
    """
    path = "results/distance_fall_detection/falseNegativeList.txt"
    falseNegatives = readToList(path)
    poseFallen = PoseFallen()

    for imgPath in falseNegatives:
        print(imgPath, end=": ")
        img = cv2.imread(imgPath)
        results, img = poseFallen.findPose(img)

        imgPath = imgPath.split('/')
        imgPath = imgPath[-1]
        success = cv2.imwrite("results/distance_fall_detection/False_Negatives/" + imgPath,
                img)

        print(success)


def getFalseNegativesBbox():
    """
    """

    path = "results/bounding_box_fall_detection/falseNegativesList.txt"
    falseNegatives = readToList(path)
    poseFallen = PoseFallen()

    for imgPath in falseNegatives:
        print(imgPath, end= ": " )
        img = cv2.imread(imgPath.strip())
        results, img = poseFallen.findPose(img)
        fallen, bbox = poseFallen.orientationOfTorso(results, img.shape)

        cv2.rectangle(img, (int(bbox[0]), int(bbox[1])),
                (int(bbox[2]), int(bbox[3])), (0,0,255),
                thickness=2, lineType=cv2.LINE_AA)

        imgPath = imgPath.split('/')
        imgPath = imgPath[-1]
        success = cv2.imwrite("results/bounding_box_fall_detection/False_Negatives/" + imgPath,
                img)

        print(success)



def calcPerformance(actualDict, resultsDict, savePath, fnPath):
    """
    """
    performance = {}
    falseNegativeList = []
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
                falseNegativeList.append(key)
        #if the resident hasn't fallen
        else:
            #resident has fallen, although algorithm detects it as a fall, count
            #as a false positive
            if resultsDict[key].strip() == "True":
                falsePositives += 1
            else:
                trueNegatives += 1

    if (truePostives > 0) & (falsePositives > 0):
        precision = truePostives / (truePostives + falsePositives)
    else:
        precision = "UNDEFINED"

    if (truePostives > 0) & (falseNegatives > 0):
        recall = truePostives / (truePostives + falseNegatives)
    else:
        recall = "UNDEFINED"

    #re-cording how the algorithm performed with the test data provided
    performance["No Detection"] = len(noresultsDict)
    performance["True Positives (TP) "] = truePostives
    performance["False Negatives (FN)"] = falseNegatives
    performance["False positives (FP)"] = falsePositives
    performance["True Negatives (TN)"] = trueNegatives
    performance["Precision (Pr)"] = precision
    performance["Recall (Re)"] = recall

    writeToFile(savePath, performance)
    writeToFileList(fnPath, falseNegativeList)



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

    for currImgPath in imgPaths:
        print(currImgPath)
        fallen = None
        img = cv2.imread(currImgPath)
        results,_ = poseFallen.findPose(img)

        #if they has being results found, we want to calculate if the resident
        #has fallen given the orientation of the bounding box around torso
        if results.pose_landmarks:
            fallen = poseFallen.orientationOfTorso(results, img.shape)

        if fallen:
            obtainedResults[currImgPath] = str(fallen[0])

    writeToFile("results/bounding_box_fall_detection/data.csv", obtainedResults)

    return obtainedResults


def writeToFile(path, data):
    """
    PURPOSE: To write data which has being stored in a dictionary to a file
    """
    with open(path, "w") as outStrm:
        for key in data.keys():
            outStrm.write("%s,%s\n"%(key,data[key]))


def writeToFileList(path, inList):
    """
    PURPOSE: To write data which has being stored in a list format to a file
    """

    with open(path, "w") as outStrm:
        for item in inList:
            outStrm.write(item + "\n")


def readToList(path):
    """
    PURPOSE: to read in a file which will be a list of items 
    """

    retItems = []
    with open(path, "r") as inStrm:
        for item in inStrm:
            retItems.append(item.strip())


    return retItems


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
