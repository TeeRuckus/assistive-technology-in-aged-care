from pose_fallen import PoseFallen
import cv2

def main():
    #testImage()
    testVideo()


def testImage():
    poseFallen = PoseFallen()
    testDataPath = "test_data/SetElderly/"
    #reading in an image from the test data 
    image = cv2.imread(testDataPath + "img_058.png") #image 2 and 58 are going to be good for testing this
    image = cv2.imread(testDataPath + "img_002.png")

    results, image = poseFallen.findPose(image)
    inView = poseFallen.noseHipInView(results)
    distance = poseFallen.getDistanceHeadKnee(results, image.shape)
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

def testVideo():
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
            distance = poseFallen.getDistanceHeadKnee(results, img.shape)
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

def segmentImages():
    """
    to iterate over the setElderly data, make folders for fallen, and not fallen
    """

if __name__ == "__main__":
    main()
