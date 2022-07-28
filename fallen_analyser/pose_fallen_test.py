from pose_fallen import PoseFallen
import cv2

keepOpen = True
cap = cv2.VideoCapture(0)
testDataPath = "test_data/SetElderly/"

poseFallen = PoseFallen()

"""
#reading in an image from the test data 
image = cv2.imread(testDataPath + "img_002.png")

results, image = poseFallen.findPose(image)
inView = poseFallen.torsoInView(results)

if inView:
    print("person's torso is in view")
else:
    print("person torso is NOT in view")

cv2.imshow("fallen resident", image)


#waiting for a key to be pressed before closing down
cv2.waitKey(0)
cv2.destroyAllWindows()
"""

while cap.isOpened and keepOpen:
    _,frame = cap.read()
    results, img = poseFallen.findPose(frame)

    inView = poseFallen.torsoInView(results)

    color = None
    if inView:
        text = "Person is in view"
        color = (0,255,0)
    else:
        text = "Person is not in view"
        color = (0,0,255)

    img = cv2.putText(img, text, (100,100), cv2.FONT_HERSHEY_SIMPLEX,1, color,
            2, cv2.LINE_AA)

    cv2.imshow("Testing life vide feed", img)

    if cv2.waitKey(10) & 0xFF == ord('q'):
        keepOpen = False

cap.release()
cv2.destroyAllWindows()
