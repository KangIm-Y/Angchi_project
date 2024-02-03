import cv2

webcam = cv2.VideoCapture(0)

if not webcam.isOpened():
    print("Webcam connect Error")

webcam.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
webcam.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

while webcam.isOpened():
    ref, frame = webcam.read()
    if not ref:
        print("Cam Read Error")
        break

    cv2.imshow("Test Frame", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

webcam.release()
cv2.destroyAllWindows()