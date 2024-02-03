import cv2

def detect_aruco(image):
    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_100)
    aruco_param = cv2.aruco.DetectorParameters_create()
    corners, ids, rejected = cv2.aruco.detectMarkers(image, aruco_dict, parameters=aruco_param)
    detect_markers = aruco_display(corners, ids, rejected, image)
    return detect_markers


def aruco_display(corners, ids, rejected, image):
    if len(corners) > 0:
        ids = ids.flatten()
        for (markerCorner, markerID) in zip(corners, ids):
            corners = markerCorner.reshape((4, 2))
            (topLeft, topRight, bottomRight, bottomLeft) = corners
            topRight = (int(topRight[0]), int(topRight[1]))
            bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
            bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
            topLeft = (int(topLeft[0]), int(topLeft[1]))

            cv2.line(image, topLeft, topRight, (0, 255, 0), 1)
            cv2.line(image, topRight, bottomRight, (0, 255, 0), 1)
            cv2.line(image, bottomRight, bottomLeft, (0, 255, 0), 1)
            cv2.line(image, bottomLeft, topLeft, (0, 255, 0), 1)

            # cX = int((topLeft[0] + bottomRight[0]) / 2.0)
            # cY = int((topLeft[1] + bottomRight[1]) / 2.0)

            # cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)

            cv2.putText(image, str(markerID), (topLeft[0], topLeft[1]+15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

    return image
            

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
    
    aruco_frame = detect_aruco(frame)
    cv2.imshow("Aruco Frame", aruco_frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

webcam.release()
cv2.destroyAllWindows()