import datetime
import cv2
from ultralytics import YOLO

CONFIDENCE_THRESHOLD = 0.6
GREEN = (0, 255, 0)
WHITE = (255, 255, 255)

coco128 = open('./cocoABC.txt', 'r')
data = coco128.read()
class_list = data.split('\n')
coco128.close()

model = YOLO('./best.pt')

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
global pub_data


while True:
    start = datetime.datetime.now()

    ret, frame = cap.read()
    if not ret:
        print('Cam Error')
        break

    detection = model(frame)[0]

    for data in detection.boxes.data.tolist(): # data : [xmin, ymin, xmax, ymax, confidence_score, class_id]
        confidence = float(data[4])
        if confidence < CONFIDENCE_THRESHOLD:
            continue

        xmin, ymin, xmax, ymax = int(data[0]), int(data[1]), int(data[2]), int(data[3])
        label = int(data[5])
        if label == 21 or label == 10: # V K
            cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), GREEN, 2)
            cv2.putText(frame, 'Feedback'+' '+str(int(round(confidence, 2)*100)) + '%', (xmin, ymin), cv2.FONT_ITALIC, 1, WHITE, 2)
            pub_data = "Feedback"
        
        elif label == 11: # L
            cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), GREEN, 2)
            cv2.putText(frame, 'Left'+' '+str(int(round(confidence, 2)*100)) + '%', (xmin, ymin), cv2.FONT_ITALIC, 1, WHITE, 2)
            pub_data = "Left"

        elif label == 24: # Y
            cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), GREEN, 2)
            cv2.putText(frame, 'Right'+' '+str(int(round(confidence, 2)*100)) + '%', (xmin, ymin), cv2.FONT_ITALIC, 1, WHITE, 2)
            pub_data = "Right"

        elif label == 22: # W
            cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), GREEN, 2)
            cv2.putText(frame, 'Go'+' '+str(int(round(confidence, 2)*100)) + '%', (xmin, ymin), cv2.FONT_ITALIC, 1, WHITE, 2)
            pub_data = "Go"

        elif label == 1 or label == 20: # B U
            cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), GREEN, 2)
            cv2.putText(frame, 'Stop'+' '+str(int(round(confidence, 2)*100)) + '%', (xmin, ymin), cv2.FONT_ITALIC, 1, WHITE, 2)
            pub_data = "Stop"

    end = datetime.datetime.now()

    total = (end - start).total_seconds()
    print(f'Time to process 1 frame: {total * 1000:.0f} milliseconds')

    fps = f'FPS: {1 / total:.2f}'
    cv2.putText(frame, fps, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

    cv2.imshow('frame', frame)

    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()