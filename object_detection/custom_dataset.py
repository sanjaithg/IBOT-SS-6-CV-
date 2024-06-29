from ultralytics import YOLO
import cv2
import cvzone
import math
 
cap = cv2.VideoCapture("/home/saber-ubuntu/Documents/iBot/cv_class/custom_dataset_trained/video1.mp4")  #loading videos
 
model = YOLO("/home/saber-ubuntu/Documents/iBot/cv_class/custom_dataset_trained/box3.pt") # loading the trained model based on the dataset included in the folder
 
classNames = ['box']

myColor = (0, 0, 255)

while True:
    success, img = cap.read()
    results = model(img, stream=True)
    for r in results:
        boxes = r.boxes
        for box in boxes:
            x1, y1, x2, y2 = box.xyxy[0]
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
            
            w, h = x2 - x1, y2 - y1
            
            # accuracy
            conf = math.ceil((box.conf[0] * 100)) / 100

            # class
            cls = int(box.cls[0])
            Class = classNames[cls]
            print(Class)
            if conf>0.5: 
                cvzone.putTextRect(img, f'{classNames[cls]} {conf}',
                                   (max(0, x1), max(35, y1-10)), scale=1, thickness=1,colorB=myColor,
                                   colorT=(255,255,255),colorR=myColor, offset=5)
                cv2.rectangle(img, (x1, y1), (x2, y2), (0,0,255), 3)
 
    cv2.imshow("Image", img)
    cv2.waitKey(10)
