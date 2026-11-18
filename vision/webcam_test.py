import sys
import cv2
from ultralytics import YOLO
MODEL_PATH = "yolo26n-pose.pt" # Ultralytics pose model, nano size( smallest/fastest)
FRAME_WIDTH = 640 #same as robot camera
FRAME_HEIGHT = 480
CAMERA_INDEX = 0
model = YOLO(MODEL_PATH)
print("model loaded")
cap = cv2.VideoCapture(CAMERA_INDEX)
if not cap.isOpened(): #in case it doesnt detect any camera
    print("can't open camera")
    sys.exit(1)

#for testing with a different camera but with the camera's width and height
cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

while True:
    ok,frame = cap.read()
    if not ok: #if a frame didnt arrive
        print("can't read camera")
        break
    if frame.shape != (FRAME_HEIGHT, FRAME_WIDTH, 3):  # for testing if the camera brings a different shape
        print(f"Wrong frame shape: {frame.shape}")
    r = model(frame,verbose=False)[0]
    print(len(r.boxes))
    cv2.imshow("webcam_test", r.plot())
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break



cap.release()
cv2.destroyAllWindows()







