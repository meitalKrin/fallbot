import sys
import cv2
from ultralytics import YOLO
MODEL_PATH = "yolo26n-pose.pt" # Ultralytics pose model, nano size( smallest/fastest)
FRAME_WIDTH, FRAME_HEIGHT = 640, 480 #same as robot camera
CAMERA_INDEX = 0
KP_CONF_MIN = 0.5 # min confidence for a keypoint to be valid
#from COCO keypoint order
L_SHOULDER, R_SHOULDER = 5,6
L_HIP, R_HIP = 11,12 
L_ANKLE, R_ANKLE = 15,16
L_KNEE, R_KNEE = 13,14

def pair_visible(kps_conf, a, b):
    """True if both keypoints a and b are above KP_CONF_MIN"""
    return kps_conf[a] >= KP_CONF_MIN and kps_conf[b] >= KP_CONF_MIN

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
    result = model(frame,verbose=False)[0]

    frame_out = result.plot()
    if result.keypoints is not None and result.keypoints.conf is not None: #if person detected
        kp_conf = result.keypoints.conf.tolist()
        boxes = result.boxes.xyxy.tolist()
        for i in range(len(kp_conf)):
            good_kps = sum(c >= KP_CONF_MIN for c in kp_conf[i])
            x1, y1, x2 , y2 = boxes[i]
            ratio = (x2-x1) / (y2-y1) # width / height of the box
            shoulders = pair_visible(kp_conf[i], L_SHOULDER, R_SHOULDER)
            hips = pair_visible(kp_conf[i], L_HIP, R_HIP)
            knee = pair_visible(kp_conf[i], L_KNEE, R_KNEE)
            ankle = pair_visible(kp_conf[i], L_ANKLE, R_ANKLE)

            line1 = f"kps {good_kps}/17 ratio {ratio:.2f} top {int(y1)}"
            line2 = f"sh {shoulders:d} | hip {hips:d} | ankle {ankle:d} | knee {knee:d}"
            cv2.putText(frame_out,line1, (int(x1)+10, int(y1)+40),
                        cv2.FONT_HERSHEY_SIMPLEX,0.6, (0,255,0), 2)
            cv2.putText(frame_out,line2, (int(x1), int(y2) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,0.6, (0,255,0), 2)




    cv2.imshow("webcam_test", frame_out)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break



cap.release()
cv2.destroyAllWindows()







