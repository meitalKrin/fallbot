import sys
import cv2
import math
from ultralytics import YOLO
MODEL_PATH = "yolo26n-pose.pt" # Ultralytics pose model, nano size( smallest/fastest)
FRAME_WIDTH, FRAME_HEIGHT = 640, 480 #same as robot camera
CAMERA_INDEX = 0
KP_CONF_MIN = 0.3 # min confidence for a keypoint to be valid
#from COCO keypoint order
L_SHOULDER, R_SHOULDER = 5,6
L_HIP, R_HIP = 11,12 
L_ANKLE, R_ANKLE = 15,16
L_KNEE, R_KNEE = 13,14
LYING_ANGLE = 45
LYING_RATIO = 1.3

def pair_visible(kps_conf, a, b):
    """True if both keypoints a and b are above KP_CONF_MIN"""
    return kps_conf[a] >= KP_CONF_MIN and kps_conf[b] >= KP_CONF_MIN

def segment_angle(kps_xy, a1, a2, b1, b2):
    """Returns angle  from vertical (deg) of the line between midpoint(a1,a2)
       and midpoint(b1,b2). 0=vertical (standing), 90 = horizontal (lying)"""
    # middle point between the first pair
    ax = (kps_xy[a1][0] + kps_xy[a2][0])/2
    ay = (kps_xy[a1][1] + kps_xy[a2][1])/2

    # middle point between the second pair
    bx = (kps_xy[b1][0] + kps_xy[b2][0])/2
    by = (kps_xy[b1][1] + kps_xy[b2][1])/2

    #the difference between both middle points in abs
    dx = abs(bx - ax)
    dy = abs(by - ay)

    #Angle in degrees relative to the vertical (y-axis)
    return math.degrees(math.atan2(dx,dy))



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
        kps_xy = result.keypoints.xy.tolist()
        boxes = result.boxes.xyxy.tolist()
        for i in range(len(kp_conf)):
            good_kps = sum(c >= KP_CONF_MIN for c in kp_conf[i])
            x1, y1, x2 , y2 = boxes[i]
            ratio = (x2-x1) / (y2-y1) # width / height of the box

            shoulders = pair_visible(kp_conf[i], L_SHOULDER, R_SHOULDER)
            hips = pair_visible(kp_conf[i], L_HIP, R_HIP)
            knee = pair_visible(kp_conf[i], L_KNEE, R_KNEE)
            ankle = pair_visible(kp_conf[i], L_ANKLE, R_ANKLE)

            if shoulders and hips:
                angle = segment_angle(kps_xy[i], L_SHOULDER, R_SHOULDER, L_HIP, R_HIP)
                seg = "torso"
            elif hips and knee:
                angle = segment_angle(kps_xy[i], L_HIP, R_HIP, L_KNEE, R_KNEE)
                seg = "thigh"
            elif knee and ankle:
                angle = segment_angle(kps_xy[i], L_KNEE, R_KNEE, L_HIP, R_HIP)
                seg = "shin"
            else:
                angle = None
                seg = "-"
            if angle is not None:
                posture = "lying" if angle > LYING_ANGLE else "standing"
            else:
                posture = "lying" if ratio > LYING_RATIO else "standing"


            angle_txt = f"{angle:.0f}" if angle is not None else "-"
            line1 = f"kps {good_kps}/17 ratio {ratio:.2f} angle {angle_txt} ({seg}) | {posture}"
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







