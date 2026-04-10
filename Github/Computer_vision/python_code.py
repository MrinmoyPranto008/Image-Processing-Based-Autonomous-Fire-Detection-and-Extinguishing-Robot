import cv2
import numpy as np
import socket
from ultralytics import YOLO
import math
import time
import threading
import urllib.request
CAM_URL  = "http://10.18.127.157:8080/video"
ESP32_IP = "10.18.127.58"
UDP_PORT = 4210
MARKER_SIZE_CM  = 6.0    

PIXELS_PER_CM   = 5.25   
CM_PER_SLOT     = 1.0
DEG_PER_SLOT    = 1.5
MAX_BURST_SLOTS    = 40     
COMMAND_COOLDOWN_S = 0.25   

ARRIVE_DIST_CM     = 10.0   
ALIGN_THRESH_DEG   = 12.0   
CLOSE_RANGE_CM     = 20.0  


# INITIALISATION & THREADING 
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
last_send_time = 0.0

print("Loading YOLO model...")
try:
    model = YOLO(r"E:\Class\control project\CODE\asholei code\best.pt")
except Exception as e:
    print(f"ERROR: Could not load model — {e}")
    exit()

class ThreadedCamera:
    def __init__(self, src):
        self.src = src.replace("/video", "/shot.jpg")
        self.ret = False
        self.frame = None
        self.stopped = False

    def start(self):
        threading.Thread(target=self.update, args=(), daemon=True).start()
        return self

    def update(self):
        while not self.stopped:
            try:
               
                imgResp = urllib.request.urlopen(self.src, timeout=4.0)
                imgNp = np.array(bytearray(imgResp.read()), dtype=np.uint8)
                frame = cv2.imdecode(imgNp, -1)
                if frame is not None:
                    self.ret, self.frame = True, frame
               
                time.sleep(0.03) 
                
            except Exception as e:
                
                print(f"CAMERA LAG: Waiting for phone to catch up...")
                time.sleep(0.5)

    def read(self):
        return self.ret, self.frame

    def stop(self):
        self.stopped = True
print("Connecting to camera stream...")
cap = ThreadedCamera(CAM_URL).start()
time.sleep(2.0)

aruco_dict   = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
aruco_params = cv2.aruco.DetectorParameters()
aruco_params.adaptiveThreshWinSizeMin    = 3
aruco_params.adaptiveThreshWinSizeMax    = 53
aruco_params.adaptiveThreshWinSizeStep   = 10
aruco_params.minMarkerPerimeterRate      = 0.02
aruco_params.polygonalApproxAccuracyRate = 0.05
aruco_params.minCornerDistanceRate       = 0.02
aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

# --- HSV Fallback bounds for fire ---
lower_fire_hsv = np.array([0,  120, 150])
upper_fire_hsv = np.array([25, 255, 255])

# --- CLAHE Setup ---
clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))

def apply_clahe(bgr_frame):
    lab = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)
    l_clahe = clahe.apply(l)
    lab_clahe = cv2.merge((l_clahe, a, b))
    return cv2.cvtColor(lab_clahe, cv2.COLOR_LAB2BGR)

last_known_fire = None
fire_lost_frames = 0
MAX_LOST_FRAMES  = 15

def send_command(cmd: str, slots: int):
    packet = f"{cmd}:{slots}"
    try:
        sock.sendto(packet.encode(), (ESP32_IP, UDP_PORT))
        print(f"UDP Sent: {packet}")
    except OSError as e:
        print(f"WARNING: UDP send failed — {e}")

def find_largest_centroid(mask, min_area=80):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours: return None
    lg = max(contours, key=cv2.contourArea)
    if cv2.contourArea(lg) < min_area: return None
    M = cv2.moments(lg)
    if M['m00'] == 0: return None
    return (int(M['m10'] / M['m00']), int(M['m01'] / M['m00']))

# MAIN LOOP
print("Starting Fire Bot Command Centre... Press 'q' to quit.")
print(f"NOTE: Set MARKER_SIZE_CM to your printed marker size. Currently: {MARKER_SIZE_CM} cm")

current_ppc = PIXELS_PER_CM

while True:
    ret, raw_frame = cap.read()
    if not ret: continue

    raw_frame    = cv2.resize(raw_frame, (640, 480))
    clahe_frame  = apply_clahe(raw_frame)

    #  Locate Bot (ArUco) 
    front_center = None
    back_center  = None

    gray_frame = cv2.cvtColor(raw_frame, cv2.COLOR_BGR2GRAY)
    corners, ids, rejected = aruco_detector.detectMarkers(gray_frame)

    if ids is not None:
        c = corners[0][0]  # 4 corner points of marker

        marker_px_width = math.hypot(c[1][0] - c[0][0], c[1][1] - c[0][1])
        if marker_px_width > 15:  
            PIXELS_PER_CM = marker_px_width / MARKER_SIZE_CM
            current_ppc   = PIXELS_PER_CM

        fx, fy = int((c[0][0] + c[1][0]) / 2), int((c[0][1] + c[1][1]) / 2)
        bx, by = int((c[3][0] + c[2][0]) / 2), int((c[3][1] + c[2][1]) / 2)
        front_center = (fx, fy)
        back_center  = (bx, by)
        cv2.aruco.drawDetectedMarkers(clahe_frame, corners, ids)

    if rejected:
        cv2.aruco.drawDetectedMarkers(clahe_frame, rejected, borderColor=(0, 0, 255))

    current_fire_detected = None
    detection_src = "NONE"

    results = model(clahe_frame, stream=True, conf=0.25, verbose=False)
    for r in results:
        for box in r.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            # Track the BASE of the bounding box (y2 = bottom edge = floor level).
            # Fire flickers vertically, so the center y bounces constantly.
            # The base stays fixed at the candle/fuel source — stable every frame.
            current_fire_detected = ((x1 + x2) // 2, y2)
            cv2.rectangle(clahe_frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
            detection_src = "YOLO"
            break
        break

    if not current_fire_detected:
        blurred = cv2.GaussianBlur(clahe_frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        fire_mask = cv2.inRange(hsv, lower_fire_hsv, upper_fire_hsv)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        fire_mask = cv2.morphologyEx(fire_mask, cv2.MORPH_CLOSE, kernel)
        fire_mask = cv2.morphologyEx(fire_mask, cv2.MORPH_OPEN,  kernel)
        hsv_center = find_largest_centroid(fire_mask, min_area=80)
        if hsv_center:
            current_fire_detected = hsv_center
            cv2.circle(clahe_frame, current_fire_detected, 12, (0, 165, 255), 2)
            detection_src = "HSV"
    if current_fire_detected:
        fire_center     = current_fire_detected
        last_known_fire = current_fire_detected
        fire_lost_frames = 0
    elif last_known_fire and fire_lost_frames < MAX_LOST_FRAMES:
        fire_center  = last_known_fire
        fire_lost_frames += 1
        detection_src = f"MEM ({MAX_LOST_FRAMES - fire_lost_frames})"
        cv2.circle(clahe_frame, fire_center, 8, (128, 128, 128), -1)
    else:
        fire_center     = None
        last_known_fire = None


    if front_center: cv2.circle(clahe_frame, front_center, 8, (0, 255, 0),  -1)
    if back_center:  cv2.circle(clahe_frame, back_center,  8, (255, 0, 0),  -1)
    if fire_center and detection_src in ["YOLO", "HSV"]:
        cv2.circle(clahe_frame, fire_center, 4, (0, 0, 255), -1)
    cv2.putText(clahe_frame, f"px/cm: {current_ppc:.2f}", (460, 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 0), 1)

    if front_center and back_center and fire_center:
        bot_center = (
            (front_center[0] + back_center[0]) // 2,
            (front_center[1] + back_center[1]) // 2
        )
        dist_cm = math.hypot(
            fire_center[0] - bot_center[0],
            fire_center[1] - bot_center[1]
        ) / PIXELS_PER_CM

        bot_heading  = math.degrees(math.atan2(-(front_center[1] - back_center[1]),
                                                 front_center[0] - back_center[0]))
        target_angle = math.degrees(math.atan2(-(fire_center[1] - bot_center[1]),
                                                  fire_center[0] - bot_center[0]))
        angle_error  = (target_angle - bot_heading + 180) % 360 - 180

        cv2.line(clahe_frame, back_center,  front_center, (0, 255, 0),   2)
        cv2.line(clahe_frame, bot_center,   fire_center,  (0, 255, 255), 2)
        cv2.putText(clahe_frame,
                    f"Dist:{dist_cm:.1f}cm  Err:{angle_error:.1f}deg  {detection_src}",
                    (10, 420), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        if time.time() - last_send_time > COMMAND_COOLDOWN_S:

            if dist_cm < ARRIVE_DIST_CM:
                # ---- STOP & ENGAGE ----
                send_command('S', 0)
                cv2.putText(clahe_frame, "ENGAGING FIRE!", (50, 50),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3)

            elif abs(angle_error) > ALIGN_THRESH_DEG:
                
                if dist_cm < CLOSE_RANGE_CM:
                    turn_cap = 3
                else:
                    turn_cap = MAX_BURST_SLOTS

                cmd        = 'L' if angle_error > 0 else 'R'
                req_slots  = int(abs(angle_error) / DEG_PER_SLOT)
                final_slots = max(1, min(req_slots, turn_cap))
                send_command(cmd, final_slots)
                cv2.putText(clahe_frame, f"ALIGNING: {cmd} {final_slots}", (50, 50),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

            else:
                
                remaining_cm = max(0.0, dist_cm - ARRIVE_DIST_CM)

                fwd_cap = 4 if dist_cm < CLOSE_RANGE_CM else MAX_BURST_SLOTS

                req_slots   = int(remaining_cm / CM_PER_SLOT)
                final_slots = max(1, min(req_slots, fwd_cap))
                send_command('F', final_slots)
                cv2.putText(clahe_frame,
                            f"ADVANCING: F {final_slots} ({remaining_cm:.1f}cm left)",
                            (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            last_send_time = time.time()

    else:
        lost = []
        if not front_center: lost.append("ROBOT")
        if not fire_center:  lost.append("FIRE")
        cv2.putText(clahe_frame, f"SEARCHING: {', '.join(lost)}", (10, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)

    cv2.imshow("Fire Bot — Command Centre", clahe_frame)
    if cv2.waitKey(1) == ord('q'): break

# ================= 4. CLEANUP =================
cap.stop()
sock.close()
cv2.destroyAllWindows()
print("Shutdown complete.")