import cv2
from ultralytics import YOLO

# ================= CONFIGURATION =================
# Added '/video' which is standard for apps like IP Webcam on port 8080
CAM_URL = "http://192.168.0.102:8080/video"

# Load your custom trained model
print("Loading model...")
try:
    # Kept your exact local path
    model = YOLO(r"E:\Class\control project\CODE\asholei code\best.pt")
except Exception as e:
    print("Error: Could not find 'best.pt'. Make sure it is in the correct folder!")
    exit()

# Open the Camera Stream
cap = cv2.VideoCapture(CAM_URL)

# OPTIMIZATION: Reduce the buffer size to 1 to prevent the video stream from lagging behind real-time
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

if not cap.isOpened():
    print(f"Error: Could not connect to {CAM_URL}")
    print("1. Check if the IP Webcam app is currently running 'Start Server'.")
    print("2. Check if your laptop and phone are on the exact same Wi-Fi.")
    exit()

print("Starting Phone Camera Fire Detection... Press 'q' to quit.")

while True:
    # 1. Read a frame from the stream
    ret, frame = cap.read()
    if not ret:
        print("Frame lost, retrying...")
        continue

    # 2. Resize to 640x480 (VGA)
    # Changed from 320x320. Phones shoot in 4:3 or 16:9. Squishing it to a square (320x320) 
    # distorts the image, which ruins the coordinate math for your robot.
    frame = cv2.resize(frame, (640, 480))

    # 3. Run YOLO detection
    # Using 0.35 confidence. Since the phone is much clearer, if it detects false fires (like lightbulbs), 
    # raise this to 0.50 or 0.60.
    results = model(frame, stream=True, conf=0.35, verbose=False)

    # 4. Draw boxes
    for r in results:
        boxes = r.boxes
        for box in boxes:
            # Get box coordinates
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            
            # Draw Red Box
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
            
            # Add Label
            cv2.putText(frame, "FIRE DETECTED", (x1, y1 - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

    # 5. Show the video
    cv2.imshow("Overhead Phone Stream - Fire Detector", frame)

    # Press 'q' to exit
    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()