from picamera2 import Picamera2
import cv2, time, os

picam2 = Picamera2()
picam2.configure(
    picam2.create_preview_configuration(
        main={"size": (640,480), "format":"RGB888"}
    )
)
picam2.start()
time.sleep(2)

os.makedirs("images", exist_ok=True)
i = 0

while True:
    frame = picam2.capture_array()
    cv2.imwrite(f"images/red_{i:04d}.jpg", frame)
    print("Saved", i)
    i += 1
    time.sleep(0.4)
