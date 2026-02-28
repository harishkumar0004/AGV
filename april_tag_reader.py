import apriltag
import cv2
import numpy as np

image = None
cap = cv2.VideoCapture(0)
# detector = apriltag.apriltag('tag36h11', threads=4)
detector = apriltag.apriltag(
    family='tag36h11',      # Tag family
    threads=4,              # Number of threads
    maxhamming=1,           # Maximum hamming distance for error correction
    decimate=2.0,           # Image downsampling factor
    blur=0.0,               # Gaussian blur sigma
    refine_edges=True,      # Refine quad edges
    debug=False             # Debug mode
)


while cap.isOpened():
    ret, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    detections = detector.detect(gray)

    # Draw detections
    for det in detections:
        pts = det['lb-rb-rt-lt'].astype(int)
        cv2.polylines(frame, [pts], True, (0,255,0), 2)
        cx, cy = map(int, det['center'])
        cv2.circle(frame, (cx,cy), 5, (0,0,255), -1)

        tag_id = det['id']

        # position text above tag
        x = pts[:,0].min()
        y = pts[:,1].min() - 10

        cv2.putText(frame, f"ID:{tag_id}", (x,y),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (255,0,0), 2)

    cv2.imshow('image', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break


# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()

