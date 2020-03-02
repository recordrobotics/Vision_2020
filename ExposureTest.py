import numpy as np
import cv2
import Distance

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_EXPOSURE, -10)

while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()

    # Display the resulting frame
    cv2.imshow('frame', frame)

    print(Distance.distanceToGoal(frame)[1])
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()