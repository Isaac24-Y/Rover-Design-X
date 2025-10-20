import cv2

cam0 = cv2.VideoCapture(0)
cam1 = cv2.VideoCapture(1)

while True:
    ret0, frame0 = cam0.read()
    ret1, frame1 = cam1.read()

    if ret0 and ret1:
        combined = cv2.hconcat([frame0, frame1])  # lado a lado
        cv2.imshow("Cámaras", combined)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cam0.release()
cam1.release()
cv2.destroyAllWindows()