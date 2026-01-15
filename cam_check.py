import cv2

def check_cams(max_to_test=5):
    for i in range(max_to_test):
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            ret, frame = cap.read()
            if ret:
                cv2.imshow(f"Camera ID: {i} - Press any key to next", frame)
                cv2.waitKey(0)
            cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    check_cams()