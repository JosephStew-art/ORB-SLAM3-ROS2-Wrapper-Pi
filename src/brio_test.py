import cv2

def main():
    # Open the default camera (usually the built-in webcam)
    # If you have multiple cameras, you might need to change the index (e.g., cv2.VideoCapture(1))

    vid = cv2.VideoCapture(2)
    #vid.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    #vid.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

    if not vid.isOpened():
        print(f"Failed to open camera")
        return


    while True:
        # Capture frame-by-frame
        ret, frame = vid.read()

        # If frame is read correctly ret is True
        if not ret:
            print("Error: Can't receive frame (stream end?). Exiting ...")
            break

        # Display the resulting frame
        cv2.imshow('Camera Live Stream', frame)

        # Press 'q' to quit
        if cv2.waitKey(1) == ord('q'):
            break

    # When everything done, release the capture and destroy all windows
    vid.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()