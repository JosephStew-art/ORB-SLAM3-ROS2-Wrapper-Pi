import cv2
import time

def capture_image():
    # Open the default camera (usually the built-in webcam)
    cap = cv2.VideoCapture(0)

    # Check if the camera opened successfully
    if not cap.isOpened():
        print("Error: Could not open camera.")
        return

    # Set the resolution to 1920x1080
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

    # Check if the resolution was set successfully
    width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    print(f"Camera resolution set to: {width}x{height}")

    # Wait for the camera to initialize and adjust light levels
    time.sleep(2)

    # Capture frame
    ret, frame = cap.read()

    # If frame is read correctly ret is True
    if not ret:
        print("Error: Can't capture image. Exiting ...")
        cap.release()
        return

    # Generate a filename with timestamp
    timestamp = time.strftime("%Y%m%d-%H%M%S")
    filename = f"captured_image_{timestamp}.jpg"

    # Save the image
    cv2.imwrite(filename, frame)
    print(f"Image saved as {filename}")

    # Release the capture
    cap.release()

if __name__ == "__main__":
    capture_image()