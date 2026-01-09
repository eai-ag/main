#!/usr/bin/env python3
import cv2

def view_stream(url='tcp://10.99.2.59:5000'):

    print(f"Connecting to {url}...")

    cap = cv2.VideoCapture(url)

    if not cap.isOpened():
        print(f"Error: Could not open stream at {url}")
        return

    backend = cap.getBackendName()
    print(f"Stream opened successfully using backend: {backend}")
    print("Press 'q' to quit.")

    try:
        while True:
            ret, frame = cap.read()

            if not ret:
                print("Failed to receive frame")
                break

            cv2.imshow('Stream', frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        print("\nStream interrupted by user")
    finally:
        cap.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    vieg_stream()
