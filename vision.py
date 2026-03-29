# vision.py
# Dependencies: opencv-python, pytesseract
# System requirement: Tesseract-OCR must be installed (https://github.com/tesseract-ocr/tesseract)

import cv2
import pytesseract

pytesseract.pytesseract.tesseract_cmd = r'C:\Program Files\Tesseract-OCR\tesseract.exe'


def get_vision():
    cap = cv2.VideoCapture(0) 
    ret, frame = cap.read()
    cap.release()

    if not ret:
        raise RuntimeError("Failed to capture frame from webcam")

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    
    gray = cv2.resize(gray, None, fx=2, fy=2, interpolation=cv2.INTER_CUBIC) #Upscale = zoom

    # Slight blur to reduce noise before thresholding
    gray = cv2.GaussianBlur(gray, (3, 3), 0)

    thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY
                                    ,21 #Block size; increase if text is large, decrease if small
                                    ,5 #Constant subtracted from the mean
                                    )

    text = pytesseract.image_to_string(thresh, config='--psm 1') #Tesseract mode to handle inclininate text
    return text.strip()


if __name__ == "__main__":
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920) # Request 1080p — OpenCV defaults to 640x480
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    ret, frame = cap.read()
    cap.release()
    if ret:
        cv2.imwrite("capture_test.png", frame) #Save capture_test.png
        print(f"Resolution: {frame.shape[1]}x{frame.shape[0]}")
        print("Saved capture_test.png")

    result = get_vision()
    print(f"Detected: {result}")
