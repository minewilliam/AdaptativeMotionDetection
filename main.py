"""
MIT License

Copyright (c) 2023 William Longpré, Jonathan (the spider)

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

"""

import pyvirtualcam
import numpy as np
import cv2
import time

# Motion filter parameters:
MOTION_INTERVAL = 0.1 * 10e8 # nanoseconds
EXPOSURE_FRAMES = 50 # Number of frames to average for long term exposure effect
# Increase this value if your application can generate higher contrasts.
# A bigger value translates to a bigger pixel selection, but also more error.
THRESHOLD_CONTRAST = 2
MAX_BLOB_AREA = 400 # Largest detectable area on screen, in pixels

# Blurring helps mitigate sensor noise.
BLUR_KSIZE = 3
blurKernel = np.ones((BLUR_KSIZE, BLUR_KSIZE), np.float64)/(BLUR_KSIZE**2)

# Device id changes when you unplug a device
CAPTURE_DEVICE_ID = 1
# Device property indexes
PROP_WIDTH = 3
PROP_HEIGHT = 4
PROP_FPS = 5

# TODO: List available devices.
capture = cv2.VideoCapture(CAPTURE_DEVICE_ID)
captureWidth = int(capture.get(PROP_WIDTH))
captureHeight = int(capture.get(PROP_HEIGHT))
captureFps = capture.get(PROP_FPS)

print(f"Using device {CAPTURE_DEVICE_ID}: {captureWidth}x{captureHeight} @ {captureFps}fps")

def addMats(mat1, mat2, ratio=0.5):
    return np.floor(mat1*ratio + mat2*(1-ratio)).astype(np.uint8)


with pyvirtualcam.Camera(
    width=captureWidth,
    height=captureHeight,
    fps=captureFps) as cam:

    success, img = capture.read()
    pastImgTime = time.time_ns()
    imgBlurred = cv2.filter2D(img, -1, blurKernel)
    # Invert colors
    pastImg = 255 - imgBlurred
    averageImg = pastImg

    while success:
        frameTime = time.time_ns()
        if abs(frameTime - pastImgTime) > MOTION_INTERVAL:
            pastImgTime = frameTime
            # Invert colors
            pastImg = 255 - imgBlurred
            # Progressively average motion information.
            averageImg = (averageImg * EXPOSURE_FRAMES + pastImg) / (EXPOSURE_FRAMES + 1)

        imgMotion = addMats(imgBlurred, averageImg, ratio=0.5)
        imgMotionGray = cv2.cvtColor(imgMotion, cv2.COLOR_RGB2GRAY)
        
        # Adapts to the smallest motion in view. This is responsible for the filter going crazy on a static image.
        # TODO: Find a way to balance this.
        smallestMotion = np.min(imgMotionGray)+THRESHOLD_CONTRAST
        _, thresh = cv2.threshold(imgMotionGray, smallestMotion, 255, cv2.THRESH_BINARY_INV)
        threshInflated = cv2.dilate(thresh, np.ones((10,10), np.uint8))

        # In our use case, there is only one target to detect
        # The filter gets more and more sensitive when there is no movement, so many targets appear.
        singleDetection = None
        contours = cv2.findContours(threshInflated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
        for c in contours:
            x, y, w, h = cv2.boundingRect(c)
            area = w * h
            if area > MAX_BLOB_AREA:
                continue
            if singleDetection is not None:
                singleDetection = None
                break
            singleDetection = (int(x-w/2), int(y-h/2), w, h)
        
        imgOut = img
        if singleDetection is not None:
            x, y, w, h = singleDetection
            pos = (x, y)
            imgOut = cv2.rectangle(imgOut, pos, (x+w, y+h), (36,12,255), 2)
            # The spider's name is Jonathan
            cv2.putText(imgOut, 'Jonathan', (x, y-20), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (36,12,255), 2)
        
        cv2.imshow("Webcam", imgOut)
        #cv2.imshow("Webcam", threshInflated)
        #cv2.imshow("Webcam", imgMotion)
        cam.send(cv2.cvtColor(imgOut, cv2.COLOR_RGB2BGR))
        cam.sleep_until_next_frame()
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        success, img = capture.read()
        imgBlurred = cv2.filter2D(img, -1, blurKernel)
