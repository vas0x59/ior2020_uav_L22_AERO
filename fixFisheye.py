import numpy as np
import cv2
import os
import pickle

def get_undistorted_image(cv2_image, cM = None, dC = None):
    matrix = np.array([[166.23942373, 0.0, 162.19011247],
                       [0.0, 166.5880924, 109.82227736],
                       [0.0, 0.0, 1.0]])
    distortions = np.array([2.15356885e-01, -1.17472846e-01, -3.06197672e-04, -1.09444025e-04, -4.53657258e-03, 5.73090623e-01, -1.27574577e-01, -2.86125589e-02])
    h, w = cv2_image.shape[:2]
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(matrix, distortions, (w, h), 1, (w, h))
    dst = cv2.undistort(cv2_image, matrix, distortions, None, newcameramtx)
    x, y, w, h = roi
    dst = dst[y:y + h, x:x + w]
    height_or, width_or, depth_or = cv2_image.shape
    height_un, width_un, depth_un = dst.shape
    frame = cv2.resize(dst, (0, 0), fx=(width_or / width_un), fy=(height_or / height_un))
    return frame

if not os.path.exists('pics'):
    os.makedirs('pics')


cap = cv2.VideoCapture('LOG.avi')
UPDATE_RATE = 5
fourcc = cv2.VideoWriter_fourcc(*'XVID')
video_writer = cv2.VideoWriter("output.avi", fourcc, UPDATE_RATE, (320, 240))

if (cap.isOpened()== False): 
  print("Error opening video stream or file")

count = 0

while(cap.isOpened()):
    ret, frame = cap.read()
    if ret == True:
        image = get_undistorted_image(frame)
        if count % 15 == 0: cv2.imwrite('pics/'+str(count)+'.jpg', image)
        count += 1
        video_writer.write(image)
        cv2.imshow('img', image)
        if cv2.waitKey(1) == 27:
            break
    else: 
        break

cap.release()
video_writer.release()
cv2.destroyAllWindows()
