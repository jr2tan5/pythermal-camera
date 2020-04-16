import cv2
#import PySimpleGui as sg
import datetime

timenow = datetime.datetime.now()

cv2.namedWindow("raspberry_pi_camera_v1_3")
cv2.namedWindow("flir_thermal_camera")
rasp_pi_camera = 2
thermal_camera = 0
scale_percent = 400 # percent of original size

vc_normal = cv2.VideoCapture(rasp_pi_camera)
vc_thermal = cv2.VideoCapture(thermal_camera)

if vc_normal.isOpened(): # try to get the first frame
    rval, frame = vc_normal.read()
else:
    rval = False
    
if vc_thermal.isOpened(): # try to get the first frame
    rval_t, frame_t = vc_thermal.read()
    width = int(frame_t.shape[1] * scale_percent / 100)
    height = int(frame_t.shape[0] * scale_percent / 100)
    dim_t = (width, height)
    # resize image
    frame_t_r = cv2.resize(frame_t, dim_t, interpolation = cv2.INTER_AREA)
    cv2.imwrite(str(timenow)+".jpg",frame_t_r)
else:
    rval_t = False
    
print "rval = " + str(rval)
print "rval_t = " + str(rval_t)

while rval and rval_t:
    cv2.imshow("raspberry_pi_camera_v1_3", cv2.flip(cv2.transpose(cv2.flip(cv2.transpose(frame),flipCode=1)),flipCode=1))
    #cv2.imshow("raspberry_pi_camera_v1_3", frame)
    cv2.imshow("flir_thermal_camera", frame_t_r)
    rval, frame = vc_normal.read()
    rval_t, frame_t = vc_thermal.read()
    frame_t_r = cv2.resize(frame_t, dim_t, interpolation = cv2.INTER_AREA)
    key = cv2.waitKey(10)
    if key == 27: # exit on ESC
        break
