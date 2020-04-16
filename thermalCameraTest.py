import cv2
import RPi.GPIO as GPIO
import datetime
import time

timenow = datetime.datetime.now()

# Camera Enum by Raspberry Pi
rasp_pi_camera = 0
thermal_camera = 1

cv2.namedWindow("raspberry_pi_camera_v1_3")
cv2.namedWindow("flir_thermal_camera")

# Scale percent of original frame size
scale_percent = 400 # 400 is tested and shown to be the same resolution as the raspberry pi camera

# GPIO settings
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(18,GPIO.OUT)
GPIO.setup(17
,GPIO.OUT)

vc_normal = cv2.VideoCapture(rasp_pi_camera)
vc_thermal = cv2.VideoCapture(thermal_camera)

def turnOnOrOffLED(cameraStatus, cameraLED_GPIO_Name):
    if cameraStatus:
        GPIO.output(cameraLED_GPIO_Name, GPIO.HIGH)
    else:
        GPIO.output(cameraLED_GPIO_Name,GPIO.LOW)

if vc_normal.isOpened(): # try to get the first frame
    rval, frame = vc_normal.read()
    turnOnOrOffLED(rval, 18)
else:
    rval = False
    
if vc_thermal.isOpened(): # try to get the first frame
    rval_t, frame_t = vc_thermal.read()
    
    turnOnOrOffLED(rval_t, 17)
    width = int(frame_t.shape[1] * scale_percent / 100)
    height = int(frame_t.shape[0] * scale_percent / 100)
    dim_t = (width, height)
    # resize image
    frame_t_r = cv2.resize(frame_t, dim_t, interpolation = cv2.INTER_AREA)
    
    # Save image
    #cv2.imwrite(str(timenow)+".jpg",frame_t_r)
else:
    rval_t = False
    
print "rval = " + str(rval)
print "rval_t = " + str(rval_t)

while rval and rval_t:
    cv2.imshow("raspberry_pi_camera_v1_3", cv2.flip(cv2.transpose(cv2.flip(cv2.transpose(frame),flipCode=1)),flipCode=1))
    #cv2.imshow("raspberry_pi_camera_v1_3", frame)
    cv2.imshow("flir_thermal_camera", frame_t_r)
    rval, frame = vc_normal.read()
    turnOnOrOffLED(rval, 18)
    rval_t, frame_t = vc_thermal.read()
    turnOnOrOffLED(rval_t, 17)
    frame_t_r = cv2.resize(frame_t, dim_t, interpolation = cv2.INTER_AREA)
    key = cv2.waitKey(10)
    if key == 27: # exit on ESC
        break
