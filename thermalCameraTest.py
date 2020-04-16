import cv2
import RPi.GPIO as GPIO
import datetime
import time
import os

# Assign Functions to Variables
timenow = datetime.datetime.now

# Camera Enum by Raspberry Pi
rasp_pi_camera = 0
thermal_camera = 1

# Set GPIO Object Enum
RASP_PI_CAM_STATUS_LED_EN = 18 # Raspberry Pi Camera Readability Status LED is connected to GPIO 18
THERMAL_CAM_STATUS_LED_EN = 17 # Thermal Camera Readability Status LED is connected to GPIO 17
SCREENSHOT_BUTTON_EN = 21 # Screenshot Button is connected to GPIO21
VIDEO_START_STOP_BUTTON_EN = 20 # Video Start/Stop Button is connected to GPIO20

# File Paths
screenshot_folder = '/home/pi/Desktop/pythermal-camera-git/Screenshots'

# Create OpenCV UI object
cv2.namedWindow("raspberry_pi_camera_v1_3")
cv2.namedWindow("flir_thermal_camera")

# Scale percent of original frame size
scale_percent = 400 # 400 is tested and shown to be the same resolution as the raspberry pi camera

# GPIO settings
GPIO.setmode(GPIO.BCM) # BCM => Broadcom SOC channel
GPIO.setwarnings(False)

# LED Status Lights
GPIO.setup(RASP_PI_CAM_STATUS_LED_EN,GPIO.OUT)
GPIO.setup(THERMAL_CAM_STATUS_LED_EN,GPIO.OUT)

# Push Button Inputs
GPIO.setup(SCREENSHOT_BUTTON_EN,GPIO.IN)
GPIO.setup(VIDEO_START_STOP_BUTTON_EN,GPIO.IN)

# Create Video Capture Object
vc_normal = cv2.VideoCapture(rasp_pi_camera)
vc_thermal = cv2.VideoCapture(thermal_camera)

def turnOnOrOffStatus_LED(cameraStatus, cameraLED_GPIO_Name):
    if cameraStatus:
        GPIO.output(cameraLED_GPIO_Name, GPIO.HIGH)
    else:
        GPIO.output(cameraLED_GPIO_Name,GPIO.LOW)
        
def saveImages(normal_frame, thermal_frame):
    # Check if screenshot folder exits:
    if not os.path.exists(screenshot_folder):
        os.mkdir(screenshot_folder)
    # Save image
    cv2.imwrite(screenshot_folder + "/" + str(timenow() ) + ".jpg",normal_frame)
    cv2.imwrite(screenshot_folder + "/" + str(timenow() ) + "_thermal.jpg",thermal_frame)

if vc_normal.isOpened(): # try to get the first frame
    normal_cam_status, normal_frame = vc_normal.read()
    turnOnOrOffStatus_LED(normal_cam_status, RASP_PI_CAM_STATUS_LED_EN)
else:
    normal_cam_status = False
    
if vc_thermal.isOpened(): # try to get the first frame
    thermal_cam_status, thermal_frame = vc_thermal.read()
    turnOnOrOffStatus_LED(thermal_cam_status, THERMAL_CAM_STATUS_LED_EN)
    
    # Get Scaled Thermal Image parameters
    thermal_frame_width = int(thermal_frame.shape[1] * scale_percent / 100)
    thermal_frame_height = int(thermal_frame.shape[0] * scale_percent / 100)
    
    # Save desired resolution dimensions
    thermal_frame_dim = (thermal_frame_width, thermal_frame_height)
else:
    thermal_cam_status = False

while normal_cam_status and thermal_cam_status:
    # Get Status and Frames from camera
    normal_cam_status, normal_frame = vc_normal.read()
    turnOnOrOffStatus_LED(normal_cam_status, RASP_PI_CAM_STATUS_LED_EN)
    thermal_cam_status, thermal_frame = vc_thermal.read()
    turnOnOrOffStatus_LED(thermal_cam_status, THERMAL_CAM_STATUS_LED_EN)
    
    # Image Manipulation
    normal_frame_flipped_180 = cv2.flip(cv2.transpose(cv2.flip(cv2.transpose(normal_frame),flipCode=1)),flipCode=1) # Because the Raspberry Pi Camera is Mounted Upside, we need to rotate it 180 degrees
    thermal_frame_resized = cv2.resize(thermal_frame, thermal_frame_dim, interpolation = cv2.INTER_AREA) # Thermal Camera's Native Resolution is 4 times smaller than the Raspi Cam's
    
    # Display Image in UI
    cv2.imshow("raspberry_pi_camera_v1_3", normal_frame_flipped_180)
    cv2.imshow("flir_thermal_camera", thermal_frame_resized)
    
    # Screenshot purposes 
    if GPIO.input(SCREENSHOT_BUTTON_EN) == GPIO.HIGH: # Check if screenshot button is pressed
        saveImages(normal_frame_flipped_180, thermal_frame_resized) 
        print "screenshot taken" # Used for debugging
    
    # Delay to show frame for desired duration before reading the next frame
    key = cv2.waitKey(10)
    
    if key == 27: # exit on ESC
        break
