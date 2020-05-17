import cv2
import RPi.GPIO as GPIO
import datetime
from time import time as timeInSeconds
import os

# Global Variable
videoCapture_Status = 0x0
screenshot_button_pressed = False
video_button_pressed = False

# Assign Functions to Variables
timenow = datetime.datetime.now

# Camera Enum by Raspberry Pi
rasp_pi_camera_id = 0
thermal_camera_id = 0

# Camera Status
normal_cam_status = False
thermal_cam_status = False
# Set GPIO Object Enum
RASP_PI_CAM_STATUS_LED_EN = 18 # Raspberry Pi Camera Readability Status LED is connected to GPIO18 of the raspberry pi
THERMAL_CAM_STATUS_LED_EN = 17 # Thermal Camera Readability Status LED is connected to GPIO17 of the raspberry pi
VIDEO_CAPTURE_STATUS_LED_EN = 16 # Video Capture Status LED is connected to GPIO16 of the raspberry pi
SCREENSHOT_BUTTON_EN = 21 # Screenshot Button is connected to GPIO21 of the raspberry pi
VIDEO_START_STOP_BUTTON_EN = 20 # Video Start/Stop Button is connected to GPIO20 of the raspberry pi

# File Paths
screenshot_folder = '/home/pi/Desktop/pythermal-camera-git/Screenshots'
video_folder = '/home/pi/Desktop/pythermal-camera-git/Video'

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
GPIO.setup(VIDEO_CAPTURE_STATUS_LED_EN,GPIO.OUT)

# Push Button Inputs
GPIO.setup(SCREENSHOT_BUTTON_EN,GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(VIDEO_START_STOP_BUTTON_EN,GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

def setRaspPiCameraID(cameraID):
    global rasp_pi_camera_id
    
    rasp_pi_camera_id = cameraID

def setThermalCameraID(cameraID):
    global thermal_camera_id
    thermal_camera_id = cameraID

def checkAvailableDeviceID(cam):
    return cam.isOpened()

def getFrameWidth(cam):
    cap_status, cap_frame = cam.read()
    return cap_frame.shape[1]

def scanAndSaveCameraID():
    global normal_cam_status
    global thermal_cam_status
    
    scanID = 0
    print('The cameraID is Valid?')
    while scanID < 5:
        cam = cv2.VideoCapture(scanID)
        cameraID_IsValid = checkAvailableDeviceID(cam)
        print(cameraID_IsValid)
        if cameraID_IsValid:
            frameWidth = getFrameWidth(cam)
            if frameWidth < 640:
                setThermalCameraID(scanID)
                thermal_cam_status = True
                print('Thermal Camera ID is ' + str(scanID))
            else:
                setRaspPiCameraID(scanID)
                normal_cam_status = True
                print('Rasp Pi Camera ID is ' + str(scanID))
        scanID+=1
        cam.release()
    print('\n\n')

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
    time = timenow()
    
    normal_frame_save = resizeFrameForSaving(normal_frame)
    thermal_frame_save = resizeFrameForSaving(thermal_frame)
    
    cv2.imwrite(screenshot_folder + "/" + str(time) + ".jpg",normal_frame_save)
    cv2.imwrite(screenshot_folder + "/" + str(time) + "_thermal.jpg",thermal_frame_save)

def saveVideo(normal_writer, thermal_writer,normal_frame, thermal_frame):
    # Check if video folder exits:
    if not os.path.exists(video_folder):
        os.mkdir(video_folder)
    normal_frame_save = resizeFrameForSaving(normal_frame)
    thermal_frame_save = resizeFrameForSaving(thermal_frame)
    normal_writer.write(normal_frame_save)
    thermal_writer.write(thermal_frame_save)
    
def getVideoWriter(videoName, dimensions):
    return cv2.VideoWriter(video_folder + "/" + videoName, cv2.VideoWriter_fourcc(*'XVID'), 10, dimensions)

def init():
    turnOnOrOffStatus_LED(False, THERMAL_CAM_STATUS_LED_EN)
    turnOnOrOffStatus_LED(False, RASP_PI_CAM_STATUS_LED_EN)
    turnOnOrOffStatus_LED(False, VIDEO_CAPTURE_STATUS_LED_EN)

def getDesiredFrameResolution(capture):
    status, frame = capture.read()
    
    # Get Scaled Thermal Image parameters
    width = int(frame.shape[1] * scale_percent / 100) # scale_percent is a defined global variable
    height = int(frame.shape[0] * scale_percent / 100) # scale_percent is a defined global variable
    return (width, height)
    
def resizeFrameForSaving(frame):
    width = int(frame.shape[1] / (scale_percent / 100)) # scale_percent is a defined global variable
    height = int(frame.shape[0] / (scale_percent / 100)) # scale_percent is a defined global variable
    return cv2.resize(frame, desired_dim, interpolation = cv2.INTER_AREA)
    
def rotate180Degrees(frame):
    #return cv2.flip(cv2.transpose(cv2.flip(cv2.transpose(frame),flipCode=1)),flipCode=1)
    return cv2.rotate(frame,cv2.ROTATE_180)
    
def resizeFrameToDesiredDim(frame, desired_dim):
    return cv2.resize(frame, desired_dim, interpolation = cv2.INTER_AREA)

init()
scanAndSaveCameraID()

# Create Video Capture Object
if normal_cam_status and thermal_cam_status:
    vc_normal = cv2.VideoCapture(rasp_pi_camera_id)
    vc_thermal = cv2.VideoCapture(thermal_camera_id)

    desired_dim = getDesiredFrameResolution(vc_thermal)

while normal_cam_status and thermal_cam_status:
    timeStart = timeInSeconds()
    
    # Get Status and Frames from camera
    normal_cam_status, normal_frame = vc_normal.read()
    turnOnOrOffStatus_LED(normal_cam_status, RASP_PI_CAM_STATUS_LED_EN)
    thermal_cam_status, thermal_frame = vc_thermal.read()
    turnOnOrOffStatus_LED(thermal_cam_status, THERMAL_CAM_STATUS_LED_EN)
    
    # Image Manipulation
    normal_frame_flipped_180 = rotate180Degrees(normal_frame) # Because the Raspberry Pi Camera is Mounted Upside, we need to rotate it 180 degrees
    thermal_frame_resized =  resizeFrameToDesiredDim(thermal_frame, desired_dim) # Thermal Camera's Native Resolution is 4 times smaller than the Raspi Cam's
    
    # Display Image in UI
    cv2.imshow("raspberry_pi_camera_v1_3", normal_frame_flipped_180)
    cv2.imshow("flir_thermal_camera", thermal_frame_resized)
    
    # Screenshot Purposes
    if GPIO.input(SCREENSHOT_BUTTON_EN) == GPIO.HIGH: # Check if screenshot button is pressed
        screenshot_button_pressed = True
    else:
        # This Ensures that there is only one input per button press
        if screenshot_button_pressed == True:
            saveImages(normal_frame_flipped_180, thermal_frame_resized) 
            print "screenshot taken" # Used for debugging
            screenshot_button_pressed = False
    
    # Video Saving Purposes
    if GPIO.input(VIDEO_START_STOP_BUTTON_EN) == GPIO.HIGH:
        video_button_pressed = True
    else:
        # This Ensures that there is only one input per button press
        if video_button_pressed == True:
            print('video record pressed')
            
            # Stop Recording Procedures if Button is pressed
            if videoCapture_Status:
                normal_video_writer.release()
                thermal_video_writer.release()
                
            # Toggle Start/Stop when button is pressed
            videoCapture_Status ^= 0x1
            
            # Start Recording Procedures if Button is pressed
            if videoCapture_Status:
                time = str(timenow())
                normal_video_filename = time + ".avi"
                thermal_video_filename = time + "_thermal.avi"
                
                normal_video_writer = getVideoWriter(normal_video_filename, desired_dim)
                thermal_video_writer = getVideoWriter(thermal_video_filename, desired_dim)
            
            video_button_pressed = False
    
    # Save the Frame while video is recording
    if videoCapture_Status:
        saveVideo(normal_video_writer, thermal_video_writer, normal_frame_flipped_180, thermal_frame_resized)
    
    # Show the current recording status on the LED
    turnOnOrOffStatus_LED(videoCapture_Status, VIDEO_CAPTURE_STATUS_LED_EN)
        
    # Delay to show frame for desired duration before reading the next frame
    key = cv2.waitKey(10)

    timeEnd = timeInSeconds()
    print(timeStart - timeEnd)
