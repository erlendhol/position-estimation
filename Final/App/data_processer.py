from threading import Thread
import cv2
import numpy as np
import time
import math
from image_stream import imageStream
import depthai as dai
class dataProcesser:

    def __init__(self, port='/dev/ttyUSB0', baud=115200):
        self.cx = 0.0
        self.cy = 0.0
        self.cx_ref = 0
        self.cy_ref = 0
        self.z_translatoric_offset = 0.0
        self.x_translatoric_offset = 0.0
        self.x_angle_offset = 0.0
        self.first_run = True
        self.yellowLower = (15,60,50)
        self.yellowUpper = (45,200,255)
        self.z_depth_ref = 0
        self.frame = None
        self.get_frame = None
        self.depthFrame = None
        self.spatialDepth = None
        self.stop = False
        self.port = port
        self.baud = baud
        self.image_stream = imageStream()
        self.heigth, self.width, self.fov = self.image_stream.get_image_resolution()
        self.deg_per_pix = self.fov / self.heigth
        self.frame, self.depth, self.spatialDepth = self.image_stream.get_frame()
        self.img_thread = Thread(target=self.get_image_stream, args=())
        self.img_thread.daemon = True
        self.img_thread.start()
        self.topLeftX = 0.0
        self.topLeftY = 0.0
        self.bottomRightX = 1.0
        self.bottomRightY = 1.0
        self.z_ref_noted = False
        self.z_depth = 0.0
        self.z_depth_ref = 0.0
        self.imu_euler_x = 0.0
        

    def get_image_stream(self):
        while not self.stop: 
            self.frame, self.depthFrame, self.spatialDepth = self.image_stream.get_frame()

    def get_RGB_frame(self):
        return self.frame

    def kill_thread(self):
        self.stop = True

    def get_contour_center(self, contour):
        # Get the moments in the contour
        M = cv2.moments(contour)
        cx=-1
        cy=-1
        # Make sure not to divide by 0
        if(M['m00']!=0):
            # Calculate x and y coordinates for the center of the contour
            cx=int(M['m10']/M['m00'])
            cy=int(M['m01']/M['m00'])
        return cx,cy

    def convert_2_HSV(self, frame, show=0):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        if show:
            cv2.imshow('HSV frame', hsv)
        return hsv

    def get_mask_from_hsv_frame(self, hsv_frame, lowerValues, upperValues, normalized_mask=0):
        # Define a mask using the lower and upper bound
        color_mask = cv2.inRange(hsv_frame, lowerValues, upperValues)
        if normalized_mask:
            # Create nparray to multiply with depth image to filter out depth values which are not in the color area
            color_mask_normalized = color_mask/255  
            return color_mask, color_mask_normalized
        return color_mask  

    def get_grey_image_from_frame(self, frame):
        blurred_frame = cv2.GaussianBlur(frame, (5, 5), 0)
        grey_frame = cv2.cvtColor(blurred_frame, cv2.COLOR_BGR2GRAY)
        return grey_frame

    def get_biggest_contour(self, mask, return_area=0):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        biggest_area = 0
        biggest_contour = 0
        for contour in contours:
            area = cv2.contourArea(contour)
            
            if area > biggest_area:
                biggest_area = area
                biggest_contour = contour
        
        if return_area:
            return biggest_contour, biggest_area
        return biggest_contour
    
    def update_roi(self):
        self.topLeftX = float((self.cx/self.width)-0.025)
        self.topLeftY = float((self.cy/self.heigth)-0.025)
        if self.topLeftX < 0.0:
            self.topLeftX = 0.0
        if self.topLeftY < 0.0:
            self.topLeftY = 0.0

        self.bottomRightX = float((self.cx/self.width)+0.025)
        self.bottomRightY = float((self.cy/self.heigth)+0.025)
        if self.bottomRightX > 1.0:
            self.bottomRightX = 1.0
        if self.bottomRightY > 1.0:
            self.bottomRightY = 1.0
    
    def update_config(self):
        self.topLeft_tuple = [self.topLeftX, self.topLeftY]
        self.bottomRight_tuple = [self.bottomRightX, self.bottomRightY]
        self.topLeft = dai.Point2f(self.topLeft_tuple[0], self.topLeft_tuple[1])
        self.bottomRight = dai.Point2f(self.bottomRight_tuple[0], self.bottomRight_tuple[1])
        self.image_stream.update_config(self.topLeft, self.bottomRight)

    def run(self, euler_x=0):
        if self.first_run:
            self.update_roi()
        self.update_config()
        self.frame = self.get_RGB_frame()
        
        self.hsv = self.convert_2_HSV(self.frame)
        self.yellow_mask = self.get_mask_from_hsv_frame(self.hsv, self.yellowLower, self.yellowUpper)
        self.biggest_contour = self.get_biggest_contour(self.yellow_mask)
        self.cx, self.cy = self.get_contour_center(self.biggest_contour)
        if self.first_run:
            self.cx_ref = self.cx
            self.cy_ref = self.cy
        pixle_offset_x = self.cx_ref - self.cx
        pixle_offset_y = self.cy_ref - self.cy
        camera_angle_offset = pixle_offset_x*self.deg_per_pix
        self.x_angle_offset = camera_angle_offset - self.imu_euler_x
        self.y_angle_offset = pixle_offset_y*self.deg_per_pix

        if self.depthFrame is not None:
            for depthData in self.spatialDepth:
                roi = depthData.config.roi
                roi = roi.denormalize(width=self.depthFrame.shape[1], height=self.depthFrame.shape[0])
                pt1 = (int(self.topLeftX*self.width), int(self.topLeftY*self.heigth))
                pt2 = (int(self.bottomRightX*self.width), int(self.bottomRightY*self.heigth))
                color = (255,255,255)
                cv2.rectangle(self.frame, pt1=pt1, pt2=pt2, color=color)
                z_depth = int(depthData.spatialCoordinates.z)
                cv2.putText(self.frame, f"Z: {z_depth} mm", (int(self.topLeftX) + 10, int(self.topLeftY) + 50), cv2.FONT_HERSHEY_TRIPLEX, 0.5, (255,255,255))
        
        if not self.z_ref_noted and (self.z_depth > 5 or self.z_depth < -5):
            self.z_depth_ref = self.z_depth * math.cos(math.radians(self.x_angle_offset))
            z_translatoric_offset = 0.
            self.z_ref_noted = True
        else:
            z_translatoric_offset = self.z_depth_ref - self.z_depth * math.cos(math.radians(self.x_angle_offset))
        
        x_translatoric_offset = self.z_depth * math.sin(math.radians(self.x_angle_offset))
        self.get_frame = self.frame
        first_run = False

        self.z_translatoric_offset = z_translatoric_offset
        self.x_translatoric_offset = x_translatoric_offset

    def get_processed_data(self):
        return self.z_translatoric_offset, self.x_translatoric_offset, self.x_angle_offset

    def get_frame(self):
        return self.get_frame
