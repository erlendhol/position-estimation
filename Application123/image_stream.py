import depthai as dai
import cv2

class imageStream():

    def __init__(self):

        # Start defining a pipeline
        self.pipeline = dai.Pipeline()

        """
        Defining the RGB camera
        """
        self.camRgb = self.pipeline.createColorCamera()
        self.camRgb.setPreviewSize(1920, 1080)
        self.camRgb.setBoardSocket(dai.CameraBoardSocket.RGB)
        self.camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        self.camRgb.setInterleaved(False)
        self.camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)

        # Create output
        self.xoutRgb = self.pipeline.createXLinkOut()
        self.xoutRgb.setStreamName("rgb")
        self.camRgb.preview.link(self.xoutRgb.input)

        """
        Defining the Stereo camera
        """
        # Define a source - two mono (grayscale) cameras
        self.monoLeft = self.pipeline.createMonoCamera()
        self.monoRight = self.pipeline.createMonoCamera()
        self.stereo = self.pipeline.createStereoDepth()
        self.spatialLocationCalculator = self.pipeline.createSpatialLocationCalculator()

        self.xoutDepth = self.pipeline.createXLinkOut()
        self.xoutSpatialData = self.pipeline.createXLinkOut()
        self.xinSpatialCalcConfig = self.pipeline.createXLinkIn()

        self.xoutDepth.setStreamName("depth")
        self.xoutSpatialData.setStreamName("spatialData")
        self.xinSpatialCalcConfig.setStreamName("spatialCalcConfig")

        # MonoCamera
        self.monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
        self.monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
        self.monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
        self.monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)

        self.outputDepth = True
        self.outputRectified = False
        self.lrcheck = False
        self.subpixel = False

        # StereoDepth
        self.stereo.setOutputDepth(self.outputDepth)
        self.stereo.setOutputRectified(self.outputRectified)
        self.stereo.setConfidenceThreshold(255)

        self.stereo.setLeftRightCheck(self.lrcheck)
        self.stereo.setSubpixel(self.subpixel)

        self.monoLeft.out.link(self.stereo.left)
        self.monoRight.out.link(self.stereo.right)

        """
        Starting the spatial calculator and defining the ROI
        """
        self.spatialLocationCalculator.passthroughDepth.link(self.xoutDepth.input)
        self.stereo.depth.link(self.spatialLocationCalculator.inputDepth)

        self.topLeft = dai.Point2f(0.0, 0.0)
        self.bottomRight = dai.Point2f(0.0, 0.0)

        self.spatialLocationCalculator.setWaitForConfigInput(False)
        self.config = dai.SpatialLocationCalculatorConfigData()
        self.config.depthThresholds.lowerThreshold = 100
        self.config.depthThresholds.upperThreshold = 10000
        self.config.roi = dai.Rect(self.topLeft, self.bottomRight)
        self.spatialLocationCalculator.initialConfig.addROI(self.config)
        self.spatialLocationCalculator.out.link(self.xoutSpatialData.input)
        self.xinSpatialCalcConfig.out.link(self.spatialLocationCalculator.inputConfig)

        """
        Defining and starting the pipeline
        """
        self.device = dai.Device(self.pipeline)
        self.device.startPipeline()

        """
        Initiating the output queues for both depth frame and RBG frame
        """
        self.depthQueue = self.device.getOutputQueue(name="depth", maxSize=4, blocking=False)
        self.spatialCalcQueue = self.device.getOutputQueue(name="spatialData", maxSize=4, blocking=False)
        self.spatialCalcConfigInQueue = self.device.getInputQueue("spatialCalcConfig")

        self.qRgb = self.device.getOutputQueue(name="rgb", maxSize=4, blocking=False)

        self.width = self.camRgb.getResolutionWidth()
        self.heigth = self.camRgb.getResolutionHeight()
        self.fov = 68.8

    def get_frame(self):
        inDepth = self.depthQueue.get() # blocking call, will wait until a new data has arrived
        inDepthAvg = self.spatialCalcQueue.get() # blocking call, will wait until a new data has arrived
        self.depthFrame = inDepth.getFrame()
        self.spatialData = inDepthAvg.getSpatialLocations()

        inRgb = self.qRgb.get()  # blocking call, will wait until a new data has arrived
        self.frame = inRgb.getCvFrame()
        
        return self.frame, self.depthFrame, self.spatialData

    def get_image_resolution(self):
        return self.heigth, self.width, self.fov

    def update_config(self, topLeft, bottomRight):
        self.config.roi = dai.Rect(topLeft, bottomRight)
        self.cfg = dai.SpatialLocationCalculatorConfig()
        self.cfg.addROI(self.config)
        self.spatialCalcConfigInQueue.send(self.cfg)