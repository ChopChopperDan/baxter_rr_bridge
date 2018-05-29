#!/usr/bin/env python
import roslib
roslib.load_manifest('baxter_rr_bridge')
import rospy
import baxter_interface
import cv2
import cv2.aruco as aruco
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo

import sys, argparse
import struct
import time
import RobotRaconteur as RR
import thread
import threading
import numpy
import traceback

baxter_servicedef="""
#Service to provide simple interface to Baxter
service BaxterCamera_interface

option version 0.4

struct BaxterImage
    field int32 width
    field int32 height
    field int32 step
    field uint8[] data
end struct

struct CameraIntrinsics
    field double[] K
    field double[] D
end struct

struct ImageHeader
    field int32 width
    field int32 height
    field int32 step
end struct

struct ARtagInfo
    field double[] rvec
    field double[] tvec
    field int32[] ids
end struct

object BaxterCamera

    property uint8 camera_open

    # camera control functions
    function void openCamera()
    function void closeCamera()
    function void setExposure(int16 exposure)
    function void setGain(int16 gain)
    function void setWhiteBalance(int16 red, int16 green, int16 blue)
    function void setFPS(double fps)
    function void setCameraIntrinsics(CameraIntrinsics data)
    function void setMarkerSize(uint8 markerSize)
    
    # functions to acquire data on the image
    function BaxterImage getCurrentImage()
    function ImageHeader getImageHeader()
    function CameraIntrinsics getCameraIntrinsics()
    function uint8 getMarkerSize()
    function ARtagInfo ARtag_Detection()
    
    # pipe to stream images through
    pipe BaxterImage ImageStream
    
end object

"""
class BaxterCamera_impl(object):
    def __init__(self, camera_name, mode, half_res):
        print "Initializing ROS Node"
        rospy.init_node('baxter_cameras', anonymous=True)
        
        
        # Lock for multithreading
        self._lock = threading.RLock()
        
        # for image pipe
        self._imagestream=None
        self._imagestream_endpoints = dict()
        self._imagestream_endpoints_lock = threading.RLock()
        
        # get access to camera controls from RSDK
        self._camera = baxter_interface.CameraController(camera_name)
        self._camera_name = camera_name;
        
        # automatically close camera at start
        # self._camera.close()
        self._camera_open = False
        
        # set constant ImageHeader structure
        self.setResolution(mode,half_res)
        self._image_header = RR.RobotRaconteurNode.s.NewStructure( 
                                    "BaxterCamera_interface.ImageHeader" )
        self._image_header.width = int(self._camera.resolution[0])
        self._image_header.height = int(self._camera.resolution[1])
        self._image_header.step = int(4)
        
        
        self._camera_intrinsics = None
        
        # set exposure, gain, white_balance to auto
        self._camera.exposure = self._camera.CONTROL_AUTO
        self._camera.gain = self._camera.CONTROL_AUTO
        self._camera.white_balance_red = self._camera.CONTROL_AUTO
        self._camera.white_balance_green = self._camera.CONTROL_AUTO
        self._camera.white_balance_blue = self._camera.CONTROL_AUTO
        
        # set BaxterImage struct
        self._image = RR.RobotRaconteurNode.s.NewStructure("BaxterCamera_interface.BaxterImage")
        self._image.width = self._image_header.width
        self._image.height = self._image_header.height
        self._image.step = self._image_header.step

        # Initialize ARtag detection
        self._aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
        self._arucoParams = aruco.DetectorParameters_create()
        self._markerSize = 0.06
        
    # open camera 
    def openCamera(self):
        if self._camera_open:
            return
        
        # start camera and subscription
        try:
            self._camera.open()
        except (OSError):
            print "Could not open camera.  Perhaps the other cameras are already open?"
            return
        
        # get camera intrinsic values and fill Robot Raconteur struct
        self._caminfo_sub = rospy.Subscriber("cameras/" + self._camera_name + "/camera_info", 
                                CameraInfo, self.set_CameraIntrinsics)
        # Suscriber to camera image
        print "Subscribing to", self._camera_name
        self._image_sub = rospy.Subscriber("cameras/" + self._camera_name + "/image", Image, self.set_imagedata) 
        self._camera_open = True
    
    
    def closeCamera(self):
        if not self._camera_open:
            return
    
        if self._image_sub:
            self._image_sub.unregister()
        self._camera.close()
        self._camera_open = False
    
    @property
    def camera_open(self):
        if self._camera_open:
            return 1
        else:
            return 0
    
    # subscriber function for camera image
    def set_imagedata(self,camdata):
        with self._lock:
            if camdata.data:
                self._image.data = numpy.frombuffer(camdata.data,dtype="u1")
        
        with self._imagestream_endpoints_lock:
            # Send to pipe endpoints
            for ep in self._imagestream_endpoints:
                dict_ep = self._imagestream_endpoints[ep]
                # Loop through indices in nested dict
                for ind in dict_ep:
                    # Attempt to send frame to connected endpoint
                    try:
                        pipe_ep=dict_ep[ind]
                        pipe_ep.SendPacket(self._image)
                    except:
                        # on error, assume pipe has been closed
                        self.ImageStream_pipeclosed(pipe_ep)
    
    def set_CameraIntrinsics(self, data):
        if (self._camera_intrinsics is None):
            print "Setting Camera Intrinsic Data"
            self._camera_intrinsics = RR.RobotRaconteurNode.s.NewStructure( 
                                        "BaxterCamera_interface.CameraIntrinsics" )
            K = list(data.K)
            K[2] -= data.roi.x_offset;
            K[5] -= data.roi.y_offset;
            self._camera_intrinsics.K = tuple(K)
            self._camera_intrinsics.D = tuple(data.D)
            self._caminfo_sub.unregister()

    # The following function is to set camera parameters manually
    def setCameraIntrinsics(self, data):
        if (self._camera_intrinsics is None):
            print "Setting Camera Intrinsic Data"
        else:
            print "Setting already exists. Overwriting now..."
        K = list(data.K)
        self._camera_intrinsics.K = tuple(K)
        self._camera_intrinsics.D = tuple(data.D)
        self._caminfo_sub.unregister()
            
    def getCurrentImage(self):
        with self._lock:    
            return self._image
    
    def getImageHeader(self):
        return self._image_header
    
    def getCameraIntrinsics(self):
        return self._camera_intrinsics
        
    ''' This is meant to only be called once at the initialization of the program'''
    def setResolution(self, mode, half_res):
        
        self._camera.resolution = self._camera.MODES[mode]
        
        # half resolution not always possible
        if (mode in [0,1,4] and half_res == 1):
            print 'Cannot do half-resolution at (1280,800), (960, 600), or (384,240)'
            half_res = 0
        self._camera.half_resolution = (half_res != 0)
        
        print 'Resolution set to: ', self._camera.resolution
        if (self._camera.half_resolution):
            print '**Displaying at half-resolution'
    
    def setExposure(self, exposure):
        if (exposure < 0 or exposure > 100 and exposure != self._camera.CONTROL_AUTO):
            print 'Exposure must be in [0, 100]'
            return
        self._camera.exposure = exposure
    
    def setGain(self, gain):
        if (gain < 0 or gain > 79 and gain != self._camera.CONTROL_AUTO):
            print 'Gain must be in [0, 79]'
            return
        self._camera.gain = gain
    
    def setWhiteBalance(self, red, green, blue):
        if (red < 0 or red > 4095 and red != self._camera.CONTROL_AUTO):
            print 'White Balance values must be in [0, 4095]'
            return
        self._camera.white_balance_red = red
        
        if (green < 0 or green > 4095 and green != self._camera.CONTROL_AUTO):
            print 'White Balance values must be in [0, 4095]'
            return
        self._camera.white_balance_green = green
        
        if (blue < 0 or blue > 4095 and blue != self._camera.CONTROL_AUTO):
            print 'White Balance values must be in [0, 4095]'
            return
        self._camera.white_balance_blue = blue
    
    def setFPS(self, fps):
        if (fps <= 0 or fps > 30):
            print 'fps must be positive and cannot exceed 30'
            return
        self._camera.fps = fps

    # Functions related to AR tags
    # Marker size
    def setMarkerSize(self, markerSize):
        with self._lock:
            self._markerSize = markerSize

    def getMarkerSize(self):
        with self._lock:
            markerSize = self._markerSize
            return markerSize

    # need modifications ################
    def ARtag_Detection(self):
        if not self.camera_open:
            self.openCamera()
        print "Detecting AR tags..."
        currentImage = self.getCurrentImage()
        imageData = currentImage.data
        # constructing BGR matrix
        Blist = [None] * currentImage.width * currentImage.height
        Glist = [None] * currentImage.width * currentImage.height
        Rlist = [None] * currentImage.width * currentImage.height
        i = 0
        while i < currentImage.step * currentImage.width * currentImage.height:
            Blist[i/currentImage.step] = imageData[i]
            i += 1
            Glist[i/currentImage.step] = imageData[i]
            i += 1
            Rlist[i/currentImage.step] = imageData[i]
            i += (currentImage.step - 2)
        # make those lists into opencv usable numpy array format and reshape them 
        Brow = numpy.array(Blist)
        Grow = numpy.array(Glist)
        Rrow = numpy.array(Rlist)
        B = numpy.reshape(Brow, (currentImage.height, currentImage.width))
        G = numpy.reshape(Grow, (currentImage.height, currentImage.width))
        R = numpy.reshape(Rrow, (currentImage.height, currentImage.width))
        # merge into a colored image then converted into grayscale
        trueImgData = cv2.merge((B,G,R))
        gray = cv2.cvtColor(trueImgData, cv2.COLOR_BGR2GRAY)
        # detect aruco tags
        corners, ids, rejected = aruco.detectMarkers(trueImgData, self._aruco_dict, parameters=self._arucoParams) 
        # debug information
        print(corners, ids) 
        # make sure tag are detected
        markerLength = self._markerSize # unit in meters
        if ids is not None:
            camparam = numpy.reshape(self._camera_intrinsics.K, (3, 3))
            rvec, tvec, _objpoints = aruco.estimatePoseSingleMarkers(corners, markerLength, camparam, self._camera_intrinsics.D) # For a single marker
            print(rvec, tvec)
            detectioninfo = RR.RobotRaconteurNode.s.NewStructure("BaxterCamera_interface.ARtagInfo")
            rvecs = []
            tvecs = []
            allids = []
            for rveci in rvec:
                x, y, z = rveci[0]
                rvecs.append(x)
                rvecs.append(y)
                rvecs.append(z)
            for tveci in tvec:
                x, y, z = tveci[0]
                tvecs.append(x)
                tvecs.append(y)
                tvecs.append(z)
            for id_i in ids:
                allids.append(id_i[0])
            detectioninfo.rvec = rvecs
            detectioninfo.tvec = tvecs
            detectioninfo.ids = allids
            return detectioninfo


    ######################################

    # pipe functions
    @property
    def ImageStream(self):
        return self._imagestream
    
    @ImageStream.setter
    def ImageStream(self, value):
        self._imagestream = value
        # Set the PipeConnecCallback to ImageStream_pipeconnect that will
        # called when a PipeEndpoint connects
        value.PipeConnectCallback = self.ImageStream_pipeconnect
    
    def ImageStream_pipeconnect(self, pipe_ep):
        # Lock the _imagestream_endpoints deictionary and place he pipe_ep in 
        # a nested dict that is indexed by the endpoint of the client and the 
        # index of the pipe
        with self._imagestream_endpoints_lock:
            # if there is not an enry for this client endpoint, add it
            if (not pipe_ep.Endpoint in self._imagestream_endpoints):
                self._imagestream_endpoints[pipe_ep.Endpoint] = dict()
            
            # Add pipe_ep to the correct dictionary given the endpoint + index
            dict_ep = self._imagestream_endpoints[pipe_ep.Endpoint]
            dict_ep[pipe_ep.Index] = pipe_ep
            pipe_ep.PipeEndpointClosedCallback = self.ImageStream_pipeclosed
    
    def ImageStream_pipeclosed(self, pipe_ep):
        with self._imagestream_endpoints_lock:
            try:
                dict_ep = self._imagestream_endpoints[pipe_ep.Endpoint]
                del(dict_ep[pipe_ep.Index])
            except:
                traceback.print_exc()
    

def main(argv):
    # parse command line arguments
    parser = argparse.ArgumentParser(description='Initialize Baxter Camera.')
    parser.add_argument('camera_name', metavar='camera_name',
			   choices=['left_hand_camera', 'right_hand_camera', 'head_camera'], 
			   help='name of the camera to connect to')
    parser.add_argument('--mode', type=int, default = 5,
			   choices=range(0,6),
		           help='mode of camera resolution')
    parser.add_argument('--half_res', type=int, default = 0, 
			   choices=range(0,2),
		           help='Show in half resolution [0 / 1]')
    parser.add_argument('--port', type=int, default = 0,
                    help='TCP port to host service on (will auto-generate if not specified)')
    args = parser.parse_args(argv)

    #Enable numpy
    RR.RobotRaconteurNode.s.UseNumPy=True

    #Set the RobotRaconteur Node name
    RR.RobotRaconteurNode.s.NodeName="BaxterCameraServer"

    #Create transport, register it, and start the server
    print "Registering Transport"
    t = RR.TcpTransport()
    t.EnableNodeAnnounce(RR.IPNodeDiscoveryFlags_NODE_LOCAL | 
        RR.IPNodeDiscoveryFlags_LINK_LOCAL | RR.IPNodeDiscoveryFlags_SITE_LOCAL)
    RR.RobotRaconteurNode.s.RegisterTransport(t)
    t.StartServer(args.port)
    port = args.port
    if (port == 0):
        port = t.GetListenPort()
    
    #Register the service type and the service
    print "Starting Service"
    RR.RobotRaconteurNode.s.RegisterServiceType(baxter_servicedef)
    
    #Initialize object
    baxter_obj = BaxterCamera_impl(args.camera_name, args.mode, args.half_res)
    
    RR.RobotRaconteurNode.s.RegisterService(args.camera_name, 
				"BaxterCamera_interface.BaxterCamera", baxter_obj)

    print "Service started, connect via"
    print "tcp://localhost:" + str(port) + "/BaxterCameraServer/" + args.camera_name
    raw_input("press enter to quit...\r\n")

    baxter_obj.closeCamera()

    # This must be here to prevent segfault
    RR.RobotRaconteurNode.s.Shutdown()

if __name__ == '__main__':
    main(sys.argv[1:])
