#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import gi
import numpy as np
from sensor_msgs.msg import Image #new This line imports the ROS 2 message type 
from cv_bridge import CvBridge  #new converting between ROS Image messages and OpenCV images (numpy arrays).

gi.require_version('Gst', '1.0')
from gi.repository import Gst

class Controller(Node):
    """BlueRov video capture class constructor

    Attributes:
        port (int): Video UDP port
        video_codec (string): Source h264 parser
        video_decode (string): Transform YUV (12bits) to BGR (24bits)
        video_pipe (object): GStreamer top-level pipeline
        video_sink (object): Gstreamer sink element
        video_sink_conf (string): Sink configuration
        video_source (string): Udp source ip and port
    """

    g   = 9.81      # m.s^-2 gravitational acceleration 
    p0  = 103425    # Surface pressure in Pascal
    rho = 1000      # kg/m^3  water density

    def __init__(self):
        super().__init__("video")

        self.declare_parameter("port", 5600) 

        self.port               = self.get_parameter("port").value
        self._frame             = None
        self.video_source       = 'udpsrc port={}'.format(self.port)
        self.video_codec        = '! application/x-rtp, payload=96 ! rtph264depay ! h264parse ! avdec_h264'
        self.video_decode       = '! decodebin ! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert'
        self.video_sink_conf    = '! appsink emit-signals=true sync=false max-buffers=2 drop=true'

        self.video_pipe         = None
        self.video_sink         = None

        # font
        self.font               = cv2.FONT_HERSHEY_PLAIN

        Gst.init() 

        # Initialize CvBridge
        self.bridge = CvBridge() # initializes an instance of CvBridge and assigns it to self.bridge. 

        # Create a publisher for the image
        self.image_publisher = self.create_publisher(Image, 'camera/image', 10) #image_publisher: publisher name
        #publishes messages of type Image to the topic 'bluerov2/camera/image'.(10): the queue size of the publisher.

        self.run()

        # Start update loop
        self.create_timer(0.033, self.update)    #0.01 milliseconds update 100 Hz / 60 Hz 0.0167 now 30 hz 

    def start_gst(self, config=None):
        """ Start gstreamer pipeline and sink
        Pipeline description list e.g:
            [
                'videotestsrc ! decodebin', \
                '! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert',
                '! appsink'
            ]

        Args:
            config (list, optional): Gstreamer pileline description list
        """

        if not config:
            config = \
                [
                    'videotestsrc ! decodebin',
                    '! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert',
                    '! appsink'
                ]

        command = ' '.join(config)
        self.video_pipe = Gst.parse_launch(command)
        self.video_pipe.set_state(Gst.State.PLAYING)
        self.video_sink = self.video_pipe.get_by_name('appsink0')

    @staticmethod
    def gst_to_opencv(sample):
        """Transform byte array into np array

        Args:
            sample (TYPE): Description

        Returns:
            TYPE: Description
        """
        buf = sample.get_buffer()
        caps = sample.get_caps()
        array = np.ndarray(
            (
                caps.get_structure(0).get_value('height'),
                caps.get_structure(0).get_value('width'),
                3
            ),
            buffer=buf.extract_dup(0, buf.get_size()), dtype=np.uint8)
        return array

    def frame(self):
        """ Get Frame

        Returns:
            iterable: bool and image frame, cap.read() output
        """
        return self._frame

    def frame_available(self):
        """Check if frame is available

        Returns:
            bool: true if frame is available
        """
        return type(self._frame) != type(None)

    def run(self):
        """ Get frame to update _frame
        """

        self.start_gst(
            [
                self.video_source,
                self.video_codec,
                self.video_decode,
                self.video_sink_conf
            ])

        self.video_sink.connect('new-sample', self.callback)

    def callback(self, sink):
        sample = sink.emit('pull-sample')
        new_frame = self.gst_to_opencv(sample)
        self._frame = new_frame

        return Gst.FlowReturn.OK
    
    def update(self):        
        if not self.frame_available():
            return

        frame = self.frame()
        width = int(1920/2)#1.5
        height = int(1080/2)#1.5
        dim = (width, height)
        img = cv2.resize(frame, dim, interpolation = cv2.INTER_AREA)   

        #self.draw_gui(img, width, height)        

        # Convert OpenCV image to ROS 2 Image message
        #This line converts the OpenCV image img (which is a numpy array) into a ROS 2 Image message (img_msg).
        #The encoding='bgr8' parameter specifies the color encoding of the image (BGR format with 8 bits per channel).
        img_msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        # Publish the image message
        #This line publishes the img_msg (which now contains the image data)
        # to the 'bluerov2/camera/image' topic using self.image_publisher.
        self.image_publisher.publish(img_msg)

        cv2.imshow('BlueROV2 Camera', img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.destroy_node()

    def draw_gui(self, img, width, height):        
        img = cv2.rectangle(img,(0, height-100),(520,height),(0,0,0),-1)
        

def main(args=None):
    rclpy.init(args=args)    
    node = Controller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
