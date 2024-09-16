#!/usr/bin/env python3
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib
Gst.init(None)

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from cv_bridge import CvBridge
import cv2
import numpy as np
import socket
import struct
import sys
import traceback

class VideoReceiver(Node):
    def __init__(self):
        super().__init__('video_receiver')
        self.publisher_ = self.create_publisher(Image, 'camera/image_raw', 10)
        self.timestamp_publisher_ = self.create_publisher(Float64, 'camera/timestamp', 10)
        self.bridge = CvBridge()
        self.create_pipeline()
        
        # Create UDP socket for receiving video data and timestamps
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_socket.bind(('0.0.0.0', 5000))  # Bind to all interfaces, port 5000
        self.udp_socket.setblocking(False)

    def create_pipeline(self):
        pipeline_str = ("appsrc name=src ! "
                        "rtph264depay ! decodebin ! videoconvert ! "
                        "video/x-raw,format=BGR ! appsink name=sink")
        self.pipeline = Gst.parse_launch(pipeline_str)
        self.appsrc = self.pipeline.get_by_name("src")
        self.sink = self.pipeline.get_by_name("sink")
        
        if not self.appsrc or not self.sink:
            raise ValueError("Could not find 'src' or 'sink' element in the pipeline")
        
        self.sink.set_property("emit-signals", True)
        self.sink.connect("new-sample", self.on_new_sample)

    def on_new_sample(self, sink):
        sample = sink.emit("pull-sample")
        if not sample:
            return Gst.FlowReturn.OK
        
        buffer = sample.get_buffer()
        caps = sample.get_caps()
        structure = caps.get_structure(0)
        height = structure.get_value('height')
        width = structure.get_value('width')
        
        buffer_size = buffer.get_size()
        array = np.ndarray(shape=(height, width, 3), dtype=np.uint8, buffer=buffer.extract_dup(0, buffer_size))
        
        # Convert to ROS Image message
        ros_image = self.bridge.cv2_to_imgmsg(array, "bgr8")
        self.publisher_.publish(ros_image)
        
        return Gst.FlowReturn.OK

    def run(self):
        ret = self.pipeline.set_state(Gst.State.PLAYING)
        if ret == Gst.StateChangeReturn.FAILURE:
            self.get_logger().error("Failed to set pipeline to playing state")
            return
        
        self.get_logger().info("Pipeline is playing")
        
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0)
            
            try:
                data, addr = self.udp_socket.recvfrom(65536)  # Adjust buffer size if needed
                timestamp = struct.unpack('d', data[:8])[0]
                video_data = data[8:]
                
                # Push video data to the pipeline
                buffer = Gst.Buffer.new_wrapped(video_data)
                self.appsrc.emit("push-buffer", buffer)
                
                # Publish timestamp
                self.timestamp_publisher_.publish(Float64(data=timestamp))
            except BlockingIOError:
                pass  # No data available
            except Exception as e:
                self.get_logger().error(f"Error processing data: {str(e)}")
        
        self.pipeline.set_state(Gst.State.NULL)
        self.udp_socket.close()

def main(args=None):
    rclpy.init(args=args)
    video_receiver = VideoReceiver()
    try:
        video_receiver.run()
    except Exception as e:
        print(f"Error in main: {e}", file=sys.stderr)
        traceback.print_exc()
    finally:
        video_receiver.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()