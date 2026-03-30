import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image
from cv_bridge import CvBridge
import cv2
import Jetson.GPIO as GPIO
import subprocess
import time
import threading

class SyncCameraNode(Node):
    def __init__(self):
        super().__init__('sync_camera_node')
        
        # 1. Setup GPIO
        self.trigger_pin = 7
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.trigger_pin, GPIO.OUT, initial=GPIO.LOW)
        
        # 2. Reset the camera to Free-Run mode for initialization
        self.get_logger().info("Resetting camera to freerun mode for initialization...")
        subprocess.run(['v4l2-ctl', '-d', '/dev/video0', '--set-ctrl=exposure_dynamic_framerate=0'])
        time.sleep(0.5) 
        
        # 3. Open OpenCV Capture
        self.get_logger().info("Starting OpenCV Capture...")
        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 800)
        
        # CRITICAL: Tell V4L2 to only keep the newest frame, discard old buffered ones
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        
        # Prime the pump
        self.cap.read() 
        
        # 4. Lock into External Trigger mode
        self.get_logger().info("Locking camera into External Trigger mode...")
        subprocess.run(['v4l2-ctl', '-d', '/dev/video0', '--set-ctrl=auto_exposure=1'])
        subprocess.run(['v4l2-ctl', '-d', '/dev/video0', '--set-ctrl=exposure_time_absolute=50'])
        subprocess.run(['v4l2-ctl', '-d', '/dev/video0', '--set-ctrl=exposure_dynamic_framerate=1'])
        
        self.bridge = CvBridge()
        
        # 5. Setup Threading variables
        self.latest_frame = None
        self.new_frame_event = threading.Event()
        self.is_running = True
        
        # Start the background capture thread
        self.capture_thread = threading.Thread(target=self.capture_loop, daemon=True)
        self.capture_thread.start()
        
        # 6. Setup ROS Publishers and Subscribers
        self.image_pub = self.create_publisher(Image, '/trig/image_raw', 10)
        self.lidar_sub = self.create_subscription(PointCloud2, '/unilidar/cloud', self.lidar_callback, 10)
        
        self.get_logger().info("Node Ready. Waiting for point clouds...")

    def capture_loop(self):
        """Background thread that constantly drains the OpenCV buffer."""
        while self.is_running and rclpy.ok():
            ret, frame = self.cap.read()
            if ret:
                self.latest_frame = frame
                self.new_frame_event.set() # Signal that a new frame arrived

    def pulse_trigger(self):
        """1ms hardware pulse"""
        GPIO.output(self.trigger_pin, GPIO.HIGH)
        time.sleep(0.001)
        GPIO.output(self.trigger_pin, GPIO.LOW)

    def lidar_callback(self, lidar_msg):
            # 1. Clear the event flag BEFORE triggering
            self.new_frame_event.clear()
            
            # 2. Capture the exact trigger time
            trigger_time = self.get_clock().now().to_msg()
            
            # 3. Fire the hardware trigger
            self.pulse_trigger()
            
            # 4. Wait for the background thread to catch the frame
            if self.new_frame_event.wait(timeout=0.3):
                # We got a fresh frame!
                frame = self.latest_frame
                
                # THE OPTIMIZATION: No cvtColor conversion! 
                # Pass the raw OpenCV frame and label it as "bgr8"
                img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                
                img_msg.header.stamp = trigger_time
                img_msg.header.frame_id = "camera_frame" 
                
                self.image_pub.publish(img_msg)
            else:
                self.get_logger().warn("Timeout: Camera took longer than 300ms to respond!")

                
    def destroy_node(self):
        self.is_running = False
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SyncCameraNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        node.cap.release()
        GPIO.cleanup()
        rclpy.shutdown()

if __name__ == '__main__':
    main()