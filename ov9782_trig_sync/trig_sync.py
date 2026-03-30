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
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        
        self.cap.read() # Prime the pump
        
        # 4. Lock into External Trigger mode
        self.get_logger().info("Locking camera into External Trigger mode...")
        subprocess.run(['v4l2-ctl', '-d', '/dev/video0', '--set-ctrl=auto_exposure=1'])
        subprocess.run(['v4l2-ctl', '-d', '/dev/video0', '--set-ctrl=exposure_time_absolute=100'])
        subprocess.run(['v4l2-ctl', '-d', '/dev/video0', '--set-ctrl=exposure_dynamic_framerate=1'])
        
        self.bridge = CvBridge()
        
        # 5. Setup Threading variables
        self.is_running = True
        self.latest_trigger_time = None
        self.trigger_lock = threading.Lock()
        
        # 6. Setup ROS Publishers and Subscribers
        self.image_pub = self.create_publisher(Image, '/trig/image_raw', 10)
        self.lidar_sub = self.create_subscription(PointCloud2, '/unilidar/cloud', self.lidar_callback, 10)
        
        # Start the background capture thread LAST so everything is ready
        self.capture_thread = threading.Thread(target=self.capture_loop, daemon=True)
        self.capture_thread.start()
        
        self.get_logger().info("Node Ready. Waiting for point clouds...")

    def capture_loop(self):
        """Background thread that constantly waits for frames and publishes them."""
        while self.is_running and rclpy.ok():
            # This fundamentally blocks until the triggered frame arrives from the USB bus
            ret, frame = self.cap.read() 
            
            if ret:
                # Safely grab the timestamp of the LiDAR message that triggered this frame
                with self.trigger_lock:
                    if self.latest_trigger_time is None:
                        continue # Ignore stray frames before the first LiDAR msg arrives
                    publish_time = self.latest_trigger_time
                
                # Convert and Publish immediately
                img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                img_msg.header.stamp = publish_time
                img_msg.header.frame_id = "camera_frame" 
                
                self.image_pub.publish(img_msg)
            else:
                self.get_logger().warn("OpenCV read failed or timed out internally!")

    def pulse_trigger(self):
        """1ms hardware pulse"""
        GPIO.output(self.trigger_pin, GPIO.HIGH)
        time.sleep(0.001)
        GPIO.output(self.trigger_pin, GPIO.LOW)

    def lidar_callback(self, lidar_msg):
        # 1. Update the timestamp for the background thread to use
        with self.trigger_lock:
            self.latest_trigger_time = self.get_clock().now().to_msg()
            
        # 2. Instantly fire the hardware trigger and EXIT the callback!
        self.pulse_trigger()

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