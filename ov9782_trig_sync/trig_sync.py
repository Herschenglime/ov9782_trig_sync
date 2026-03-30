import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image
from cv_bridge import CvBridge
import cv2
import Jetson.GPIO as GPIO
import subprocess
import time

class SyncCameraNode(Node):
    def __init__(self):
        super().__init__('sync_camera_node')
        
        # 1. Setup GPIO
        self.trigger_pin = 7
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.trigger_pin, GPIO.OUT, initial=GPIO.LOW)
        
        # 2. Reset the camera to Free-Run mode so OpenCV doesn't hang on boot
        self.get_logger().info("Resetting camera to freerun mode for initialization...")
        subprocess.run(['v4l2-ctl', '-d', '/dev/video0', '--set-ctrl=exposure_dynamic_framerate=0'])
        time.sleep(0.5) # Give the hardware a split second to apply the change
        
        # 3. Open OpenCV Capture in MJPG mode FIRST
        self.get_logger().info("Starting OpenCV Capture...")
        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 800)
        
        # Prime the pump: Read a dummy frame to initialize OpenCV buffers
        self.cap.read() 
        
        # 4. NOW Configure V4L2 Hardware Trigger via subprocess
        self.get_logger().info("Locking camera into External Trigger mode...")
        subprocess.run(['v4l2-ctl', '-d', '/dev/video0', '--set-ctrl=auto_exposure=1'])
        subprocess.run(['v4l2-ctl', '-d', '/dev/video0', '--set-ctrl=exposure_time_absolute=50'])
        subprocess.run(['v4l2-ctl', '-d', '/dev/video0', '--set-ctrl=exposure_dynamic_framerate=1'])
        
        self.bridge = CvBridge()
        
        # 5. Setup ROS Publishers and Subscribers
        self.image_pub = self.create_publisher(Image, '/trig/image_raw', 10)
        self.lidar_sub = self.create_subscription(PointCloud2, '/unilidar/cloud', self.lidar_callback, 10)
        
        self.get_logger().info("Node Ready. Waiting for point clouds to trigger camera...")

    def pulse_trigger(self):
        """1ms hardware pulse"""
        GPIO.output(self.trigger_pin, GPIO.HIGH)
        time.sleep(0.001)
        GPIO.output(self.trigger_pin, GPIO.LOW)

    def lidar_callback(self, lidar_msg):
        self.get_logger().info("Entering callback")
        # Capture the exact time the callback was hit for accurate sensor fusion
        trigger_time = self.get_clock().now().to_msg()
        
        # 1. Instantly fire the hardware trigger
        self.pulse_trigger()
        
        # 2. Grab the frame
        ret, frame = self.cap.read()
        
        if ret:
            # 3. Convert OpenCV's native BGR to standard RGB
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            
            # 4. Convert to ROS Image
            img_msg = self.bridge.cv2_to_imgmsg(frame_rgb, encoding="rgb8")
            
            # 5. Apply the accurate trigger timestamp
            # img_msg.header.stamp = trigger_time
            # img_msg.header.frame_id = "camera_frame" 
            
            # 6. Publish
            self.image_pub.publish(img_msg)
            self.get_logger().info("Successfully triggered!")
        else:
            self.get_logger().warn("Triggered, but dropped a frame!")

def main(args=None):
    rclpy.init(args=args)
    node = SyncCameraNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        GPIO.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
