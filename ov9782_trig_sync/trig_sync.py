import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image
from cv_bridge import CvBridge
import cv2
import Jetson.GPIO as GPIO
import subprocess
import time
import threading
from collections import deque

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
        subprocess.run(['v4l2-ctl', '-d', '/dev/video0', '--set-ctrl=exposure_time_absolute=50'])
        subprocess.run(['v4l2-ctl', '-d', '/dev/video0', '--set-ctrl=exposure_dynamic_framerate=1'])
        
        self.bridge = CvBridge()
        
        # 5. Setup Threading variables
        self.is_running = True
        self.max_pending_triggers = 32
        self.pending_triggers = deque()
        self.trigger_lock = threading.Lock()
        self.last_warn_time = 0.0

        # Counters to monitor where rate mismatch occurs
        self.lidar_count = 0
        self.trigger_count = 0
        self.frame_count = 0
        self.publish_count = 0
        self.read_fail_count = 0
        self.stray_frame_count = 0
        self.dropped_trigger_count = 0
        self.bad_lidar_stamp_count = 0

        # Previous counters used to compute per-second rates
        self.prev_lidar_count = 0
        self.prev_trigger_count = 0
        self.prev_frame_count = 0
        self.prev_publish_count = 0
        
        # 6. Setup ROS Publishers and Subscribers
        self.image_pub = self.create_publisher(Image, '/trig/image_raw', 10)
        self.lidar_sub = self.create_subscription(PointCloud2, '/unilidar/cloud', self.lidar_callback, 10)
        self.stats_timer = self.create_timer(1.0, self.log_stats)
        
        # Start the background capture thread LAST so everything is ready
        self.capture_thread = threading.Thread(target=self.capture_loop, daemon=True)
        self.capture_thread.start()
        
        self.get_logger().info("Node Ready. Waiting for point clouds...")

    def capture_loop(self):
        """Background thread that constantly waits for frames and publishes them."""
        while self.is_running and rclpy.ok():
            try:
                # This fundamentally blocks until the triggered frame arrives from the USB bus
                ret, frame = self.cap.read()

                if ret:
                    self.frame_count += 1

                    # Pop exactly one trigger timestamp for exactly one captured frame.
                    with self.trigger_lock:
                        if not self.pending_triggers:
                            self.stray_frame_count += 1
                            continue
                        publish_time = self.pending_triggers.popleft()

                    # Convert and Publish immediately
                    img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                    img_msg.header.stamp = publish_time
                    img_msg.header.frame_id = "camera_frame"

                    self.image_pub.publish(img_msg)
                    self.publish_count += 1
                else:
                    self.read_fail_count += 1
                    now = time.monotonic()
                    if now - self.last_warn_time > 1.0:
                        self.get_logger().warn("OpenCV read failed or timed out internally!")
                        self.last_warn_time = now
            except Exception as exc:
                self.get_logger().error(f"capture_loop exception: {exc}")
                time.sleep(0.01)

    def pulse_trigger(self):
        """1ms hardware pulse"""
        GPIO.output(self.trigger_pin, GPIO.HIGH)
        time.sleep(0.001)
        GPIO.output(self.trigger_pin, GPIO.LOW)

    def lidar_callback(self, lidar_msg):
        self.lidar_count += 1

        stamp = lidar_msg.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            self.bad_lidar_stamp_count += 1
            stamp = self.get_clock().now().to_msg()

        # Keep one timestamp entry per trigger so no LiDAR event is overwritten.
        with self.trigger_lock:
            if len(self.pending_triggers) >= self.max_pending_triggers:
                self.pending_triggers.popleft()
                self.dropped_trigger_count += 1
            self.pending_triggers.append(stamp)
            
        # Instantly fire the hardware trigger and exit the callback.
        self.pulse_trigger()
        self.trigger_count += 1

    def log_stats(self):
        lidar_rate = self.lidar_count - self.prev_lidar_count
        trigger_rate = self.trigger_count - self.prev_trigger_count
        frame_rate = self.frame_count - self.prev_frame_count
        publish_rate = self.publish_count - self.prev_publish_count

        with self.trigger_lock:
            pending = len(self.pending_triggers)

        self.get_logger().info(
            "rates Hz lidar=%d trigger=%d frame=%d publish=%d | pending=%d dropped_trigger=%d stray_frame=%d read_fail=%d bad_lidar_stamp=%d"
            % (
                lidar_rate,
                trigger_rate,
                frame_rate,
                publish_rate,
                pending,
                self.dropped_trigger_count,
                self.stray_frame_count,
                self.read_fail_count,
                self.bad_lidar_stamp_count,
            )
        )

        self.prev_lidar_count = self.lidar_count
        self.prev_trigger_count = self.trigger_count
        self.prev_frame_count = self.frame_count
        self.prev_publish_count = self.publish_count

    def destroy_node(self):
        self.is_running = False
        if self.capture_thread.is_alive():
            self.capture_thread.join(timeout=1.0)
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