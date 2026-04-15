import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import Jetson.GPIO as GPIO
import subprocess
import time
import threading
import copy
from collections import deque
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory

class SyncCameraNode(Node):
    def __init__(self):
        super().__init__('sync_camera_node')

        default_info_path = (
            Path(get_package_share_directory('ov9782_trig_sync'))
            / 'config'
            / 'ov9782_info.yaml'
        )
        self.declare_parameter('camera_info_path', str(default_info_path))
        self.declare_parameter('camera_frame_id', 'camera_frame')
        self.camera_frame_id = self.get_parameter('camera_frame_id').value
        self.camera_info_template = self.load_camera_info(
            self.get_parameter('camera_info_path').value
        )
        
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
        subprocess.run(['v4l2-ctl', '-d', '/dev/video0', '--set-ctrl=exposure_time_absolute=10'])
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
        self.camera_info_pub = self.create_publisher(CameraInfo, '/trig/camera_info', 10)
        self.lidar_sub = self.create_subscription(PointCloud2, '/unilidar/cloud', self.lidar_callback, 10)
        self.stats_timer = self.create_timer(1.0, self.log_stats)
        
        # Start the background capture thread LAST so everything is ready
        self.capture_thread = threading.Thread(target=self.capture_loop, daemon=True)
        self.capture_thread.start()
        
        self.get_logger().info("Node Ready. Waiting for point clouds...")

    def load_camera_info(self, camera_info_path):
        camera_info_msg = CameraInfo()
        path = Path(camera_info_path)

        if not path.is_file():
            self.get_logger().warn(
                f"Camera info file not found at '{path}'. Publishing default (empty) CameraInfo."
            )
            return camera_info_msg

        try:
            with path.open('r', encoding='utf-8') as stream:
                data = yaml.safe_load(stream) or {}

            camera_info_msg.width = int(data.get('image_width', 0))
            camera_info_msg.height = int(data.get('image_height', 0))
            camera_info_msg.distortion_model = str(data.get('distortion_model', ''))
            camera_info_msg.d = [
                float(value) for value in data.get('distortion_coefficients', {}).get('data', [])
            ]
            camera_info_msg.k = [float(value) for value in data.get('camera_matrix', {}).get('data', [0.0] * 9)]
            camera_info_msg.r = [
                float(value) for value in data.get('rectification_matrix', {}).get('data', [0.0] * 9)
            ]
            camera_info_msg.p = [
                float(value) for value in data.get('projection_matrix', {}).get('data', [0.0] * 12)
            ]

            self.get_logger().info(f"Loaded CameraInfo from '{path}'")
        except Exception as exc:
            self.get_logger().error(
                f"Failed to parse camera info yaml '{path}': {exc}. Publishing default (empty) CameraInfo."
            )

        return camera_info_msg

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
                    img_msg.header.frame_id = self.camera_frame_id

                    self.image_pub.publish(img_msg)

                    camera_info_msg = copy.deepcopy(self.camera_info_template)
                    camera_info_msg.header.stamp = publish_time
                    camera_info_msg.header.frame_id = self.camera_frame_id
                    self.camera_info_pub.publish(camera_info_msg)

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