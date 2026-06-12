import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2

import tty
import termios
import sys
import select
import os

SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=2,
)

#data filepaths
REALSENSE_RGB_DIR   = os.path.expanduser('~/JIFREED_REU/REU_ws/datasets/realsense_rgb_data')
REALSENSE_DEPTH_DIR = os.path.expanduser('~/JIFREED_REU/REU_ws/datasets/realsense_depth_data')
ARDUCAM_RGB_DIR     = os.path.expanduser('~/JIFREED_REU/REU_ws/datasets/arducam_rgb_data')

#keybinds
SAVE    = 's'
DISCARD = 'd'
QUIT    = '\x03'  # Ctrl+C

def _next_index(*dirs: str) -> int:
    """
    Scans all save directories and returns last_saved_index + 1.
    Returns 0 if no files exist yet. This means resuming a session
    never overwrites existing data.
    """
    max_idx = -1
    for d in dirs:
        if not os.path.isdir(d):
            continue
        for fname in os.listdir(d):
            name, _ = os.path.splitext(fname)
            # Filenames are camera_datatype_NUM — index is the last token
            parts = name.rsplit('_', 1)
            if len(parts) == 2 and parts[1].isdigit():
                max_idx = max(max_idx, int(parts[1]))
    return max_idx + 1


class dataCollectionNode(Node):
    def __init__(self):
        super().__init__('data_collection_node')

        #latest frames cached for capture on keypress
        self._latest_rs_color = None
        self._latest_rs_depth = None
        self._latest_arducam  = None

        #initialize cv bridge
        self._bridge = CvBridge()

        #vision topic subscribers
        self._realsense_rgb_sub   = self.create_subscription(Image, '/vision/realsense_color', self._rs_color_cb,  SENSOR_QOS)
        self._realsense_depth_sub = self.create_subscription(Image, '/vision/realsense_depth', self._rs_depth_cb,  SENSOR_QOS)
        self._arducam_rgb_sub     = self.create_subscription(Image, '/vision/arducam_raw',     self._arducam_cb,   SENSOR_QOS)

        #prepare data collection directories and starting index
        for d in [REALSENSE_RGB_DIR, REALSENSE_DEPTH_DIR, ARDUCAM_RGB_DIR]:
            os.makedirs(d, exist_ok=True)
        self._save_index = _next_index(REALSENSE_RGB_DIR, REALSENSE_DEPTH_DIR, ARDUCAM_RGB_DIR)

        self.get_logger().info(f'Data collection node ready. Next save index: {self._save_index}')

    # --- Vision callbacks: display live video and cache latest frame ---

    def _rs_color_cb(self, msg: Image):
        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self._latest_rs_color = frame
        cv2.imshow('RealSense Color', frame)
        cv2.waitKey(1)

    def _rs_depth_cb(self, msg: Image):
        depth = self._bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
        self._latest_rs_depth = depth
        #normalize depth to 8-bit colormap for display (raw 16UC1 appears black)
        depth_display  = cv2.normalize(depth, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        depth_colormap = cv2.applyColorMap(depth_display, cv2.COLORMAP_JET)
        cv2.imshow('RealSense Depth', depth_colormap)
        cv2.waitKey(1)

    def _arducam_cb(self, msg: Image):
        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self._latest_arducam = frame
        cv2.imshow('Arducam', frame)
        cv2.waitKey(1)

    #Capture: snapshot all currently displayed frames ---

    def capture(self):
        """Snapshots the latest cached frame from each stream simultaneously."""
        missing = []
        if self._latest_rs_color is None: missing.append('RealSense color')
        if self._latest_rs_depth is None: missing.append('RealSense depth')
        if self._latest_arducam  is None: missing.append('Arducam')
        if missing:
            print(f'  [!] Missing frames from: {", ".join(missing)}')
            return None

        #return copies so the cache can keep updating without affecting pending
        return {
            'rs_color': self._latest_rs_color.copy(),
            'rs_depth': self._latest_rs_depth.copy(),
            'arducam':  self._latest_arducam.copy(),
        }

    #save data frames
    def save_all(self, frames: dict):
        idx = self._save_index

        paths = {
            'rs_color': os.path.join(REALSENSE_RGB_DIR,   f'realsense_color_{idx}.jpg'),
            'rs_depth': os.path.join(REALSENSE_DEPTH_DIR, f'realsense_depth_{idx}.png'), 
            'arducam':  os.path.join(ARDUCAM_RGB_DIR,     f'arducam_color_{idx}.jpg'),
        }

        cv2.imwrite(paths['rs_color'], frames['rs_color'])
        cv2.imwrite(paths['rs_depth'], frames['rs_depth'])
        cv2.imwrite(paths['arducam'],  frames['arducam'])

        self._save_index += 1

        print(f'  [✓] Saved capture #{idx}:')
        for p in paths.values():
            print(f'        {p}')

    def destroy_node(self) -> None:
        self.get_logger().info('Destroying data_collection_node')
        cv2.destroyAllWindows()
        super().destroy_node()


# ---------------------------------------------------------------------------

def get_key(settings, timeout=0.1):
    """Reads a single keypress from the terminal without requiring Enter."""
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)

    if rlist:
        key = sys.stdin.read(1)
        # If it's an escape character, read the next two bytes for arrow keys
        if key == '\x1b':
            key += sys.stdin.read(2)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main(args=None):
    rclpy.init(args=args)
    node = dataCollectionNode()

    # Save original terminal settings so we don't break the user's console
    settings = termios.tcgetattr(sys.stdin)

    print("\n" + "="*40)
    print("  S — capture and save all frames")
    print("  Ctrl+C — exit")
    print("="*40 + "\n")

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            key = get_key(settings, timeout=0.1)

            if key == SAVE:
                frames = node.capture()
                if frames is not None:
                    node.save_all(frames)

            elif key == QUIT:
                break

    except Exception as e:
        node.get_logger().error(f'Error: {e}')

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()