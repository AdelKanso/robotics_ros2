#!/usr/bin/env python3
import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, Int32


class LapMarkerDetector(Node):
    def __init__(self):
        super().__init__('lap_marker_detector')

        # -------- Topics --------
        self.image_topic = '/camera/image_raw/compressed'
        self.pub_seen = self.create_publisher(Bool, '/lap_marker/seen', 10)
        self.pub_id = self.create_publisher(Int32, '/lap_marker/id', 10)

        self.sub = self.create_subscription(
            CompressedImage,
            self.image_topic,
            self.image_callback,
            10
        )

        self.bridge = CvBridge()

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_6X6_50
        )
        self.aruco_params = cv2.aruco.DetectorParameters()

        if hasattr(cv2.aruco, "ArucoDetector"):
            self.detector = cv2.aruco.ArucoDetector(
                self.aruco_dict, self.aruco_params
            )
        else:
            self.detector = None

        self.last_detection_time = 0.0
        self.min_interval = 2.0  

        self.get_logger().info("Lap marker detector started (DICT_6X6_50).")

    def image_callback(self, msg):
        # Decode compressed image
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if frame is None:
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if self.detector is not None:
            corners, ids, _ = self.detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray, self.aruco_dict, parameters=self.aruco_params
            )

        detected = False
        marker_id = -1

        if ids is not None and len(ids) > 0:
            detected = True
            marker_id = int(ids[0][0])

            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            cv2.putText(
                frame,
                f"LAP MARKER DETECTED (ID {marker_id})",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (0, 255, 0),
                2
            )

        now = self.get_clock().now().nanoseconds / 1e9
        if detected and (now - self.last_detection_time) > self.min_interval:
            self.last_detection_time = now

            msg_seen = Bool()
            msg_seen.data = True
            self.pub_seen.publish(msg_seen)

            msg_id = Int32()
            msg_id.data = marker_id
            self.pub_id.publish(msg_id)

            self.get_logger().info(
                f"Lap marker detected → ID {marker_id}"
            )

        cv2.imshow("Lap Marker Detection (DICT_6X6_50)", frame)
        cv2.waitKey(1)


def main():
    rclpy.init()
    node = LapMarkerDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()