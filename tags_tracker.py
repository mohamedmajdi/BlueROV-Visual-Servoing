import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from tag_msgs.msg import TagInfo, TagArray

import cv2
import numpy as np
import random

def get_color_from_id(tag_id):
    random.seed(tag_id)
    return tuple(random.randint(100, 255) for _ in range(3))

class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector')

        # ROS 2 setup
        self.subscription = self.create_subscription(Image, 'camera/image', self.image_callback, 10)
        self.publisher_ = self.create_publisher(TagArray, 'bluerov2/tracked_point', 10)
        self.desired_publisher = self.create_publisher(TagArray, 'bluerov2/desired_point', 10)
        self.bridge = CvBridge()

        # ArUco setup
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.window_created = False

        # Preprocessing
        self.clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        self.sharpen_kernel = np.array([
            [0, -1, 0],
            [-1, 5, -1],
            [0, -1, 0]
        ])

        self.scale_factor = 1.0
        self.target_ids = list(range(1, 10))  # Expected tag IDs from 1 to 9

        # Initialize tracked and desired tags
        self.tracked_tags = TagArray()
        for tag_id in range(1, 10):
            tag = TagInfo()
            tag.id = tag_id
            tag.x = -999.0
            tag.y = -999.0
            self.tracked_tags.tags.append(tag)

        self.mode = 'click'
        # self.mode = 'distance'

        self.desired_tags = TagArray()
        for tag_id in range(1, 10):
            tag = TagInfo()
            tag.id = tag_id
            tag.x = -999.0
            tag.y = -999.0
            self.desired_tags.tags.append(tag)

        if self.mode == 'distance':
            f = 75/0.11
            z = 0.75
            # center = [480,270]
            center = [320,240]
            cb = [20,15]
            c_values = {
                1: [11.5, 17.5],
                2: [28, 12],
                3: [31.25, 20.75],
                4: [25.25, 20.75],
                5: [20.25, 20.75],
                6: [19.25, 8.75],
                7: [14.25, 8.75],
                8: [8.25, 8.75],
                9: [19.75, 14.75]
            }
            pixel_centers = {}
            for i, c in c_values.items():
                pixel_tag = TagInfo()
                pixel_tag.id = i
                pixel_tag.x = center[0] + (c[0] - cb[0]) * 0.01 * (f / z)
                pixel_tag.y = center[1] + (c[1] - cb[1]) * 0.01 * (f / z)
                self.desired_tags.tags.append(pixel_tag)
                pixel_centers[i] = [
                    center[0] + (c[0] - cb[0]) * 0.01 * (f / z),
                    center[1] + (c[1] - cb[1]) * 0.01 * (f / z)
                ]
                


    def on_mouse(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDBLCLK:
            if any((tag.x != -999.0 and tag.y != -999.0) for tag in self.tracked_tags.tags):
                self.get_logger().info("Desired tags confirmed")
                self.desired_tags = self.tracked_tags
            else:
                self.get_logger().info("No detected tags")

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        vis_frame = frame.copy()
        overlay = vis_frame.copy()

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = self.clahe.apply(gray)
        sharpened = cv2.filter2D(gray, -1, self.sharpen_kernel)
        sharpened_up = cv2.resize(sharpened, (0, 0), fx=self.scale_factor, fy=self.scale_factor)

        corners, ids, _ = cv2.aruco.detectMarkers(sharpened_up, self.aruco_dict, parameters=self.aruco_params)

        detected_tags = {i: None for i in self.target_ids}
        top_right_tag1 = None
        bottom_left_tag2 = None

        if ids is not None:
            for i in range(len(ids)):
                tag_id = int(ids[i][0])
                if tag_id in detected_tags:
                    upscaled_corners = corners[i][0]

                    cX_up = np.mean(upscaled_corners[:, 0])
                    cY_up = np.mean(upscaled_corners[:, 1])

                    cX = int(cX_up / self.scale_factor)
                    cY = int(cY_up / self.scale_factor)

                    detected_tags[tag_id] = [cX, cY]

        tag_array = TagArray()
        for tag_id in self.target_ids:
            tag = TagInfo()
            tag.id = tag_id
            if detected_tags[tag_id] is not None:
                coords = detected_tags[tag_id]
                tag.x = float(coords[0])
                tag.y = float(coords[1])
            else:
                tag.x = -999.0
                tag.y = -999.0
            tag_array.tags.append(tag)

        self.tracked_tags = tag_array
        any_detected = any(coords is not None for coords in detected_tags.values())
        any_valid_desired = any(
            (tag.x != -999.0 and tag.y != -999.0) for tag in self.desired_tags.tags
        )

        if any_detected:
            self.publisher_.publish(tag_array)
            if any_valid_desired:
                self.desired_publisher.publish(self.desired_tags)

        # Draw the detected tags on the image
        if ids is not None:
            for i in range(len(ids)):
                tag_id = int(ids[i][0])
                upscaled_corners = corners[i][0]

                if tag_id == 1:
                    top_right_tag1 = upscaled_corners[1] / self.scale_factor
                elif tag_id == 2:
                    bottom_left_tag2 = upscaled_corners[3] / self.scale_factor    
                    
                    
                    # Extract corners
                    # top_left = upscaled_corners[0]
                    # top_right = upscaled_corners[1]
                    # bottom_right = upscaled_corners[2]
                    # bottom_left = upscaled_corners[3]

                    # # Compute width and length
                    # width = np.linalg.norm(top_right - top_left)
                    # length = np.linalg.norm(bottom_left - top_left)

                    # # Scale back to original image if needed
                    # width_orig = width / self.scale_factor
                    # length_orig = length / self.scale_factor

                    # self.get_logger().info(f"Aruco ID 2: Width = {width_orig:.2f} px, Length = {length_orig:.2f} px")


                if top_right_tag1 is not None and bottom_left_tag2 is not None:
                    center_x = (top_right_tag1[0] + bottom_left_tag2[0]) / 2
                    center_y = (top_right_tag1[1] + bottom_left_tag2[1]) / 2

                    cv2.circle(overlay, (int(center_x), int(center_y)), 6, (0, 0, 255), -1)

                cX_up = np.mean(upscaled_corners[:, 0])
                cY_up = np.mean(upscaled_corners[:, 1])

                cX = int(cX_up / self.scale_factor)
                cY = int(cY_up / self.scale_factor)

                scaled_corners = (upscaled_corners / self.scale_factor).astype(int)

                color = get_color_from_id(tag_id)

                cv2.polylines(overlay, [scaled_corners], isClosed=True, color=color, thickness=2)
                cv2.circle(overlay, (cX, cY), radius=6, color=color, thickness=-1)

                cv2.rectangle(overlay, (cX + 5, cY - 20), (cX + 55, cY - 2), (0, 0, 0), -1)
                cv2.putText(overlay, f"ID:{tag_id}", (cX + 8, cY - 6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        # Draw desired points
        for tag in self.desired_tags.tags:
            if tag.x != -999.0 and tag.y != -999.0:
                desired_color = (0, 255, 0)  # Green color for desired points
                cv2.circle(overlay, (int(tag.x), int(tag.y)), 8, desired_color, 2)
                cv2.putText(overlay, f"Desired ID:{tag.id}", (int(tag.x) + 10, int(tag.y) - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, desired_color, 2)

        # Blend overlay onto vis_frame
        alpha = 0.8
        cv2.addWeighted(overlay, alpha, vis_frame, 1 - alpha, 0, vis_frame)

        if not self.window_created:
            cv2.namedWindow("Tag Detection", cv2.WINDOW_NORMAL)
            if self.mode == 'click':
                cv2.setMouseCallback("Tag Detection", self.on_mouse)
            self.window_created = True
            cv2.resizeWindow("Tag Detection", vis_frame.shape[1], vis_frame.shape[0])

        cv2.imshow("Tag Detection", vis_frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

