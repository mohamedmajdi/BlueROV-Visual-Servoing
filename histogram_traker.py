import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
import imutils

from . import camera_parameters as cam

get_hsv = False
mouseX, mouseY = 0, 0

# Click detection callback function
def click_detect(event, x, y, flags, param):
    global get_hsv, mouseX, mouseY
    if event == cv2.EVENT_LBUTTONDOWN:
        get_hsv = True
    mouseX, mouseY = x, y

class BuoyTracker(Node):
    def __init__(self):
        super().__init__('buoy_tracker')

        # ROS2 subscribers & publishers
        self.subscription = self.create_subscription(
            Image,
            'camera/image',
            self.image_callback,
            10
        )
        self.publisher = self.create_publisher(Float64MultiArray, 'bluerov2/tracked_point', 10)
        self.publisher_desired = self.create_publisher(Float64MultiArray, 'bluerov2/desired_point', 10)

        # OpenCV Bridge
        self.bridge = CvBridge()

        # Initialize variables for tracking
        self.track_window = None
        self.roi_hist = None
        self.term_crit = None
        self.get_logger().info("Buoy Tracker Node Initialized.")

    def overlay_points(self, image, pt, r, g, b, text="", scale=1, offsetx=5, offsety=5):
        cv2.circle(image, (int(pt[0]), int(pt[1])), int(4 * scale + 1), (b, g, r), -1)
        position = (int(pt[0]) + offsetx, int(pt[1]) + offsety)
        cv2.putText(image, text, position, cv2.FONT_HERSHEY_SIMPLEX, scale, (b, g, r, 255), 1)

    def process_frame(self, image_np, tracker_type):
        global get_hsv, mouseX, mouseY
        current_point = None

        image_height, image_width, image_channels = image_np.shape
        # self.get_logger().info(f"Image dimensions: {image_height}x{image_width}x{image_channels}")

        desired_point = [image_width // 2, image_height // 2-200,1]

        if tracker_type == 'color':
             # Track the buoy using color segmentation
            hsv_img = cv2.cvtColor(image_np, cv2.COLOR_BGR2HSV)
            # lower_range = np.array([15, 200, 60])
            # upper_range = np.array([30, 255, 255])
            lower_range = np.array([0, 130, 150])
            upper_range = np.array([180, 200, 255])
            mask = cv2.inRange(hsv_img, lower_range, upper_range)
            non_zero_pixels = cv2.findNonZero(mask)

            if non_zero_pixels is not None and len(non_zero_pixels) > 0:
                color_image = cv2.bitwise_and(image_np, image_np, mask=mask)
                gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
                blurred = cv2.GaussianBlur(gray, (5, 5), 0)
                thresh = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY)[1]
                cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                cnts = imutils.grab_contours(cnts)


                if cnts:
                    buoy_points = []
                    #for c in cnts:
                    c = max(cnts, key=cv2.contourArea) # Use the largest contour
                    cv2.drawContours(image_np, [c], -1, (0, 255, 255), 2)  # <- ADD THIS LINE
                    area = cv2.contourArea(c)
                    rect = cv2.minAreaRect(c)  # (center(x, y), (width, height), angle)
                    box = cv2.boxPoints(rect)
                    box = np.intp(box)

                    

                    # Extract width and height from the rectangle
                    (width_px, height_px) = rect[1]

                    # Determine which dimension is the width
                    buoy_width_px = max(width_px, height_px)
                    # print("Buoy width in pixels:", buoy_width_px)
                    # Compute start and end points for horizontal line
                    (center_x, center_y), (width, height), angle = rect

                    # Determine the width (longer side)
                    line_length = max(width, height)
                    half_length = line_length / 2
                    pt1 = (int(center_x - half_length), int(center_y))
                    pt2 = (int(center_x + half_length), int(center_y))

                    # Draw the width line (always horizontal, through center)
                    cv2.line(image_np, pt1, pt2, (255, 0, 0), 2)  # blue line
                    # Calculate the distance to the buoy using the known size
                    #  the actual buoy is 0.3 meters wide
                    known_width_m = 0.3
                    focal_length = 455
                    distance = (known_width_m * focal_length) / buoy_width_px
                    print("Distance to buoy in meters:", distance)
                    M = cv2.moments(c)
                    if M["m00"] != 0:
                        cX = int(M["m10"] / M["m00"])
                        cY = int(M["m01"] / M["m00"])
                        buoy_points.append([cX, cY])

                    if buoy_points:
                        # Convert points to meters and calculate distances
                        buoy_points_meter = [cam.convertOnePoint2meter(pt) for pt in buoy_points]
                        distances = [np.linalg.norm(pt) for pt in buoy_points_meter]
                        nearest_buoy_index = np.argmin(distances)
                        current_point = buoy_points[nearest_buoy_index]

                # if cnts:
                #     buoy_points = []
                #     c = max(cnts, key=cv2.contourArea) # Use the largest contour
                #     cv2.drawContours(image_np, [c], -1, (0, 255, 255), 2)
                #     M = cv2.moments(c)
                #     if M["m00"] != 0:
                #         cX = int(M["m10"] / M["m00"])
                #         cY = int(M["m01"] / M["m00"])
                #         buoy_points.append([cX, cY])

                #     if buoy_points:
                #         # Convert points to meters and calculate distances
                #         buoy_points_meter = [cam.convertOnePoint2meter(pt) for pt in buoy_points]
                #         distances = [np.linalg.norm(pt) for pt in buoy_points_meter]
                #         nearest_buoy_index = np.argmin(distances)
                #         current_point = buoy_points[nearest_buoy_index]

        elif tracker_type == 'meanshift':
            if 'track_window' not in self.process_frame.__dict__:
                track_window = (int(image_width / 2), int(image_height / 2), 100, 100)
                roi = image_np[track_window[1]:track_window[1]+track_window[3], track_window[0]:track_window[0]+track_window[2]]
                hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
                mask = cv2.inRange(hsv_roi, np.array((120, 177, 190)), np.array((192, 233, 255)))
                roi_hist = cv2.calcHist([hsv_roi], [0], mask, [180], [0, 180])
                cv2.normalize(roi_hist, roi_hist, 0, 255, cv2.NORM_MINMAX)
                term_crit = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 1)
                self.process_frame.track_window = track_window
                self.process_frame.roi_hist = roi_hist
                self.process_frame.term_crit = term_crit

            hsv = cv2.cvtColor(image_np, cv2.COLOR_BGR2HSV)
            dst = cv2.calcBackProject([hsv], [0], self.process_frame.roi_hist, [0, 180], 1)
            ret, self.process_frame.track_window = cv2.meanShift(dst, self.process_frame.track_window, self.process_frame.term_crit)
            x, y, w, h = self.process_frame.track_window
            current_point = [x + w // 2, y + h // 2]

        if current_point is not None:
            self.overlay_points(image_np, current_point, 0, 255, 0, 'tracked buoy')
            self.overlay_points(image_np, desired_point, 255, 0, 0, 'desired point')

            current_point_meter = cam.convertOnePoint2meter(current_point)
            # self.get_logger().info(f'current point in meters: {current_point_meter}')

            msg = Float64MultiArray()
            msg.data = [float(current_point[0]), float(current_point[1]),float(buoy_width_px),float(distance)]
            msg_desired = Float64MultiArray()
            msg_desired.data = [float(desired_point[0]), float(desired_point[1]),float(desired_point[2])]
            self.publisher.publish(msg)
            self.publisher_desired.publish(msg_desired)

        cv2.imshow("image", image_np)
        cv2.setMouseCallback("image", click_detect)


        if get_hsv:
            hsv = cv2.cvtColor(image_np, cv2.COLOR_BGR2HSV)
            if 0 <= mouseY < hsv.shape[0] and 0 <= mouseX < hsv.shape[1]:
                hsv_value = hsv[mouseY, mouseX]
                self.get_logger().info(f"HSV Value at ({mouseX}, {mouseY}): {hsv_value}")
            get_hsv = False

        cv2.waitKey(2)

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            tracker_type = 'color'  # or 'meanshift'
            self.process_frame(frame, tracker_type)

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = BuoyTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()