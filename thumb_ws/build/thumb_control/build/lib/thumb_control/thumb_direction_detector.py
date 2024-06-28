#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
import numpy as np

class ThumbDirectionDetector(Node):
    def __init__(self):
        super().__init__('thumb_direction_detector')
        self.publisher = self.create_publisher(String, '/thumb_direction', 10)
        self.cap = cv2.VideoCapture(1)
        
        if not self.cap.isOpened():
            self.get_logger().error('Failed to open camera')
            rclpy.shutdown()
            return
        
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error('Failed to capture image')
            return
        
        direction = self.detect_thumb_direction(frame)
        self.publisher.publish(String(data=direction))

        cv2.imshow('Thumb Direction Detector', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            rclpy.shutdown()

    def detect_thumb_direction(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        lower_skin = np.array([0, 20, 70], dtype=np.uint8)
        upper_skin = np.array([20, 255, 255], dtype=np.uint8)
        
        mask = cv2.inRange(hsv, lower_skin, upper_skin)
        
        blur = cv2.GaussianBlur(mask, (5, 5), 0)
        
        contours, _ = cv2.findContours(blur, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            contour = max(contours, key=cv2.contourArea)
            

            hull = cv2.convexHull(contour, returnPoints=False)
            if len(hull) > 3:
                defects = cv2.convexityDefects(contour, hull)
                if defects is not None:
                    finger_count = 0
                    for i in range(defects.shape[0]):
                        s, e, f, d = defects[i, 0]
                        start = tuple(contour[s][0])
                        end = tuple(contour[e][0])
                        far = tuple(contour[f][0])
                        a = np.linalg.norm(np.array(end) - np.array(start))
                        b = np.linalg.norm(np.array(far) - np.array(start))
                        c = np.linalg.norm(np.array(end) - np.array(far))
                        angle = np.arccos((b**2 + c**2 - a**2) / (2 * b * c)) * 57.2957795

                        if angle <= 90:
                            finger_count += 1

                    if finger_count == 1:
                        x, y, w, h = cv2.boundingRect(contour)
                        if x + w // 2 < image.shape[1] // 2:
                            return 'left'
                        else:
                            return 'right'
                    elif finger_count == 0:
                        return 'down'
                    else:
                        return 'up'
        return 'down'

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    thumb_direction_detector = ThumbDirectionDetector()
    try:
        rclpy.spin(thumb_direction_detector)
    except KeyboardInterrupt:
        pass
    thumb_direction_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
