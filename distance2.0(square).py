import cv2
import numpy as np
import logging
import rospy
import sys
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from clover import long_callback

ENTRY_W = 70
EXIT_W = 50
MIN_AREA = 500
MAX_LOST_FRAMES = 5

gates_count = 0
in_gate = False
lost_frames = 0
bridge = CvBridge()
image_pub = None

logging.basicConfig(level=logging.INFO, format="%(message)s")

@long_callback
def image_callback(data):
    global gates_count, in_gate, lost_frames, image_pub

    frame = bridge.imgmsg_to_cv2(data, 'bgr8')
    
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    edged = cv2.Canny(blur, 50, 150)
    
    contours, _ = cv2.findContours(edged, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    found_valid_square = False
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < MIN_AREA: continue

        peri = cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)

        if len(approx) == 4:
            found_valid_square = True
            lost_frames = 0
            x, y, rect_w, rect_h = cv2.boundingRect(approx)
            
            cv2.drawContours(frame, [approx], -1, (0, 255, 0), 3)
            cv2.circle(frame, (x + rect_w//2, y + rect_h//2), 5, (0, 0, 255), -1)

            if rect_w > ENTRY_W and not in_gate:
                gates_count += 1
                in_gate = True
                logging.info(f"ПРОЛЕТ #{gates_count}. Ширина: {rect_w}px")
            
            if rect_w < EXIT_W and in_gate:
                in_gate = False
            break

    if not found_valid_square:
        lost_frames += 1
        if lost_frames >= MAX_LOST_FRAMES:
            in_gate = False

    cv2.putText(frame, f"GATES: {gates_count}", (20, 50), 
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    if image_pub is not None:
        img_msg = bridge.cv2_to_imgmsg(frame, 'bgr8')
        image_pub.publish(img_msg)

    cv2.imshow("Clover Geometry Debug", frame)
    cv2.waitKey(1)

if __name__ == "__main__":
    rospy.init_node('square_detector')
    
    image_pub = rospy.Publisher('~debug', Image, queue_size=1)
    
    rospy.Subscriber('main_camera/image_raw', Image, image_callback)
    
    logging.info("Детектор запущен. Топик публикации: /square_detector/debug")
    rospy.spin()