import cv2
import numpy as np
import logging
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from clover import long_callback

#НАСТРОЙКИ
ENTRY_RADIUS = 40  # радиус для засчета пролета
EXIT_RADIUS = 30   # радиус для сброса состояния
MIN_AREA = 200     # порог
CIRCULARITY_THRESHOLD = 0.8   #квадрат 0.78, круг 1.0
MAX_LOST_FRAMES = 5

#состояние
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
    h, w = frame.shape[:2]
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blur, 40, 120)
    
    kernel = np.ones((3, 3), np.uint8)
    dilated = cv2.dilate(edges, kernel, iterations=1)
    
    contours, _ = cv2.findContours(dilated, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours = sorted(contours, key=cv2.contourArea, reverse=True)

    found_valid_circle = False
    visible_circles = 0
    
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < MIN_AREA: continue

        perimeter = cv2.arcLength(cnt, True)
        if perimeter == 0: continue
        
        approx = cv2.approxPolyDP(cnt, 0.02 * perimeter, True)
        if len(approx) < 6:  #минимум вершин
            continue

        circularity = 4 * np.pi * area / (perimeter * perimeter)

        if circularity > CIRCULARITY_THRESHOLD:
            (x, y), radius = cv2.minEnclosingCircle(cnt) #минимальная оркужность
            radius = int(radius)
            
            found_valid_circle = True
            visible_circles += 1
            
            cv2.circle(frame, (int(x), int(y)), radius, (0, 255, 0), 3)
            cv2.circle(frame, (int(x), int(y)), 2, (0, 0, 255), -1)

            if visible_circles == 1:
                lost_frames = 0
                
                dist_to_center = ((x - w/2)**2 + (y - h/2)**2)**0.5
                
                if radius > ENTRY_RADIUS and dist_to_center < 120 and not in_gate:
                    gates_count += 1
                    in_gate = True
                    logging.info(f"ПРОЛЕТ КОЛЬЦА #{gates_count}. R: {radius}px")
                
                if radius < EXIT_RADIUS and in_gate:
                    in_gate = False

    if not found_valid_circle:
        lost_frames += 1
        if lost_frames >= MAX_LOST_FRAMES:
            in_gate = False

    #HUD
    cv2.putText(frame, f"PASS: {gates_count} | VIS: {visible_circles}", (20, 50), 
                cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 2)

    if image_pub is not None:
        img_msg = bridge.cv2_to_imgmsg(frame, 'bgr8')
        image_pub.publish(img_msg)

    cv2.imshow("Clover Ring Detector", frame)
    cv2.waitKey(1)

if __name__ == "__main__":
    rospy.init_node('ring_detector')
    image_pub = rospy.Publisher('~debug', Image, queue_size=1)
    rospy.Subscriber('main_camera/image_raw', Image, image_callback)
    
    logging.info("Детектор колец запущен.")
    rospy.spin()