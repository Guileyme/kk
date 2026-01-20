import cv2
import json
import numpy as np
import os
import logging
import sys

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    handlers=[
        logging.FileHandler("drone_debug.log"),
        logging.StreamHandler()
    ]
)

def load_config(file_path='config.json'):
    try:
        with open(file_path, 'r') as f:
            data=json.load(f)
        logging.info(f"Конфигурация загружена из {file_path}")

        data['hsv_settings']['lower']  = np.array(data['hsv_settings']['lower'])
        data['hsv_settings']['upper']  = np.array(data['hsv_settings']['upper'])
        return data
    except FileNotFoundError:
        logging.warning(f"Критическая ошибка: Файл {file_path} не найден")
        return None
    except json.JSONDecodeError:
        logging.error(f"Ошибка парсинга из {file_path}")
        return None
    except Exception as e:
        logging.exception(f"Непредвиденная ошибка")
        return None

cfg=load_config()

if cfg is None:
    logging.critical("Ошибка: Конфиг не загружен. Завершение программы.")
    sys.exit(1)

LOWER_COLOR = cfg['hsv_settings']['lower']
UPPER_COLOR = cfg['hsv_settings']['upper']
MIN_RADIUS = cfg['detection']['min_radius']
ENTRY_R = cfg['detection'].get('entry_radius', 180)
EXIT_R = cfg['detection'].get('exit_radius', 150)
MAX_LOST_FRAMES = 5

def detect_ring():

    cap = cv2.VideoCapture(0)#vebka 0,1
    
    gates_count = 0
    in_gate = False
    lost_frames = 0

    logging.info("Система детекции запущена.")

    while True:
        ret, frame = cap.read()
        if not ret:
            logging.error("Потерян сигнал")
            break

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask=cv2.inRange(hsv, LOWER_COLOR, UPPER_COLOR)
        mask=cv2.erode(mask, None, iterations=2)
        mask=cv2.dilate(mask, None, iterations=2)
        mask=cv2.medianBlur(mask, 5)

        contours,_=cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            c=max(contours,key=cv2.contourArea)
            ((x,y),radius)=cv2.minEnclosingCircle(c)

            if radius>MIN_RADIUS:
                lost_frames=0

                cv2.circle(frame,(int(x),int(y)),int(radius),(0,255,0),2)
                cv2.circle(frame,(int(x),int(y)),5,(0,0,255),-1)

                if radius> ENTRY_R and not in_gate:
                    gates_count += 1
                    in_gate = True
                    logging.info(f"Пролёт #{gates_count} зафиксирован")
                
                if radius< EXIT_R and in_gate:
                    in_gate = False
            else:
                lost_frames += 1
                if lost_frames >= MAX_LOST_FRAMES:
                    in_gate = False
        else:
            lost_frames += 1
            if lost_frames >= MAX_LOST_FRAMES:
                in_gate = False

        cv2.imshow("Drone Vision", frame)
       #cv2.imshow("Mask", mask)

        if cv2.waitKey(1)&0xFF==ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__=="__main__":
    detect_ring()