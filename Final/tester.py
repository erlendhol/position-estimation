import cv2
from data_processer import dataProcesser
import time
from threading import Thread


processer = dataProcesser()
"""img_thread = Thread(target=processer.run, args=())
img_thread.daemon = True
img_thread.start()"""


while True:
    processer.run()
    z = processer.z_translatoric_offset
    x = processer.x_translatoric_offset
    angle = processer.x_angle_offset
    frame = processer.frame
    cv2.imshow('test', frame)

    key = cv2.waitKey(1)
    if key == 27:
        break


