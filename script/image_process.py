"""
2022.07.28 Last Edited by Evin Hsu
---------------------------------------
pgm resize
"""

import cv2
import numpy as np

f_in = "/home/user/rrt_rl_ws/src/rrt_rl_navigation/maps/0712/map.pgm"
f_out = "/home/user/rrt_rl_ws/src/rrt_rl_navigation/maps/0712/big_map.png"

resolution = 0.05

if __name__ == "__main__":
    
    img = cv2.imread(f_in, cv2.IMREAD_UNCHANGED)
    print('Datatype: ', img.dtype)
    
    height  = int(img.shape[0] / resolution)
    width = int(img.shape[1] / resolution)
    
    img = cv2.resize(img, (width, height),  interpolation=cv2.INTER_AREA)
    cv2. imwrite(f_out, img)
    
