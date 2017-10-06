#!/usr/bin/env python
import os, cv2, numpy as np


def overlay_original(img):
    img_ethz = cv2.imread('/home/poine/work/smocap.git/test/track_ethz_rectified.png', cv2.IMREAD_GRAYSCALE)
    img_ethz = np.rot90(img_ethz)
    hz, wz = 495, 380
    img_ethz1 = cv2.resize(img_ethz,(hz, wz), interpolation = cv2.INTER_CUBIC)

    hm, wm = 500, 370
    dx, dy = -12, 7
    y1, x1 = (hm-hz)/2+dy, (wm-wz)/2+dx
    y2, x2 = min(y1+hz, hm), min(x1+wz, wm)
    for x in range(x1, x2):
        for y in range(y1,y2):
            img[y, x] = img_ethz1[x-x1, y-y1]
    

def make_map(map_resolution=0.005):
    path = '/home/poine/work/rosmip.git/rosmip/rosmip_worlds/maps/track_ethz'
    map_size=(500, 370, 1)
    img = np.zeros(map_size, np.uint8)
    hm, wm = 500, 370
    cv2.rectangle(img, (0,0), (map_size[1], map_size[0]), (255), -1) # white background

    overlay_original(img)
    
    img_path = path+".png"
    cv2.imwrite(img_path, img)

    yaml_txt = '''
image: {}
resolution: {}
origin: [0., -0.3, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
'''.format(os.path.basename(img_path), map_resolution)
    yaml_path = path+'.yaml'
    with open(yaml_path, 'w') as f:
        f.write(yaml_txt)




if __name__ == '__main__':
    make_map()
