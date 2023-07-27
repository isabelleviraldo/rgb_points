import numpy as np

x = 5
y = 2
z = 1

img_h = 512
img_w = 896

cam_h_fov = 70
cam_w_fov = 110

if x <= 0:
    print('object too close to lidar, please clean the lens')
    print(0)
    exit()

h_theta = np.arctan2(z,x)*(180/3.14159)
w_theta = np.arctan2(y,x)*(180/3.14159)

h_pixel = (h_theta/int(cam_h_fov/2))*(img_h/2)
w_pixel = (w_theta/int(cam_w_fov/2))*(img_w/2)

initial_h = int((img_h/2) - 1)
initial_w = int((img_w/2) - 1)

if h_pixel > 0:
    h_pixel = h_pixel - 1
h_pixel = int(h_pixel)

if w_pixel > 0:
    w_pixel = w_pixel - 1
w_pixel = int(w_pixel)

location_h = initial_h - h_pixel
location_w = initial_w - w_pixel

if location_h < 0:
    print(0)
    exit()
if location_w < 0:
    print(0)
    exit()
if location_h >= 512:
    print(0)
    exit()
if location_w >= 896:
    print(0)
    exit()

print(location_h)
print(location_w)













