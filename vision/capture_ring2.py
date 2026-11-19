"""Run manually on MaixCAM at high coarse pose. Never commands the robot.
Camera/save APIs: https://wiki.sipeed.com/maixpy/api/maix/image.html#save
Saves 60 stationary frames every 1 second; distinct session folder avoids overwrite.
"""
from maix import app, camera, display, time, err
import os
OUTPUT = '/root/ring2_capture/session_%d' % time.ticks_ms()
os.makedirs(OUTPUT, exist_ok=False)
cam = camera.Camera(320, 240, buff_num=1)
disp = display.Display()
start = time.ticks_ms()
last_saved = start
count = 0
while not app.need_exit() and count < 60:
    img = cam.read()
    now = time.ticks_ms()
    if now-start >= 2000 and now-last_saved >= 1000:
        path = OUTPUT + '/ring2_%03d.jpg' % count
        status = img.save(path, quality=95)
        if status != err.Err.ERR_NONE:
            raise RuntimeError('Image save failed: %s %s' % (path, status))
        print(path)
        last_saved=now
        count+=1
    disp.show(img)
print('Saved',count,'images to',OUTPUT)
