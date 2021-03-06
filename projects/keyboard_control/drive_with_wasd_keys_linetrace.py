import sys
import os
import cv2
import time
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../')))

import asyncio

from helper_keyboard_input import KeyboardHelper
from sphero_sdk import SerialAsyncDal
from sphero_sdk import SpheroRvrAsync
from sphero_sdk import RvrStreamingServices

# initialize global variables
key_helper = KeyboardHelper()
current_key_code = -1
driving_keys = [119, 97, 115, 100, 32]
speed = 0
heading = 0
flags = 0
sleshR = 200
sleshG = 200
sleshB = 200
now_R = 0
now_G = 0
now_B = 0


loop = asyncio.get_event_loop()
rvr = SpheroRvrAsync(
    dal=SerialAsyncDal(
        loop
    )
)

def keycode_callback(keycode):
    global current_key_code
    current_key_code = keycode
    print("Key code updated: ", str(current_key_code))

async def color_detected_handler(color_detected_data):
    #print("Color detection data response: ", color_detected_data)
    col_detdata = color_detected_data['ColorDetection']
    now_R = col_detdata['R']
    now_G = col_detdata['G']
    now_B = col_detdata['B']
    

async def main():
    """
    Runs the main control loop for this demo.  Uses the KeyboardHelper class to read a keypress from the terminal.

    W - Go forward.  Press multiple times to increase speed.
    A - Decrease heading by -10 degrees with each key press.
    S - Go reverse. Press multiple times to increase speed.
    D - Increase heading by +10 degrees with each key press.
    Spacebar - Reset speed and flags to 0. RVR will coast to a stop

    """
    global current_key_code
    global speed
    global heading
    global flags    
    global speed_bu
    global heading_bu
    global flags_bu
    speed_bu = 0
    heading_bu = 0
    flags_bu = 0
    rvrstatedic = {"run":0, "rsearch":1, "lsearch":2, "stop":3, "rcvpos":4 } 
    rvrstate = rvrstatedic["run"] # 状態
    stime = 0 # runモード開始時間
    rvrstate_bu = rvrstatedic["rsearch"] # 前の状態
    rscnt = 0 # 右旋回サーチの回数
    lscnt = 0 # 左旋回サーチの回数

    await rvr.wake()

    await rvr.reset_yaw()

    # color detect awake
    await rvr.enable_color_detection(is_enabled=True)

    await rvr.sensor_control.add_sensor_data_handler(
        service=RvrStreamingServices.color_detection,
        handler=color_detected_handler
    )
    await rvr.sensor_control.start(interval=50)
    
            
    cap = cv2.VideoCapture(0)
    
    while True:
        
        ret, frame = cap.read()
        cv2.imshow("Frame", frame)
        key = cv2.waitKey(1)

        if rvrstate == rvrstatedic["run"]:
            if rvrstate_bu = rvrstate:
                stime = time.time()
            else:
                if time.time() - stime > 1: # 1秒以上runモードで走れるなら、サーチカウントリセット
                    rscnt = 0
                    lscnt = 0
                else:
                    rvrstate = rvrstatedic["rcvpos"]
            if now_R > sleshR and now_G > sleshG and now_B > sleshB: # normal run
                heading = 0
                speed = 32
                flags = 0
            else: # over root -> position recovery
                rvrstate = rvrstatedic["rcvpos"]
                heading = 0
                speed = 0
                flags = 0
        elif rvrstate == rvrstatedic["rcvpos"]:
            if now_R > sleshR and now_G > sleshG and now_B > sleshB: # テープ上まで復帰できたなら、右サーチを開始する
                if rscnt <= 5:
                    rvrstate = rvrstatedic["rsearch"]
                elif lscnt <= 10:
                    rvrstate = rvrstatedic["lsearch"]
                else:
                    rvrstate = rvrstatedic["stop"]
                    
                heading = 0
                speed = 0
                flags = 0
            else: # テープ上に来るまでバックを継続する
                heading = 0
                speed = 32
                flags = 1
        elif rvrstate == rvrstatedic["rsearch"]:
            handing = 10
            speed = 0
            flags = 0
            rscnt += 1
            rvrstate = rvrstatedic["run"]:
        elif rvrstate = rvrstatedic["lsearch"]:
            handing = -10
            speed = 0
            flags = 0
            lscnt += 1
            rvrstate = rvrstatedic["run"]:
        elif rvrstate = rvrstatedic["stop"]: # マニュアルモードに変更
            if key == 27:
                break
            if current_key_code == 119:  # W
                # if previously going reverse, reset speed back to 64
                if flags == 1:
                    speed = 0
                    #speed = 32
                else:
                    # else increase speed
                    speed += 32
                # go forward
                flags = 0
            elif current_key_code == 97:  # A
                heading -= 10
            elif current_key_code == 115:  # S
                # if previously going forward, reset speed back to 64
                if flags == 0:
                    speed = 0
                    #speed = 32
                else:
                    # else increase speed
                    speed += 32
                # go reverse
                flags = 1
            elif current_key_code == 100:  # D
                heading += 10
            elif current_key_code == 32:  # SPACE
                # reset speed and flags, but don't modify heading.
                speed = 0
                flags = 0

        rvrstate_bu = rvrstate

        # check the speed value, and wrap as necessary.
        if speed > 255:
            speed = 255
        elif speed < -255:
            speed = -255

        # check the heading value, and wrap as necessary.
        if heading > 359:
            heading = heading - 359
        elif heading < 0:
            heading = 359 + heading

        # reset the key code every loop
        current_key_code = -1

        # issue the driving command
        if speed != speed_bu or heading != heading_bu or flags != flags_bu:
            await rvr.drive_with_heading(speed, heading, flags)
        speed_bu = speed
        heading_bu = heading
        flags_bu = flags

        # sleep the infinite loop for a 10th of a second to avoid flooding the serial port.
        await asyncio.sleep(0.1)

    cap.release()
    cv2.destroyAllWindows()

def run_loop():
    global loop
    global key_helper
    key_helper.set_callback(keycode_callback)
    loop.run_until_complete(
        asyncio.gather(
            main()
        )
    )


if __name__ == "__main__":
    loop.run_in_executor(None, key_helper.get_key_continuous)
    #try:
    run_loop()
    #except KeyboardInterrupt:
    #    print("Keyboard Interrupt...")
    #    key_helper.end_get_key_continuous()
    #    loop.run_until_complete(
    #        asyncio.gather(
    #            rvr.enable_color_detection(is_enabled=False),
    #            rvr.close()
    #        )
    #    )
    #finally:
    #    print("Press any key to exit.")
    #    exit(1)
