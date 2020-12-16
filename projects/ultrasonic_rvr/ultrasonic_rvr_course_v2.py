#-------------------------------------
# ultrasonic_rvr_course
# サンプルコードをベースに、カスタムしたコーストレースプログラム
# 2台の超音波センサ(左右)からの距離情報を元に軌道修正
# サンプルコードとの変化点
#・回転動作の速度を半減(255→128)
#  ・（未対応）角度調整を秒単位から角度単位に修正
#-------------------------------------


import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../')))

import RPi.GPIO as GPIO
import asyncio

from sphero_sdk import SpheroRvrAsync
from sphero_sdk import SerialAsyncDal
import time

loop = asyncio.get_event_loop()
rvr = SpheroRvrAsync(
    dal=SerialAsyncDal(
        loop
    )
)
GPIO.setmode(GPIO.BCM)

right_trigger = 20
right_echo = 21
left_trigger = 27
left_echo = 18

max_slesh = 5000
right_slesh = 20
left_slesh = 20

GPIO.setup(left_trigger, GPIO.OUT)
GPIO.setup(left_echo, GPIO.IN)
GPIO.setup(right_trigger, GPIO.OUT)
GPIO.setup(right_echo, GPIO.IN)


def distance_left():
    GPIO.output(left_trigger, True)

    time.sleep(0.00001)
    GPIO.output(left_trigger, False)

    start_time = time.time()
    stop_time = time.time()
    elap_time = 0
    #print("left echo off")
    while GPIO.input(left_echo) == 0:
        elap_time = time.time() - stop_time
        if elap_time > 1:
            print("left sensor echo on error")
            return 2000 
        start_time = time.time()

    #print("left echo on")
    while GPIO.input(left_echo) == 1:
        elap_time = time.time() - start_time
        if elap_time > 1:
            print("left sensor echo off error")
            return 2000 
        stop_time = time.time()

    time_elapsed = stop_time - start_time

    distance = (time_elapsed * 34300) / 2
    #print(distance)
    return distance


def distance_right():
    GPIO.output(right_trigger, True)

    time.sleep(0.00001)
    GPIO.output(right_trigger, False)

    start_time = time.time()
    stop_time = time.time()
    elap_time = 0
    #print("right echo off")
    while GPIO.input(right_echo) == 0:
        elap_time = time.time() - stop_time
        if elap_time > 1:
            print("right sensor echo on error")
            return 10000
        start_time = time.time()
    #print("right echo on")
    while GPIO.input(right_echo) == 1:
        elap_time = time.time() - start_time
        if elap_time > 1:
            print("right sensor echo off error")
            return 10000
        stop_time = time.time()

    time_elapsed = stop_time - start_time

    distance = (time_elapsed * 34300) / 2
    return distance


async def main():
    await rvr.wake()
    #print("wake")
    await rvr.reset_yaw()
    #print("reset_yaw")
    await asyncio.sleep(.5)
    #print("sleep")

    actmode = 3 # 0:go forward, 1:left, 2:right, 3:stop, 4:back
    buactmode = actmode

    while True:
        #print("watch around")
        dist_r =  distance_right()
        dist_l =  distance_left() 
        await asyncio.sleep(.03)
        print("Measurements are {0} cm right and {1} cm left".format(dist_r, dist_l))
        
        if dist_r < right_slesh and dist_l < left_slesh: # いずれのセンサの値も小さければバック
            while dist_r < right_slesh and dist_l < left_slesh: 
                print('turning right')
                actmode = 4
                #await rvr.raw_motors(2,128,1,128)
                await rvr.drive_with_heading(50,0,1)
                #await rvr.drive_with_heading(50,15,0)
                dist_r =  distance_right()
                dist_l =  distance_left()
                await asyncio.sleep(.03)
            await rvr.reset_yaw()
        elif dist_r < right_slesh:
            while dist_r < right_slesh: # 右センサが近ければ右旋回
                print('turning right')
                actmode = 2
                await rvr.raw_motors(2,128,1,128)
                #await rvr.drive_with_heading(50,15,0)
                dist_r =  distance_right()
                await asyncio.sleep(.03)
            await rvr.reset_yaw()
        elif dist_l < left_slesh: # 左センサの値が近い場合左旋回
            while dist_l < left_slesh:
                print('turning left')
                actmode = 1
                await rvr.raw_motors(1,128,2,128)
                #await rvr.drive_with_heading(50,15,0)
                dist_l =   distance_left()
                await asyncio.sleep(.03)
            await rvr.reset_yaw()
        elif dist_l >= left_slesh and dist_r >= right_slesh: # いずれの超音波センサの距離値もしきい値以上であれば直進
            if dist_l < max_slesh and dist_r < max_slesh:
                print("go forward")
                actmode = 0
                await rvr.get_motor_thermal_protection_status()
                await rvr.get_motor_fault_state()
                await rvr.get_rgbc_sensor_values()
                #if buactmode != actmode:
                await rvr.drive_with_heading(50,0,0)
            else:
                print("stop")
                actmode = 3
                if buactmode != actmode:
                    await rvr.drive_with_heading(0,0,0)
        buactmode = actmode
 
try:
    loop.run_until_complete(
        asyncio.gather(
            main()
        )
    )
except KeyboardInterrupt:
    print("Program ended by KeyboardInterrupt")
    GPIO.cleanup()


