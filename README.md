# Coptor_Controller

# encoding: utf-8
from dronekit import VehicleMode 
from pymavlink import mavutil
import time
import math


'''''
宣告陣列及需使用的變數
'''''
active = "stay"
buff_area = ['stay','stay','stay','stay','stay','stay','stay','stay','stay']
count = 0
x = 0




'''''
陣列中的所有值往右位移一格，
並且把新進的值從最左邊補上，
把最舊的值拿掉
'''''
def shift_right_buff_area():
	global buff_area
	temp = buff_area[8]
	for i in range(8,0,-1):
		buff_area[i] = buff_area[i-1]
	buff_area[0] = temp



'''''
定義陀螺儀所傳回的值，
並且在什麼姿勢(角度)下，
而給直升機下什麼樣的指令
'''''
def gyro_float_value(x):
    Stay_x_max = 6000  # -30 <= x
    Stay_x_min = -10000  # x <= 30
    right_x_max = -10000  # x <30
    right_x_min = -16000  # 90 <= x
    left_x_max = 17000  # -90 <= x
    left_x_min = 6000  # x < -30
    if Stay_x_min <= x and x <= Stay_x_max:
        return 'stay'
    elif right_x_min < x and x <= right_x_max:
        return 'right'
    elif left_x_min < x and x <= left_x_max:
        return 'left'





'''''
每當一個指令放進陣列時，並且跟上一個放進的指令相同時，則count+1，
而當陣列裡的所有值都相同時，就執行最新一個傳進的指令，
但當指令不同時時，則count重新計算

'''''
def current_pose(x):
	global buff_area
	global count
	global active
	shift_right_buff_area()
	buff_area[0] = gyro_float_value(x)
	print(buff_area)
	if buff_area[0] == buff_area[1]:
		count = count + 1
		if count == 8:
			count = 0
			active = buff_area[0]
			if active == ' right ' :
				send_ned_velocity(2,0,0,1)
			elif active == ' left ' :
				send_ned_velocity(-2,0,0,1) 
	else:
		count = 0
		




'''''
直升機檢查、解鎖、起飛函式
可設定模式及起飛高度
'''''
def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """
    print "Basic pre-arm checks"
    # Don't let the user try to arm until autopilot is ready
    while not vehicle.is_armable:
        print " Waiting for vehicle to initialise..."
        time.sleep(1)

        
    print "Arming motors"
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:      
        print " Waiting for arming..."
        time.sleep(1)

    print "Taking off!"
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command 
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print " Altitude: ", vehicle.location.global_relative_frame.alt      
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: #Trigger just below target alt.
            print "Reached target altitude"
            break
        time.sleep(1)

#Arm and take of to altitude of 3 meters
arm_and_takeoff(3)





'''''
直升機側飛函式
x 使用正負數可設定南北向
y 可設定東北向
z 為設定下次移動的高度
duration 為設定持續時間
EX：
向北移動2公尺在1秒內完成 = send_ned_velocity(2,0,0,1)
'''''
def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 

    # send command to vehicle on 1 Hz cycle
    for x in range(0,duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)



'''''
使用serial與裝置進行連接，
並接收陀螺儀數值
使用port /ttyS0
鮑rate 設置為57600
'''''
def setup():
        global x
        x = serial.Serial("/dev/ttyS0", 57600)



'''
def loop():
        global x
        print (x) 
'''




'''''
執行上方函式
'''''
if __name__ == '__main__':
        setup()
        while Ture:
                loop()
                current_pose(x)
                
                
'''''
註:程式尚未完成
'''''
