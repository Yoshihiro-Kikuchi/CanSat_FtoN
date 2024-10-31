import math
import RPi.GPIO as GPIO
import time
import serial
import threading
import smbus

import numpy as np
import cv2

from picamera2 import Picamera2

import board
import busio
from adafruit_bme280 import basic as adafruit_bme280


import micropyGPS
from bno055_new import BNO055


#定数定義
lon_goal = 139.987399 # カラーコーンの経度(本番に入力)
lat_goal = 40.142287 # カラーコーンの緯度(本番に入力)
pre0 = 1013.15

# GPIOピンの設定
R_IN1 = 5
R_IN2 = 6
L_IN1 = 19
L_IN2 = 13
Nichrome_wire = 26
LED = 17
TRIG_PIN = 23
ECHO_PIN = 18

#センサーの設定
gps = micropyGPS.MicropyGPS(9, 'dd')
bno = BNO055()
#pc2 = Picamera2()

# GPIOの初期化
GPIO.setmode(GPIO.BCM)
GPIO.setup(R_IN1, GPIO.OUT)
GPIO.setup(R_IN2, GPIO.OUT)
GPIO.setup(L_IN1, GPIO.OUT)
GPIO.setup(L_IN2, GPIO.OUT)
GPIO.setup(Nichrome_wire, GPIO.OUT)
GPIO.setup(LED, GPIO.OUT)
GPIO.setup(TRIG_PIN,GPIO.OUT)
GPIO.setup(ECHO_PIN,GPIO.IN)


L_IN1_PWM = GPIO.PWM(L_IN1, 120) #120Hz
L_IN2_PWM = GPIO.PWM(L_IN2, 120)
R_IN1_PWM = GPIO.PWM(R_IN1, 120) #120Hz
R_IN2_PWM = GPIO.PWM(R_IN2, 120)
                
L_IN1_PWM.start(0)
L_IN2_PWM.start(0)
R_IN1_PWM.start(0)
R_IN2_PWM.start(0)

#変数定義
stage = 0
pres = 0
temp = 0
lat_now = 0
lon_now = 0
dist_now = 0
angle_bet_goal_and_now = 0
angle_now = 0
alti = 0
alti_sum = 0
θ = 0
θ1 = 0
close_distance_now = 0
duty = 80
state = "stop"

#csvファイル作成
file_name = 'NSE_2024_FtoN_3.csv'
init_sd_time = time.time()
file = open(file_name, 'a')
file.write("mission_time,stage,longitude,latitude,pressure,temprature,distance,angle_now,θ,state\r\n")
ini_mission_time = time.time()


def run_gps(): 
    s = serial.Serial('/dev/serial0', 9600, timeout=10)
    s.readline() 
    while True:
        sentence = s.readline().decode('utf-8') 
        if sentence[0] != '$': 
            continue
        for x in sentence: 
            gps.update(x)

gpsthread = threading.Thread(target=run_gps) 
gpsthread.start() 


def run_bno055():
    global angle_now
    if bno.begin() is not True:
        print("Error initializing device")
        exit()
    time.sleep(1)
    bno.setExternalCrystalUse(True)
    while True:
        angle_now = bno.getVector(BNO055.VECTOR_EULER)[0]
        if 180 < angle_now < 360:
            angle_now = angle_now - 360 #0~360から-180~180に変換
        time.sleep(0.1)
    return
    
bnothread = threading.Thread(target=run_bno055) 
bnothread.start()


def run_bme():
    global pres, temp
    i2c = board.I2C()
    bme280 = adafruit_bme280.Adafruit_BME280_I2C(i2c, 0x76)
    while True:
        pres = bme280.pressure
        temp = bme280.temperature
        time.sleep(0.1)

bmethread = threading.Thread(target=run_bme) 
bmethread.start()

#以下超音波センサーのプログラム
def pulseIn(PIN, start=1, end=0): # ECHO_PINがHIGHである時間を計測
    if start==0: end = 1
    t_start = 0
    t_end = 0
    while GPIO.input(PIN) == end:
        t_start = time.time()
    while GPIO.input(PIN) == start:
        t_end = time.time()
    return t_end - t_start

# 距離計測
def calc_distance(TRIG_PIN, ECHO_PIN, v): 
    while True:
        GPIO.output(TRIG_PIN, GPIO.LOW)
        time.sleep(0.3) # TRIGピンを0.3[s]だけLOW
        GPIO.output(TRIG_PIN, True)
        time.sleep(0.00001) # TRIGピンを0.00001[s]だけ出力(超音波発射)
        GPIO.output(TRIG_PIN, False)
        t = pulseIn(ECHO_PIN)
        # 距離[cm] = 音速[cm/s] * 時間[s]/2
        close_distance = v * t/2 #単位はcm
        return close_distance
        #print(distance, "cm")
            

def run_ultrasonic():
    global close_distance_now, close_distance
    while True:
        v = 33150 + 60*24
        GPIO.setwarnings(False)
        close_distance_now = calc_distance(TRIG_PIN, ECHO_PIN, v)
        time.sleep(0.1)
    return

ultrasonicthread = threading.Thread(target=run_ultrasonic) 
#ultrasonicthread.start() #この段階ではthreadを開始せず、カメラ誘導に移行する段階でthreadを開始する

def read():
    global pres, lat_now, lon_now
    try:
        if gps.clean_sentences > 20:
            lat_now = gps.latitude[0]
            lon_now = gps.longitude[0]
            #print(lon_now, lat_now)
    except:
            pass
    
    s=threading.Timer(0.1,read)      #Timerでread関数を裏で何度も回す
    s.start()
       

def record():
    global pres, temp, lon_now, lat_now, stage, angle_now, θ, file, init_sd_time, state
    mission_time = time.time() - ini_mission_time
    file.write("%f,%d,%f,%f,%f,%f,%f,%f,%f,%s\r\n"%(mission_time, stage, lat_now, lon_now, pres, temp, distance_and_angle(lat_now, lon_now, lat_goal, lon_goal, ellipsoid=None)['distance'], angle_now, θ, state))
    if (time.time() - init_sd_time > 10):
        file.close()
        file = open(file_name, "a")
        init_sd_time = time.time()
    t=threading.Timer(1,record)     #Timerでrecord関数を裏で何度も回す
    t.start()
    
    
lon_now_rad = math.radians(lon_now) #角度をラジアンに変更
lat_now_rad = math.radians(lat_now)
lon_goal_rad = math.radians(lon_goal)
lat_goal_rad = math.radians(lat_goal)


def alti():
    global temp, pres
    T = temp + 273.15
    if pres == 0:
        alti = -55
    if pres != 0:
        nume = (pre0/pres)**(1/5.257) - 1
        alti = nume*T/0.0065
    time.sleep(0.1)
    return alti


r = 6378137

# 楕円体
ELLIPSOID_GRS80 = 1 # GRS80
ELLIPSOID_WGS84 = 2 # WGS84

# 楕円体ごとの長軸半径と扁平率
GEODETIC_DATUM = {
    ELLIPSOID_GRS80: [
        6378137.0,         # [GRS80]長軸半径
        1 / 298.257222101, # [GRS80]扁平率
    ],
    ELLIPSOID_WGS84: [
        6378137.0,         # [WGS84]長軸半径
        1 / 298.257223563, # [WGS84]扁平率
    ],
}

# 反復計算の上限回数
ITERATION_LIMIT = 1000

'''
Vincenty法(逆解法)
2地点の座標(緯度経度)から、距離と方位角を計算する
:param lat1: 始点の緯度
:param lon1: 始点の経度
:param lat2: 終点の緯度
:param lon2: 終点の経度
:param ellipsoid: 楕円体
:return: 距離と方位角
'''
def distance_and_angle(lat1, lon1, lat2, lon2, ellipsoid=None):

    # 差異が無ければ0.0を返すaaaaaaaa
    if math.isclose(lat1, lat2) and math.isclose(lon1, lon2):
        return {
            'distance': 0.0,
            'azimuth1': 0.0,
            'azimuth2': 0.0,
        }

    # 計算時に必要な長軸半径(a)と扁平率(ƒ)を定数から取得し、短軸半径(b)を算出する
    # 楕円体が未指定の場合はGRS80の値を用いる
    a, ƒ = GEODETIC_DATUM.get(ellipsoid, GEODETIC_DATUM.get(ELLIPSOID_GRS80))
    b = (1 - ƒ) * a

    φ1 = math.radians(lat1)
    φ2 = math.radians(lat2)
    λ1 = math.radians(lon1)
    λ2 = math.radians(lon2)

    # 更成緯度(補助球上の緯度)
    U1 = math.atan((1 - ƒ) * math.tan(φ1))
    U2 = math.atan((1 - ƒ) * math.tan(φ2))

    sinU1 = math.sin(U1)
    sinU2 = math.sin(U2)
    cosU1 = math.cos(U1)
    cosU2 = math.cos(U2)

    # 2点間の経度差
    L = λ2 - λ1

    # λをLで初期化
    λ = L

    # 以下の計算をλが収束するまで反復する
    # 地点によっては収束しないことがあり得るため、反復回数に上限を設ける
    for i in range(ITERATION_LIMIT):
        sinλ = math.sin(λ)
        cosλ = math.cos(λ)
        sinσ = math.sqrt((cosU2 * sinλ) ** 2 + (cosU1 * sinU2 - sinU1 * cosU2 * cosλ) ** 2)
        cosσ = sinU1 * sinU2 + cosU1 * cosU2 * cosλ
        σ = math.atan2(sinσ, cosσ)
        sinα = cosU1 * cosU2 * sinλ / sinσ
        cos2α = 1 - sinα ** 2
        cos2σm = cosσ - 2 * sinU1 * sinU2 / cos2α
        C = ƒ / 16 * cos2α * (4 + ƒ * (4 - 3 * cos2α))
        λʹ = λ
        λ = L + (1 - C) * ƒ * sinα * (σ + C * sinσ * (cos2σm + C * cosσ * (-1 + 2 * cos2σm ** 2)))

        # 偏差が.000000000001以下ならbreak
        if abs(λ - λʹ) <= 1e-12:
            break
    else:
        # 計算が収束しなかった場合はNoneを返す
        return None

    # λが所望の精度まで収束したら以下の計算を行う
    u2 = cos2α * (a ** 2 - b ** 2) / (b ** 2)
    A = 1 + u2 / 16384 * (4096 + u2 * (-768 + u2 * (320 - 175 * u2)))
    B = u2 / 1024 * (256 + u2 * (-128 + u2 * (74 - 47 * u2)))
    Δσ = B * sinσ * (cos2σm + B / 4 * (cosσ * (-1 + 2 * cos2σm ** 2) - B / 6 * cos2σm * (-3 + 4 * sinσ ** 2) * (-3 + 4 * cos2σm ** 2)))

    # 2点間の楕円体上の距離
    s = b * A * (σ - Δσ)

    # 各点における方位角
    α1 = math.atan2(cosU2 * sinλ, cosU1 * sinU2 - sinU1 * cosU2 * cosλ)

    if α1 < 0:
        α1 = α1 + math.pi * 2
        
    α1_degrees = math.degrees(α1)
        
    if 180 < α1_degrees < 360:
        α1_degrees = α1_degrees - 360 #0~360から-180~180に変換
        

    return {
        'distance': s,           # 距離
        'azimuth1': α1_degrees, # 方位角(始点→終点)
        }

"""
def straight():
    global state
    GPIO.output(R_IN1, GPIO.HIGH)
    GPIO.output(R_IN2, GPIO.LOW)
    GPIO.output(L_IN1, GPIO.HIGH)
    GPIO.output(L_IN2, GPIO.LOW)
    state = 0
    
    
    
def right():
    global state
    GPIO.output(R_IN1, GPIO.HIGH)
    GPIO.output(R_IN2, GPIO.LOW)
    GPIO.output(L_IN1, GPIO.LOW)
    GPIO.output(L_IN2, GPIO.LOW)
    state = 1
    
    
def left():
    GPIO.output(R_IN1, GPIO.LOW)
    GPIO.output(R_IN2, GPIO.LOW)
    GPIO.output(L_IN1, GPIO.HIGH)
    GPIO.output(L_IN2, GPIO.LOW)
    state = -1
    
  
def turn_right():
    GPIO.output(R_IN1, GPIO.HIGH)
    GPIO.output(R_IN2, GPIO.LOW)
    GPIO.output(L_IN1, GPIO.LOW)
    GPIO.output(L_IN2, GPIO.HIGH)
    
    
def turn_left():
    GPIO.output(R_IN1, GPIO.HIGH)
    GPIO.output(R_IN2, GPIO.LOW)
    GPIO.output(L_IN1, GPIO.HIGH)
    GPIO.output(L_IN2, GPIO.LOW)

    
def stop():
    GPIO.output(R_IN1, GPIO.LOW)
    GPIO.output(R_IN2, GPIO.LOW)
    GPIO.output(L_IN1, GPIO.LOW)
    GPIO.output(L_IN2, GPIO.LOW)
"""
    
def straight_slow():
    global duty, state
    R_IN1_PWM.ChangeDutyCycle(duty)
    R_IN2_PWM.ChangeDutyCycle(0)
    L_IN1_PWM.ChangeDutyCycle(duty)
    L_IN2_PWM.ChangeDutyCycle(0)
    state = "straight"
    
def right_slow():
    global duty, state
    R_IN1_PWM.ChangeDutyCycle(80)
    R_IN2_PWM.ChangeDutyCycle(0)
    L_IN1_PWM.ChangeDutyCycle(0)
    L_IN2_PWM.ChangeDutyCycle(0)
    state = "right"

def left_slow():
    global duty, state
    R_IN1_PWM.ChangeDutyCycle(0)
    R_IN2_PWM.ChangeDutyCycle(0)
    L_IN1_PWM.ChangeDutyCycle(80)
    L_IN2_PWM.ChangeDutyCycle(0)
    state = "left"
    
def stop_slow():
    global duty, state
    R_IN1_PWM.ChangeDutyCycle(0)
    R_IN2_PWM.ChangeDutyCycle(0)
    L_IN1_PWM.ChangeDutyCycle(0)
    L_IN2_PWM.ChangeDutyCycle(0)
    state = "stop"
    
def stuck():
    print("stuck!")
    GPIO.output(R_IN1, GPIO.LOW)
    GPIO.output(R_IN2, GPIO.HIGH)
    GPIO.output(L_IN1, GPIO.LOW)
    GPIO.output(L_IN2, GPIO.HIGH)
    time.sleep(3)
    turn_right()
    time.sleep(2)
    straight()
    time.sleep(2)


def finish():
    stop_slow()
    for i in range (1000):
        GPIO.output(17, 1)
        time.sleep(0.5)
        GPIO.output(17, 0)
        time.sleep(0.5)
    state = "goal"
    GPIO.cleanup()

"""
#カメラ誘導に使う関数
def adjust(img, alpha=1.0, beta=0.0):
    # 積和演算を行う。
    dst = alpha * img + beta
    # [0, 255] でクリップし、uint8 型にする。
    return np.clip(dst, 0, 255).astype(np.uint8)


def rgb_to_score(image, A, B, C):
    r = np.array(image[:, :, 2], dtype=np.int64)
    g = np.array(image[:, :, 1], dtype=np.int64)
    b = np.array(image[:, :, 0], dtype=np.int64)
    image_converted = ((r * A - b * B - g * C) + 255 * (B + C)) // (A + B + C)
    #print(image_converted)
    #print(image_converted[20, 20])
    image_converted = np.array(image_converted, dtype=np.uint8)
    return image_converted
    
def mask(image):
    bgrLower = np.array([0, 0, 210])    # 抽出する色の下限(bgr)
    bgrUpper = np.array([100, 100, 255])    # 抽出する色の上限(bgr)
    img_mask = cv2.inRange(image, (0, 0, 180), (220, 220, 255)) # bgrからマスクを作成
    #cv2.imshow("Camera",img_mask)
    img_mask = np.array(img_mask, dtype=np.uint8)
    return img_mask
"""

def main():
    global stage, pres, temp, lon_now, lat_now, angle_now, angle_goal, alti_sum, lat_goal, lon_goal, close_distance_now
    print('Start main program!')
    time.sleep(1)
    for i in range(10): #基準の高度を算出
        now_alti = alti()
        alti_sum += now_alti
        time.sleep(0.2)
    alti0 = alti_sum / 10
    print(alti0)
    count_phase1 = 0

    try: #本番は
        while True:
            if stage == 0: #準備
                now_alti = alti()
                #print(pres)
                time.sleep(0.5)
                if now_alti >= alti0 + 9:
                    count_phase1 = count_phase1 + 1
                    if count_phase1 >= 3:
                        stage = stage + 1
                        print("stage0 finish!")
                        for i in range (5):
                            GPIO.output(17, 1)
                            time.sleep(0.5)
                            GPIO.output(17, 0)
                            time.sleep(0.5)
                        #finish()
                        #time.sleep(1000)
                else:
                    count_phase1 = 0
                #30mほど上空から落下させる。10mごとに1hPa変化するから25hPaの気圧降下を確認後stage1へ
                
                
            elif stage == 1: #パラ分離
                if alti() - alti0 <=  5:
                    #stage1に移行後、5秒待機。ニクロム線に電流を流し、パラ分離
                    time.sleep(90)
                    GPIO.output(Nichrome_wire, GPIO.HIGH)
                    time.sleep(0.5)
                    GPIO.output(Nichrome_wire, GPIO.LOW)
                    stage = 2
                    print("stage1 finish")
                    #finish()
                    time.sleep(1)
                """
                pc2.start() #take a photo
                pc2.set_controls({"AfMode":controls.AfModeEnum.Continuous})
                time.sleep(1000)
                pc2.capture_file("stage1_finish.jpg")
                pc2.close()
                """
            
            elif stage == 2: #GPS誘導
                stop_slow()
                time.sleep(1)
                """
                while True:
                    print(distance_and_angle(lat_now, lon_now, lat_goal, lon_goal, ellipsoid=None))
                    time.sleep(1)
                    print(angle_now)
                """
                #straight()
                time.sleep(0.5)
                
                while True:
                    θ1 = distance_and_angle(lat_now, lon_now, lat_goal, lon_goal, ellipsoid=None)['azimuth1'] - angle_now
                    #print(θ1)
                    
                    if distance_and_angle(lat_now, lon_now, lat_goal, lon_goal, ellipsoid=None)['azimuth1'] > 0 and angle_now > 0 :
                        θ = θ1
                        
                    elif distance_and_angle(lat_now, lon_now, lat_goal, lon_goal, ellipsoid=None)['azimuth1'] < 0 and angle_now < 0 :
                        θ = θ1
                        
                    else :
                        if abs(θ1) < 180:
                            θ = θ1
                        else:
                            if θ1 < -180:
                                θ = 360 + θ1
                            if 180 < θ1:
                                θ = -360 + θ1
                                
                    if -20 <= θ <= 20 :
                        straight_slow()
                        print(θ, distance_and_angle(lat_now, lon_now, lat_goal, lon_goal, ellipsoid=None)['azimuth1'], distance_and_angle(lat_now, lon_now, lat_goal, lon_goal, ellipsoid=None)['distance'])
                        time.sleep(0.1)
                    elif 20 < θ < 180 :
                        left_slow()
                        print(θ, distance_and_angle(lat_now, lon_now, lat_goal, lon_goal, ellipsoid=None)['azimuth1'], distance_and_angle(lat_now, lon_now, lat_goal, lon_goal, ellipsoid=None)['distance'])
                        time.sleep(0.1)
                        #stop()
                        #time.sleep(0.25)
                    elif -180 < θ < -20:
                        right_slow()
                        print(θ, distance_and_angle(lat_now, lon_now, lat_goal, lon_goal, ellipsoid=None)['azimuth1'], distance_and_angle(lat_now, lon_now, lat_goal, lon_goal, ellipsoid=None)['distance'])
                        time.sleep(0.1)
                        #stop()
                        #time.sleep(0.25)
                
                
                    if distance_and_angle(lat_now, lon_now, lat_goal, lon_goal, ellipsoid=None)['distance'] <= 0.3:
                            break
                
                stop_slow()
                time.sleep(2)
                stage = 3
                print("stage2 finish!")
                #finish()
                         
            elif stage == 3:
                ultrasonicthread.start()
                stage3_start_time = time.time()
                camera = Picamera2

                picam2 = Picamera2()
                picam2.video_configuration.controls.FrameRate = 10.0
                picam2.configure(picam2.create_preview_configuration(main={"format": 'RGB888', "size": (200, 160)}))
                picam2.start()
                time.sleep(0.1)
                while True:
                    stop_slow()
                    time.sleep(0.1)
                    image = picam2.capture_array()

                    # 画像をHSV色空間に変換する
                    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

                    # 赤色の範囲を定義する
                    lower_red1 = np.array([0, 100, 100])
                    upper_red1 = np.array([10, 255, 255])
                    lower_red2 = np.array([170, 100, 100])
                    upper_red2 = np.array([180, 255, 255])

                    # 赤色部分のマスクを作成する
                    mask1 = cv2.inRange(hsv_image, lower_red1, upper_red1)
                    mask2 = cv2.inRange(hsv_image, lower_red2, upper_red2)
                    mask = cv2.bitwise_or(mask1, mask2)

                    #
                    red_area = np.sum (mask == 255)

                    # マスクを使って赤色部分のみを抽出する
                    red_only = cv2.bitwise_and(image, image, mask=mask)

                    # 二値化する
                    ret, binary_image = cv2.threshold(mask, 127, 255, cv2.THRESH_BINARY)

                    # 結果を表示する
                    #cv2.imshow('Original Image', image)
                    #cv2.imshow('Red Parts Only', red_only)
                    #cv2.imshow('Binary Image', binary_image)
                    #print(red_area)
                    #sleep(1)
                    
                    if red_area > 300:
                        straight_slow()
                        time.sleep(0.5)
                    
                    else:
                        right_slow()
                        time.sleep(0.3)

                    if close_distance_now < 30:
                        finish()
                        break
                    
                    if time.time() - stage3_start_time > 60:
                        finish()
                        break
                        
                cv2.waitKey(0)
                cv2.destroyAllWindows()
            
                
                
    except KeyboardInterrupt: #この４行はプログラム停止用
        pass
    finally:
        GPIO.cleanup()
        R_IN1_PWM.stop()
        R_IN2_PWM.stop()
        L_IN1_PWM.stop()
        L_IN2_PWM.stop()
       
if __name__=='__main__':
    print('start!')
    Readthread = threading.Thread(target=read())
    Recordthread = threading.Thread(target=record())
    Mainthread = threading.Thread(target=main())

    Readthread.start()
    Recordthread.start()
    Mainthread.start()
