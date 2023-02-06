# -*- coding:utf-8 -*-
#精准延迟包：
from ctypes import windll
# 离鼠标最近距离
import math,time
# from math import atan
#自动获取com端口
import random
from serial.tools.list_ports import comports
#罗技驱动包：
from ctypes import CDLL, c_char_p
from os import path,system
#kmbox包：
from serial import Serial
##检测模式
import numpy as np
import cv2
#PID
import PID


# 读取自瞄配置文件,列表类型
ai_data1 = open('AI_Configuration.ini', "r", encoding="UTF-8")
AI_config1 = ai_data1.readlines()

kp = float(AI_config1[46][5:])
ki = float(AI_config1[47][5:])
kd = float(AI_config1[48][5:])

vfov=int(AI_config1[50][7:])
hfov=float(AI_config1[51][7:])


x轴一周像素 = int(AI_config1[53][9:])
y轴半周像素 = int(AI_config1[54][9:])

def HFOV(sub_s):
    hfov_=math.radians(hfov)
    单位角度像素点=x轴一周像素/math.radians(360)
    左右像素和=1920
    摄屏距离=左右像素和/2/math.tan(hfov_/2)
    转动角度=math.atan(sub_s/摄屏距离)
    转化移动像素=转动角度*单位角度像素点
    # print(转化移动像素)
    return 转化移动像素
def VFOV(sub_s):
    vfov_=math.radians(vfov)
    单位角度像素点=y轴半周像素/math.radians(180)
    上下像素和=1080
    摄屏距离=上下像素和/2/math.tan(vfov_/2)
    转动角度=math.atan(sub_s/摄屏距离)
    转化移动像素=转动角度*单位角度像素点
    # print(转化移动像素)
    return 转化移动像素

head_pid_x = PID.pid(0, kp, ki, kd)
body_pid_x = PID.pid(0, kp, ki, kd)

# head_pid_x.cmd_pid(目标值)
#为睡眠做准备
TimeBeginPeriod = windll.winmm.timeBeginPeriod
HPSleep = windll.kernel32.Sleep
TimeEndPeriod = windll.winmm.timeEndPeriod
#精准延迟
def Precise_delay(num):
    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    TimeBeginPeriod(1)
    HPSleep(int(num))  # 减少报错
    TimeEndPeriod(1)


#离鼠标最近距离
def Target_Distance(targetx,core_x,targety,core_y):
    a = float(targetx - core_x)#-412
    b = float(targety - core_y)#-284
    dis = math.sqrt((float(a) ** 2) + (float(b) **2))
    return dis

# mkbox函数：
# 自动获取com端口：
def km_Obtain_Com():

    """
    条件寻找端口(None)
    返回 "COM*" 串口
    """
    # 抄代码链接 自动寻找com口
    # https://blog.csdn.net/cp_srd/article/details/104741429
    try:
        num = random.randint(20, 30)
        # uart_class = list(serial.tools.list_ports.comports())
        uart_class = list(comports())
        if len(uart_class) <= 0:
            print("can't find the uart")
        else:
            for i in uart_class:
                uart_temp_str = str(i)
                uart_list = uart_temp_str.split()
                for j in uart_list:
                    # 查找"USB-SERIAL"的串口
                    if "USB-SERIAL" == j:
                        uart_num = uart_list[0]
                        # print(uart_num)
        return uart_num
    except:
        print("端口出错,检测到你未安装kmbox驱动，默认使用罗技驱动")


#实例化kmbox
try:
    com = km_Obtain_Com()
    km = Serial(com, 115200)
except:
    print("km出现重复调用，问题不大")

# 罗技函数：
# ↓↓↓↓↓↓↓↓↓ 调用ghub键鼠驱动 ↓↓↓↓↓↓↓↓↓
try:
    gm = CDLL(r'./Logitech_Dll/x64/ghub_device.dll')  # ghubdlldir
    gmok = gm.device_open()
    system('cls') 
    if not gmok:
        print('未安装ghub或者lgs驱动!!!')
    else:
        print('初始化成功!')
except FileNotFoundError:
    print('重要键鼠文件缺失')
    gmok = 0


gm.key_down.argtypes = [c_char_p]
gm.key_up.argtypes = [c_char_p]


def mouse_xy(x, y, abs_move = False):
    try:
        return gm.moveR(int(x), int(y), abs_move)
    except (NameError, OSError):
        print('键鼠调用严重错误!!!')


def mouse_down(key = 1):
    try:
        return gm.mouse_down(int(key))
    except (NameError, OSError):
        print('键鼠调用严重错误!!!')


def mouse_up(key = 1):
    try:
        return gm.mouse_up(int(key))
    except (NameError, OSError):
        print('键鼠调用严重错误!!!')


def scroll(num = 1):
    try:
        return gm.scroll(int(num))
    except (NameError, OSError):
        print('键鼠调用严重错误!!!')


def key_down(key=''):
    try:
        return gm.key_down(key.encode('utf-8'))
    except (NameError, OSError):
        print('键鼠调用严重错误!!!')


def key_up(key=''):
    try:
        return gm.key_up(key.encode('utf-8'))
    except (NameError, OSError):
        print('键鼠调用严重错误!!!')


def device_close():
    try:
        return gm.device_close()
    except (OSError, NameError):
        pass


# ↑↑↑↑↑↑↑↑↑ 调用ghub键鼠驱动 ↑↑↑↑↑↑↑↑↑
# poola1.submit(fn_returntarget,window_size, 1,Queue_tag, Queue_x_center, Queue_y_center, Queue_height, Queue_width, Queue_dis)


#截图模式
def Screenshot_Mode(ps_mode,pos_x,pos_y,mcx,mcy):
    window_size = (pos_x- mcx , pos_y - mcy , pos_x + mcx , pos_y + mcy)
    #目标检测中心点
    core_x = int((window_size[2]-window_size[0])/2)
    core_y = int((window_size[3]-window_size[1])/2)
    if ps_mode == 0:
        from mss import mss
        Screenshot_value = mss()
        #目标检测范围
        windows_size = {
            "left": window_size[0],
            "top": window_size[1],
            "width": window_size[2] - window_size[0],
            "height": window_size[3] - window_size[1],
        }
    elif ps_mode == 1:
        from d3dshot import create
        # pip install d3dshot -i https://pypi.tuna.tsinghua.edu.cn/simple
        Screenshot_value = create("numpy",frame_buffer_size = 100)
        #目标检测范围

    return window_size,core_x,core_y,Screenshot_value


#检测模式
def Detection_mode(test_mode,Screenshot_value,window_size):
    if test_mode == 0:
        img = Screenshot_value.grab(window_size)
        img = np.array(img)
        img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)#原版
    elif test_mode == 1:
        img = Screenshot_value.screenshot(region=window_size)
        #使用opencv删除一个通道
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

    return img


# 瞄准目标
def Aim_Target(Mouse_drive_mode,tag, target_x, target_y,core_x,
                        core_y, target_w,target_h, Body_offset_value_x, 
                        Body_offset_value_y, head_offset_value_x, 
                        head_offset_value_y,Window_border_x,
                        Window_border_y):

    # #head_pid_x.cmd_pid
    # x1 = head_pid_x.cmd_pid(HFOV(target_x-core_x-Window_border_x)+(target_w*head_offset_value_x))
    # y1 = VFOV(target_y-core_y-Window_border_y)+(target_h*head_offset_value_y)
    # #身体目标   
    # x2 = head_pid_x.cmd_pid(HFOV(target_x-core_x-Window_border_x)-(target_w*Body_offset_value_x))
    # y2 = VFOV(target_y-core_y-Window_border_y)-(target_h*Body_offset_value_y)
    
    # xx1 = head_pid_x.cmd_pid(x1)


    #头目标body_pid_x.cmd_pid
    x1 = head_pid_x.cmd_pid(HFOV(target_x-core_x-Window_border_x)+(target_w*head_offset_value_x))
    y1 = VFOV(target_y-core_y-Window_border_y)+(target_h*head_offset_value_y)
    #身体目标   
    x2 = head_pid_x.cmd_pid(HFOV(target_x-core_x-Window_border_x)-(target_w*Body_offset_value_x))
    y2 = VFOV(target_y-core_y-Window_border_y)-(target_h*Body_offset_value_y)

    # 判断头
    if Mouse_drive_mode == 0:
            # 判断头
        if tag == 1:
            mouse_xy(int(x1),int(y1))
            # mouse_xy(int(x1),int(y1))
        # 判断身体
        elif tag == 0:
            mouse_xy(int(x2),int(y2))
    elif Mouse_drive_mode == 1:
        if tag == 1:
            km.write(f"km.move({int(x1)},{int(y1)})\r\n".encode('utf-8'))
        # 判断身体
        elif tag == 0:
            km.write(f"km.move({int(x2)},{int(y2)})\r\n".encode('utf-8'))
    # Precise_delay(55)

#判断距离开火
def FireStarter(Mouse_drive_mode,tag,distance,Head_attack_range,Physical_attack_range,Window_border_y,Spot_fire):
    # tag = tag.get()
    # distance = distance.get()

    if Mouse_drive_mode == 0:
        if tag == 1:
            if distance < Head_attack_range+Window_border_y:
                mouse_down(1)
                Precise_delay(Spot_fire)
                mouse_up(1)

        elif tag == 0:
            if distance > Head_attack_range and distance < Physical_attack_range+Window_border_y:
                mouse_down(1)
                Precise_delay(Spot_fire)
                mouse_up(1)
    elif Mouse_drive_mode == 1:
        if tag == 1:
            if distance < Head_attack_range+Window_border_y:
                km.write(f"km.left(1)\r\n".encode('utf-8'))
                Precise_delay(Spot_fire)
                km.write(f"km.left(0)\r\n".encode('utf-8'))
        elif tag == 0:
            if distance > Head_attack_range and distance < Physical_attack_range+Window_border_y:
                km.write(f"km.left(1)\r\n".encode('utf-8'))
                Precise_delay(Spot_fire)
                km.write(f"km.left(0)\r\n".encode('utf-8'))




