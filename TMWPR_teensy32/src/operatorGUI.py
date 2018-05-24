# TGGS-KMUTNB
# Embedded Software System Project of 2017
# Software System Engineering (SSE)
# Author: Mix, Bekty

import sys
import random
import math
import threading
import time

import serial
import pygame
import pygame.gfxdraw
from pygame.locals import *


pygame.init()
pygame.joystick.init()

joysticks = [pygame.joystick.Joystick(x)
             for x in range(pygame.joystick.get_count())]
myjoystick = joysticks[0]
myjoystick.init()
# set color values
black = 0, 0, 0
white = 255, 255, 255
grey = 125, 125, 125
yellow = 255, 255, 0
red = 255, 0, 0
blue = 0, 0, 255

# screen size
width = 800
height = 800

# set screen
screen = pygame.display.set_mode((width, height))
screen.fill(black)
ref_screen = pygame.surface.Surface((100, 100), pygame.SRCALPHA)
ref_screen2 = pygame.surface.Surface((width/2, height/2), pygame.SRCALPHA)
robot_surf = pygame.surface.Surface((width/2, height/2), pygame.SRCALPHA)
robot_rect = robot_surf.get_rect()
compass_surf = pygame.surface.Surface((100, 100), pygame.SRCALPHA)
compass_rect = compass_surf.get_rect()
command_surf = pygame.surface.Surface((width/2, height/2), pygame.SRCALPHA)
command_rect = command_surf.get_rect()


# set clock
clock = pygame.time.Clock()

# font setup
font = pygame.font.Font(None, 20)
flag = 1

# set serial
ser = serial.Serial('/dev/ttyUSB0', 19200)
ser.timeout = 0
ser.write_timeout = 0.1
complete_data = []
crctable = []
wheel_FL = 0
wheel_FR = 0
wheel_RL = 0
wheel_RR = 0
dist_F = 0
dist_B = 0
dist_L = 0
dist_R = 0
dist = [0, 0, 0, 0]
wheel = [0, 0, 0, 0]
deg = 0
data_out = [b'\xff', b'\xf2', b'\x01', b'\x0A', b'\x28',
            b'\xF5', b'\xD8', b'\x00', b'\x00', b'\x00', b'\x20', b'\x20']
data_ready = False
vx = 0.26
vy = -0.26
wz = 0
ref = 0


def send_data(vx, vy, wz, ref):
    global data_out
    vx_int = int(vx*10000)
    vy_int = int(vy*10000)
    wz_int = int(wz*10000)
    ref_int = int((ref)*10)
    data_out[3] = ((vx_int & 0xff00) >> 8).to_bytes(1, 'big')
    data_out[4] = (vx_int & 0x00ff).to_bytes(1, 'big')
    data_out[5] = ((vy_int & 0xff00) >> 8).to_bytes(1, 'big')
    data_out[6] = (vy_int & 0x00ff).to_bytes(1, 'big')
    data_out[7] = ((wz_int & 0xff00) >> 8).to_bytes(1, 'big')
    data_out[8] = (wz_int & 0x00ff).to_bytes(1, 'big')
    data_out[9] = ((ref_int & 0xff00) >> 8).to_bytes(1, 'big')
    data_out[10] = (ref_int & 0x00ff).to_bytes(1, 'big')

    crc = compute_crc8(data_out[:11])
    data_out[11] = ord(crc).to_bytes(1, 'big')
    # print((data_out))
    ser.write(ord(x) for x in data_out)

def compute_crc8(data):
    crc = 0
    for i in range(0, len(data)):
        dat = (ord(data[i]) ^ crc)
        # print(dat,',',crc)
        dat = dat & 0x00ff
        # print(dat)
        crc = ord(crctable[dat])
    return chr(crc)

def calulate_table_crc8():
    generator = 0x1D
    for divident in range(0, 256):
        currByte = divident
        for bit in range(0, 8):
            if ((currByte & 0x80) != 0):
                currByte <<= 1
                currByte ^= generator
            else:
                currByte <<= 1
        currByte = currByte & 0x00ff
        crctable.append(chr(currByte))

def read_from_port(ser):
    global data_ready, wheel, deg, dist, flag
    recieve_data = []
    data_ind = 0
    while ser.is_open and flag:
        rec = ser.read()
        recieve_data.append(rec)
        if (data_ind >= 14) and (recieve_data[data_ind-14] == b'\xff') and (recieve_data[data_ind-13] == b'\xf2'):
            complete_data = recieve_data[data_ind-14:data_ind]
            if(ord(complete_data[13]) == ord(compute_crc8(complete_data[:13]))):
                dist[2] = (ord(complete_data[3]))
                dist[1] = (ord(complete_data[4]))
                dist[0] = (ord(complete_data[5]))
                dist[3] = (ord(complete_data[6]))
                deg = ord(complete_data[7])
                deg <<= 8
                deg = deg+ord(complete_data[8])
                if (ord(complete_data[9]) >> 7) == 0:
                    wheel[0] = (ord(complete_data[9]) & 0x007f)
                else:
                    wheel[0] = (ord(complete_data[9]) & 0x007f) - 128

                if (ord(complete_data[10]) >> 7) == 0:
                    wheel[1] = (ord(complete_data[10]) & 0x007f)
                else:
                    wheel[1] = (ord(complete_data[10]) & 0x007f) - 128

                if (ord(complete_data[11]) >> 7) == 0:
                    wheel[2] = (ord(complete_data[11]) & 0x007f)
                else:
                    wheel[2] = (ord(complete_data[11]) & 0x007f) - 128

                if (ord(complete_data[12]) >> 7) == 0:
                    wheel[3] = (ord(complete_data[12]) & 0x007f)
                else:
                    wheel[3] = (ord(complete_data[12]) & 0x007f) - 128

                data_ready = True
                # print(deg/10)
            else:
                complete_data.clear()
            data_ind = 0
            recieve_data.clear()
        if data_ind > 50:
            data_ind = 0
            recieve_data.clear()
        data_ind = data_ind+1
    ser.close()

def show_information():
    global dist, wheel, ref_deg
    blockdist1 = 'Front Obstacle '
    blockdist2 = 'Rear Obstacle  '
    blockdist3 = 'Right Obstacle '
    blockdist4 = 'Left Obstacle  '
    block_unit = 'CM'

    if dist[1] < 100:
        dist_1 = ' {:3d}'.format(dist[1]) + ' ' + block_unit
    else:
        dist_1 = ' not detected'
    
    if dist[3] < 100:
        dist_2 = ' {:3d}'.format(dist[3]) + ' ' + block_unit
    else:
        dist_2 = ' not detected'
    
    if dist[2] < 100:
        dist_3 = ' {:3d}'.format(dist[2]) + ' ' + block_unit
    else:
        dist_3 = ' not detected'
    
    if dist[0] < 100:
        dist_4 = ' {:3d}'.format(dist[0]) + ' ' + block_unit
    else:
        dist_4 = ' not detected'
    
    # dist_2 = ' {:3.2f}'.format(dist[3]) + ' ' + block_unit
    # dist_3 = ' {:3.2f}'.format(dist[2]) + ' ' + block_unit
    # dist_4 = ' {:3.2f}'.format(dist[0]) + ' ' + block_unit

    blockwheel1 = 'Front Wheel  :'
    blockwheel2 = 'Rear Wheel   :'
    blockwheel3 = 'Right Wheel  :'
    blockwheel4 = 'Left Wheel   :'
    blockwheel_unit = 'RPM'

    wheel_1 = ' {:4d}'.format(wheel[0]) + ' ' + blockwheel_unit    
    wheel_2 = ' {:4d}'.format(wheel[1]) + ' ' + blockwheel_unit
    wheel_3 = ' {:4d}'.format(wheel[2]) + ' ' + blockwheel_unit
    wheel_4 = ' {:4d}'.format(wheel[3]) + ' ' + blockwheel_unit

    ref = 'Compass : ' + '{:5.2f}'.format(deg/10) + '\u00b0 to N'
    motion_data = compute_motion(wheel)


    textSpeed = 'Vx: ' + ' {:3.2f}'.format(motion_data[0]) + ' m/s'
    textSpeed = textSpeed + ', Vy: ' + '{:3.2f}'.format(motion_data[1]) + ' m/s'
    textSpeed = textSpeed + ', Wz: ' + '{:3.2f}'.format(motion_data[2]) + ' rad/s'
   
    
    # print(motion_data)
    # text1 = font.render(dist_1 + ', ' + dist_2 + ', ' + dist_3 + ', '+ dist_4 , True, black, white)
    ## info block
    text_block_info1 = font.render(blockdist1, True, white, black)
    text_block_info1_Rect = text_block_info1.get_rect()
    text_block_info1_Rect.centerx = screen.get_rect().center[0] + 250
    text_block_info1_Rect.centery = screen.get_rect().center[1] + 300

    text_block_info2 = font.render(blockdist2, True, white, black)
    text_block_info2_Rect = text_block_info2.get_rect()
    text_block_info2_Rect.topleft = text_block_info1_Rect.bottomleft

    text_block_info3 = font.render(blockdist3, True, white, black)
    text_block_info3_Rect = text_block_info3.get_rect()
    text_block_info3_Rect.topleft = text_block_info2_Rect.bottomleft

    text_block_info4 = font.render(blockdist4, True, white, black)
    text_block_info4_Rect = text_block_info4.get_rect()
    text_block_info4_Rect.topleft = text_block_info3_Rect.bottomleft

    

    text1 = font.render(dist_1, True, white, black)
    textRect = text1.get_rect()
    textRect.topleft = text_block_info1_Rect.topright

    
    text2 = font.render(dist_2, True, white, black)
    textRect2 = text2.get_rect()
    textRect2.topleft = textRect.bottomleft

    text3 = font.render(dist_3, True, white, black)
    textRect3 = text3.get_rect()
    textRect3.topleft = textRect2.bottomleft

    text4 = font.render(dist_4, True, white, black)
    textRect4 = text4.get_rect()
    textRect4.topleft = textRect3.bottomleft



    text5 = font.render(ref, True, white, black)
    textRect5 = text5.get_rect()
    # textRect5.center = screen.get_rect().center 
    textRect5.center = (screen.get_rect().center[0]+300, screen.get_rect().center[1]-280)

    text6 = font.render(str(ref_deg)+'\u00b0', True, black, yellow)
    textRect6 = text6.get_rect()
    textRect6.center = screen.get_rect().center

    text_block_wheel1 = font.render(blockwheel1, True, white, black)
    text_block_wheel1_Rect = text_block_wheel1.get_rect()
    text_block_wheel1_Rect.centerx = screen.get_rect().center[0] + 245
    text_block_wheel1_Rect.centery = screen.get_rect().center[1] + 200

    text_block_wheel2 = font.render(blockwheel2, True, white, black)
    text_block_wheel2_Rect = text_block_wheel2.get_rect()
    text_block_wheel2_Rect.topleft = text_block_wheel1_Rect.bottomleft

    text_block_wheel3 = font.render(blockwheel3, True, white, black)
    text_block_wheel3_Rect = text_block_wheel3.get_rect()
    text_block_wheel3_Rect.topleft = text_block_wheel2_Rect.bottomleft

    text_block_wheel4 = font.render(blockwheel4, True, white, black)
    text_block_wheel4_Rect = text_block_wheel4.get_rect()
    text_block_wheel4_Rect.topleft = text_block_wheel3_Rect.bottomleft

    text_wheel1 = font.render(wheel_1, True, white, black)
    text_wheelRect = text_wheel1.get_rect()
    text_wheelRect.topleft = text_block_wheel1_Rect.topright

    text_wheel2 = font.render(wheel_2, True, white, black)
    text_wheelRect2 = text_wheel2.get_rect()
    text_wheelRect2.topleft = text_wheelRect.bottomleft

    text_wheel3 = font.render(wheel_3, True, white, black)
    text_wheelRect3 = text_wheel3.get_rect()
    text_wheelRect3.topleft = text_wheelRect2.bottomleft

    text_wheel4 = font.render(wheel_4, True, white, black)
    text_wheelRect4 = text_wheel4.get_rect()
    text_wheelRect4.topleft = text_wheelRect3.bottomleft


    textw = font.render(textSpeed, True, black, white)
    textRectw = textw.get_rect()
    textRectw.bottomright = screen.get_rect().bottomright

    # textRect.bottomright = screen.get_rect().bottomright
    screen.blit(text_block_info1, text_block_info1_Rect)
    screen.blit(text_block_info2, text_block_info2_Rect)
    screen.blit(text_block_info3, text_block_info3_Rect)
    screen.blit(text_block_info4, text_block_info4_Rect)
    
    screen.blit(text_block_wheel1, text_block_wheel1_Rect)
    screen.blit(text_block_wheel2, text_block_wheel2_Rect)
    screen.blit(text_block_wheel3, text_block_wheel3_Rect)
    screen.blit(text_block_wheel4, text_block_wheel4_Rect)
    
    screen.blit(text_wheel1, text_wheelRect)
    screen.blit(text_wheel2, text_wheelRect2)
    screen.blit(text_wheel3, text_wheelRect3)
    screen.blit(text_wheel4, text_wheelRect4)

    screen.blit(text1, textRect)
    screen.blit(text2, textRect2)
    screen.blit(text3, textRect3)
    screen.blit(text4, textRect4)
    screen.blit(textw, textRectw)
    screen.blit(text5, textRect5)
    screen.blit(text6, textRect6)
    
    # /print(ref)

def compute_motion(wheel):
    # linear velo X = (wheel_front_left + wheel_front_right + wheel_rear_left + wheel_rear_right) * (WHEEL_RADIUS/4)
    vx = (wheel[0] + wheel[1] + wheel[2] + wheel[3]) * (0.05/4)
    # linear velo Y = ( -wheel_front_left + wheel_front_right + wheel_rear_left - wheel_rear_right) * (WHEEL_RADIUS/4)
    vy = (-wheel[0] + wheel[1] + wheel[2] - wheel[3]) * (0.05/4)
    # angular velo Z = ( -wheel_front_left + wheel_front_right - wheel_rear_left + wheel_rear_right) * (WHEEL_RADIUS/(4 * (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH)))
    wz = (-wheel[0] + wheel[1] - wheel[2] + wheel[3]) * (0.05/4)
    return [vx/10, vy/10, wz/10]


# initialize CRC8 Table
calulate_table_crc8()

# setup thread
thread_serial = threading.Thread(target=read_from_port, args=(ser,))
thread_serial.start()

vx = 0
vy = 0
wz = 0
rot = 0
hold_down_flag = False
ref_deg = 0
while flag:
    # Set FPS
    clock.tick(60)  # FPS = 60

    # Process events
    for event in pygame.event.get():
        # QUIT
        if event.type == pygame.QUIT:
            send_data(0, 0, 0, 0)
            flag = 0
        # exit with escape
        if event.type == pygame.KEYUP and event.key == pygame.K_ESCAPE:
            send_data(0, 0, 0, 0)
            flag = 0
        if event.type == pygame.JOYBUTTONDOWN:
            if myjoystick.get_button(7):
                ref_deg = ref_deg+45
            if myjoystick.get_button(5):
                ref_deg = ref_deg-45

        if event.type == pygame.JOYAXISMOTION:
            vx = myjoystick.get_axis(0)
            if(abs(vx) < 0.2):
                vx = 0
            else:
                vx = vx * 0.36

            vy = -myjoystick.get_axis(1)
            if(abs(vy) < 0.2):
                vy = 0
            else:
                vy = vy * 0.36

            wz = -myjoystick.get_axis(3)
            if(abs(wz) < 0.3):
                wz = 0
            else:
                wz = wz * 1.33

    # Set Reference Angle based on button pressed
    if myjoystick.get_button(6):
        ref_deg = ref_deg+1
    if myjoystick.get_button(4):
        ref_deg = ref_deg-1
    if myjoystick.get_button(2):
        ref_deg = 0
    if ref_deg > 360:
        ref_deg = ref_deg - 360
    elif ref_deg < 0:
        ref_deg = 360+ref_deg

    screen.fill(black)
    robot_surf = pygame.transform.rotate(screen, 0)
    command_surf = pygame.transform.rotate(ref_screen2, 0)
    compass_surf = pygame.transform.rotate(ref_screen, 0)
    TRANSPARENT = black + (0,)

    robot_surf.fill(TRANSPARENT)
    compass_surf.fill(TRANSPARENT)

    pygame.draw.rect(robot_surf, yellow, Rect(
        int(width/2)-55, int(height/2)-97, 110, 160))  # Sun
    pygame.draw.rect(robot_surf, yellow, Rect(
        int(width/2)-55, int(height/2)+66, 110, 30))  # Sun

    cmd_center_x = int(width/4)
    cmd_center_y = int(height/4)
    padding = 160

    if abs(vx) > 0 or abs(vy) > 0:
        pygame.draw.polygon(command_surf, white,
                            [[cmd_center_x+padding, cmd_center_y], [cmd_center_x-10+padding, cmd_center_y+15],
                             [cmd_center_x+20+padding, cmd_center_y], [cmd_center_x-10+padding, cmd_center_y-15]])
    if(dist[1] < 60):
        draw_dist = dist[1] * 5
        pygame.draw.rect(robot_surf, red, Rect(
            int(width/2)-28, int(height/2)-(97+draw_dist), 56, 3))  # Sun

    if(dist[3] < 60):
        draw_dist = dist[3] * 5
        pygame.draw.rect(robot_surf, red, Rect(
            int(width/2)-28, int(height/2)+(93+draw_dist), 56, 3))  # Sun

    if(dist[0] < 60):
        draw_dist = dist[0] * 5
        pygame.draw.rect(robot_surf, red, Rect(
            int(width/2)-(55+draw_dist), int(height/2)-(25), 3, 56))  # Sun

    if(dist[2] < 60):
        draw_dist = dist[2] * 5
        pygame.draw.rect(robot_surf, red, Rect(
            int(width/2)+(52+draw_dist), int(height/2)-(25), 3, 56))  # Sun

    center_x = int(compass_rect.width/2)
    center_y = int(compass_rect.height/2)
    pygame.draw.polygon(compass_surf, red,
                        [[center_x, center_y], [center_x+15, center_y+10],
                         [center_x, center_y-20], [center_x-15, center_y+10]])

    # pygame.gfxdraw.aacircle(compass_surf,center_x,center_y,30,white)
    if vx == 0:
        if vy == 0:
            theta = 0
        elif vy > 0:
            theta = 90
        else:
            theta = 270
    else:
        theta = math.atan2(vy, vx) * 180/math.pi
    
    if theta < 0:
        theta = 360+theta
    
    pygame.draw.circle(compass_surf, white, [center_x, center_y], 35, 4)
    pygame.draw.line(compass_surf, white, [
                     center_x, center_y-27], [center_x, center_y - 43], 3)

    if flag:
        try:
            print('tried to send data')
            send_data(vx, vy, wz, ref_deg)
        except:
            print('data failed')
    else:
        try:
            send_data(0, 0, 0, 0)
        except:
            print('')

    if data_ready:
        data_ready = False

    robot_surf = pygame.transform.rotate(robot_surf, -(ref_deg + 90))
    command_surf = pygame.transform.rotate(command_surf, theta)
    compass_surf = pygame.transform.rotate(compass_surf, -deg/10)

    cmdwidth, cmdheight = command_surf.get_size()
    rwidth, rheight = robot_surf.get_size()
    cwidth, cheight = compass_surf.get_size()

    screen.blit(robot_surf, ((width-rwidth)/2, (height-rheight)/2))
    screen.blit(command_surf, ((width-cmdwidth)/2, (height-cmdheight)/2))
    screen.blit(compass_surf, (((width-cwidth)/2)+299, ((height-cheight)/2)-340))

    show_information()
    # update screen
    pygame.display.flip()

pygame.quit()
