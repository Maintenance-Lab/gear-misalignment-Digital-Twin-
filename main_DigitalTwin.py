import odrive       
import time         
import threading
import multiprocessing as mp
import math         
import random
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import csv
#from queue import Queue
from multiprocessing import Queue

# packeges for mpu6050 
import smbus            
from time import sleep          
import math
import RPi.GPIO as GPIO
import sys

# import for tsai and pytorch
from tsai.all import *
computer_setup()
import torch         
import pathlib

'''
set global vars and make Queue objects 
'''
     
stop = 0
q = Queue() # main queue object for data
qs = Queue() # queue for stop signal
q_predict = Queue(maxsize = 1) # queue for broadcasting predictions
q_ai_data = Queue() # window long queue object 

"""
get file name 
"""
filenum = input("name file : ")
time1 = time.time()
#df.to_csv('datadump/data_dump_{}_{}.csv'.format(time1,filenum))
#df.to_csv('datadump/data_dump_{}.csv'.format(filenum))
#df = pd.read_csv('datadump/file.csv',names = ['time','speed','epower'],header=0)


"""
main function odrive and 6050
"""
'''
seperate all interextion to odrive in one process atherwise the
connection can't be shared 
'''
def system(q):
    odrv0 = odrive.find_any()
    """
    fuctions 6050
    """
    # adress 6050
    Device_Address = 0x68
    # I2c adresses for 6050
    PWR_MGMT_1   = 0x6B
    SMPLRT_DIV   = 0x19
    CONFIG       = 0x1A
    GYRO_CONFIG  = 0x1B
    INT_ENABLE   = 0x38
    ACCEL_XOUT_H = 0x3B
    ACCEL_YOUT_H = 0x3D
    ACCEL_ZOUT_H = 0x3F
    GYRO_XOUT_H  = 0x43
    GYRO_YOUT_H  = 0x45
    GYRO_ZOUT_H  = 0x47
    

    bus = smbus.SMBus(1)
    
    # settings 6050
    def MPU_Init():
        bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)
        bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)
        bus.write_byte_data(Device_Address, CONFIG, 0)
        bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)
        bus.write_byte_data(Device_Address, INT_ENABLE, 1)
    
    # read function 6050
    def read_raw_data(addr):
            high = bus.read_byte_data(Device_Address, addr)
            low = bus.read_byte_data(Device_Address, addr+1)
            value = ((high << 8) | low)
            if(value > 32768):
                    value = value - 65536
            return value
    
    # test if odrive is connected    
    def test():
        print(type(odrv0))
        print(odrv0)
        print('test succes')
        odrv0.axis0.controller.input_vel = 0
    #testt = threading.Thread(target=test, args = ())
    #testt.start()
    
    def vel_change():
        '''
        verander snelheid 
        '''
        # the odrive has control modes 0 = voltage 1 = torque 2 = velocity 3 = position
        #odrv0.axis0.controller.config.control_mode = 8
        odrv0.axis0.controller.config.control_mode = 2 
        global stop

        speed = 0
        y = 0
        target = 10
        # variable for the speed of the motor 
        # sine wave form 1 to 6 in steps of 0.2
        while True:
            if qs.empty():
                if speed <= target:
                    target = 6
                    speed = speed + 0.2
                    #print('+')
                else:
                    target = 1
                    #print('-')
                    speed = speed - 0.2
                #print("speed = ",speed)
                #print(odrv0)
                odrv0.axis0.controller.input_vel = speed                 
                y += 0.8
                #print("y =", y)
                time.sleep(3)

            else:
                odrv0.axis0.controller.input_vel = 0
                break
        
        
    def pos_change(q):
        '''
        verander positie
        '''
        # the odrive has control modes 0 = voltage 1 = torque 2 = velocity 3 = position
        odrv0.axis1.controller.config.control_mode = 3 
        pos = 0

        while True:
            global stop
            if qs.empty():
                # set pos to random number between 0 and 4 
                pos = random.uniform(0,4)
                odrv0.axis1.controller.input_pos = pos
                '''
                give update
                '''
                timer = time.process_time()
                speed = odrv0.axis0.encoder.vel_estimate
                pos_estimate_rotate = odrv0.axis1.encoder.pos_estimate
                pos_setpoint = odrv0.axis1.controller.pos_setpoint
                
                print("{:.2f}: rot ={:.4f}, rotsetpoint = {:.4f} speed ={:.4f} ".format(timer,pos_estimate_rotate,pos_setpoint,speed))
                time.sleep(100)
            else:
                odrv0.axis1.controller.input_pos = 0
                break
            
    '''
    read power and save power
    '''
    def read_power(out_q):
        MPU_Init()
        print(ACCEL_XOUT_H,ACCEL_YOUT_H,ACCEL_ZOUT_H)
        while True:
            global stop
            if qs.empty():
                # read data odrive
                timer = time.process_time()
                epower = odrv0.axis0.controller.electrical_power
                ppower = odrv0.axis0.controller.electrical_power
                pos_estimate_main = odrv0.axis0.encoder.pos_estimate
                pos_estimate_rotate = odrv0.axis1.encoder.pos_estimate
                vel_estimate = odrv0.axis0.encoder.vel_estimate
                pos_setpoint = odrv0.axis1.controller.pos_setpoint
                
                # read data 6050
                gyro_x = read_raw_data(GYRO_XOUT_H)
                gyro_y = read_raw_data(GYRO_YOUT_H)
                gyro_z = read_raw_data(GYRO_ZOUT_H)

                accel_x = read_raw_data(ACCEL_XOUT_H)
                accel_y = read_raw_data(ACCEL_YOUT_H)
                accel_z = read_raw_data(ACCEL_ZOUT_H)
                #put data for the writer 
                out_q.put([[timer,pos_estimate_main,pos_estimate_rotate, epower,ppower,vel_estimate,pos_setpoint,gyro_x,gyro_y,gyro_z,accel_x,accel_y,accel_z]])
                # put the data for prediction to go to ai
                q_ai_data.put([[ppower,vel_estimate,gyro_x,gyro_y,gyro_z]])

                      
            else:
                break

    def ai_rot(q):
        while True:
            prediction = q_predict.get()
            pos_estimate_rotate = odrv0.axis1.encoder.pos_estimate
            pos = pos_estimate_rotate - prediction
            if pos <= 4:  
                odrv0.axis1.controller.input_pos = abs(pos)
                print("correcting pos to :", pos, "correct pos = ", pos_estimate_rotate, "estimated pos = ", prediction)
            else:
                print("pos meer dan 4")
            sleep(10)
    #print("system start")
    
    #print(odrv0)
    #print("found motor")
    #pos_estimate_main = odrv0.axis0.encoder.pos_estimate

    #print(pos_estimate_main)
    #print(type(odrv0))
    # start the motors and the data reader
    
    power_thread = threading.Thread(target=read_power, args = (q,))
    pos_thread = threading.Thread(target=pos_change, args = (q,))
    vel_thread = threading.Thread(target=vel_change, args = ())
    ai_rot_thread = threading.Thread(target=ai_rot, args = (q,))
    vel_thread.start()  
    power_thread.start()
    pos_thread.start()
    ai_rot_thread.start()

            
"""
data verwerking 
"""

def datavis(in_q):
    global df
    time.sleep(0.1)
    while True:
            file = open('datadump/data_dump_{}.csv'.format(filenum), 'a')
            write = csv.writer(file)
            while (in_q.qsize() != 0):
                write.writerow(in_q.get()[0])
            time.sleep(2)
            
def ai(q):
    # lead model
    learn = load_learner('/home/hva/Desktop/imports/export.pkl')
    # set window to the length of the models input 
    window = 300
    sample_size = 2
    while True:
        # run inference in a loop
        if q_ai_data.qsize() > 2*window:
            sample_size = math.floor(q_ai_data.qsize()/window)
            print(q_ai_data.qsize())
            # get window of data
            x_data = np.zeros((sample_size, 5, window))
            for j in range(sample_size):
                for i in range(window):
                    dat = q_ai_data.get()
                    x_data[j, :, i] = np.asarray(dat[0])
            #prediction = learn.predict(x_data)
            valid_probas, valid_targets, valid_preds = learn.get_X_preds(x_data, with_decoded=True)
            q_predict.put(torch.mean(valid_probas))
            print("predictions",valid_probas)
            print("mean predictions", torch.mean(valid_probas))
            #print(prediction)
            
        time.sleep(0.1)

'''
start treads
'''

datavis_thread = threading.Thread(target=datavis, args = (q,),daemon=True)
system_thread = mp.Process(target=system, args = (q,),daemon=True)
ai_thread = threading.Thread(target=ai, args = (q,),daemon=True)

datavis_thread.start()
system_thread.start()
ai_thread.start()
'''
stop alles als stop != 0 
'''
stop = input("input graag")
qs.put(5)
print('Done.')

print(q.qsize())
while q.qsize() != 0:
    print(q.qsize())
    time.sleep(0.5)
    
    
exit(0)

    