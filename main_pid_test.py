from smbus2 import SMBus
import time
import nympy as np
import camera

#I2C - init
address_sensor = 0x40
register_sensor = 0
address_styr = 0x69
register_styr = 0
read_bytes_sensor = 4
read_bytes_styr = 2
styr = [255, 0]                #[PWM GAFFEL]
#u_send =[0, 0]
u_send = 0
num_bytes = 0
kp = 0
kd = 0
error = 0
autonom = 0
turn_left = 0
turn_right = 0
u_left = 0
u_right = 0
last_error = 0
distance = 0 #distance = speed x Time
td = 0
speed = 0
turn_variable = 0
pallet = 0
max_u = 7
min_u = -7

# u = Kp*e + Kd*Td
# Td = ek - ek-1 / ds
# u = kp*(e + (kd*Td / Kp))

def read_bytes(addr, num_bytes):                        # läser read_bytes_sensor antal bytes från address address_sensor
    data = bus.read_i2c_block_data(addr, 0, num_bytes)  #[TEJP WHEELSPEED-R WHEELSPEED-L IR-SENSOR Turn variable MANUELLT/AUTO distance]
    print(data)
    return data

def write_bytes(addr, out_data):
    bus.write_i2c_block_data(addr, 0, out_data)
    return

def set_bit(value, bit):
    return value | (1<<bit)

def clear_bit(value, bit):
    return value & ~(1<<bit)

# MAIN programmet
with SMBus(1) as bus:
    
    while True:
        #Should read manuellt/auto variable to see if we should do pid or not
        #PID STUFF
        #----------------------------------------------------------
        PID_Data = read_bytes(address_sensor, read_bytes_sensor)
        error = PID_Data[0]
        autonom = PID_Data[5]
        error = error / 100
        error = np.arcsin(error) # reglerfelet i radianer

        td = (error - last_error) / distance # td kommer ha meter som enhet då det är en skillnad i reglerfelet
        u = kp*(error + kd/kp * td)
        u_left = speed(1 + u)
        u_right = speed(1 - u)
        if(u_left >= max_u):
            u_left = max_u
        elif(u_right >= max_u):
            u_right = max_u
        elif(u_right >= max_u and u_left >= max_u):
            u_left = max_u
            u_right = max_u
            
        elif(u_right <= min_u):
            u_right = min_u
        elif(u_left <= min_u):
            u_left = min_u
        elif(u_right <= min_u and u_left <= min_u):
            u_left = min_u
            u_right = min_u
        
        #u_send[0] = u_left
        #u_send[1] = u_right
        u_send |= u_left << 5 | u_right << 2
        if(u_left > 0):
            u_send |= 2
        if(u_right > 0):
            u_send |= 1
        if(u_left < 0):
            u_send &= 2
        if(u_right):
            u_send &= 1
        
        last_error = error
        write_bytes(address_styr, u_send) # vi ska försöka skicka 1 byte som innehåller info för pwm för båda två hjulen
        #MSB -- första 3 bits är vänster hjul pwm, nästa 3 bits är pwm höger hjul, 2 bitar är direction, först direction vänster sen direction höger -- LSB
        #----------------------------------------------------------
        #Handle turn variable
        #----------------------------------------------------------
        turn_variable = PID_Data[4]  #read variable from sensor module if we are approaching a turn 
        match turn_variable:
            case 130: # means that we are at the last pallet station and turning left into the lane 
                #turn_left = 1
                u_send = 0
                write_bytes(address_styr, u_send)
                time.sleep(0.1)
                #send data to turn 90 degress
                turn_left = 1
                write_bytes(address_styr, turn_left)
            case 193: #this means that we are going from a pallet station to the lane, we want to turn left to turn back to the start
                #turn_left = 1
                u_send = 0
                write_bytes(address_styr, u_send)
                time.sleep(0.1)
                #send data to turn 90 degress
                turn_left = 1
                write_bytes(address_styr, turn_left)
            case 67:
                #turn_right = 1 #means that we are on the lane and a turn into a pallet station is detected
                u_send = 0
                write_bytes(address_styr, u_send)
                time.sleep(0.1)
                #send data to turn 90 degress
                turn_right = 1
                write_bytes(address_styr, turn_right)
            case _:
                turn_left = 0
                turn_right = 0
                print("error in turn variable case")
        
        #----------------------------------------------------------
        #Pallet cases
        #----------------------------------------------------------
        pos = find_pallet(pallet)
        if(pos != 1 or pos != 2 or pos != 3):
            print(pos)
        #match pos:
            #case 1:
                #if this case we need to move pallet nr 3 and pallet nr 2 to offloading station
            #case 2:
                #if this case we need to move pallet nr 3 to offloading station and grab pallet nr 2 back to start
            #case 3:
                #if this case we just need to grab pallet nr 3 back to start
            #case _:
                #print("error in pallet cases")
        #print(styr[0])
        time.sleep(0.1)