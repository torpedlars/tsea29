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
distance = 0 
td = 0
speed = 0
turn_variable = 0
pallet = 0 # värde på detta får vi från gränssnittet
max_u = 7
min_u = -7
turned_into_station = 0
station = 0 # Värde på detta får vi från gränssnittet
u_send_stop = 0
at_station = 0
fork_position = 0
u_send_short_forward = 0
u_send_short_reverse = 0
start_stop = 0

# u = Kp*e + Kd*Td
# Td = ek - ek-1 / ds
# u = kp*(e + (kd*Td / Kp))

def read_bytes(addr, num_bytes):                        # läser read_bytes_sensor antal bytes från address address_sensor
    data = bus.read_i2c_block_data(addr, 0, num_bytes)  #[TEJP WHEELSPEED-R WHEELSPEED-L IR-SENSOR Turn variable MANUELLT/AUTO Pins]
    print(data)
    return data

def write_bytes(addr, out_data):
    bus.write_i2c_block_data(addr, 0, out_data)
    return

def calculatePID(PIDdata):
    global last_error
    PIDData = read_bytes(address_sensor, read_bytes_sensor)
    error = PIDData[0]

    #distance = PIDData[7] * 5
    #distance = 1
    #error = error / 100
    #error = np.arcsin(error) # reglerfelet i radianer

    td = (error - last_error) #/ distance # td kommer ha meter som enhet då det är en skillnad i reglerfelet
    u = int(kp*error + kd*td)
    u_left = speed + u
    u_right = speed - u

    if(u_left > 120):
        u_left = 120
    if(u_right > 120):
        u_right = 120
    if(u_left < -120):
        u_left = -120
    if(u_right < -120):
        u_right = -120
    
    last_error = error
    return u_send

# MAIN programmet
with SMBus(1) as bus:
    
    while True:
        #Should read manuellt/auto variable to see if we should do pid or not
        PID_Data = read_bytes(address_sensor, read_bytes_sensor)
        while(PID_Data[6] == 1):
            #PID STUFF
            #----------------------------------------------------------
            PID_Data = read_bytes(address_sensor, read_bytes_sensor)
            motor_speed = calculatePID(PID_Data)
            motor_speed[0] |= 1
            motor_speed[1] |= 1
            write_bytes(address_styr, motor_speed)
            
            #testa att göra en 90 graders sväng
            #
            """u_left_turn = 73
            write_bytes(address_styr, u_left_turn)

            u_right_turn = 74"""

   
           