from smbus2 import SMBus
import time
import numpy as np
from camera import *

#I2C - init
global address_sensor
address_sensor = 0x40
global register_sensor
register_sensor = 0
global address_styr
address_styr = 0x69
global register_styr
register_styr = 0
global read_bytes_sensor
read_bytes_sensor = 9
global read_bytes_styr
read_bytes_styr = 2
global last_error
last_error = 0
global standard_fork_pos
standard_fork_pos = 3
# u = Kp*e + Kd*Td
# Td = ek - ek-1 / ds
# u = kp*(e + (kd*Td / Kp))


def read_bytes(addr, num_bytes):                        # läser read_bytes_sensor antal bytes från address address_sensor
    data = bus.read_i2c_block_data(addr, 0, num_bytes)  #[TEJP WHEELSPEED-R WHEELSPEED-L IR-SENSOR Turn variable MANUELLT/AUTO Pins]
    #data = 1
    if(data[0] > 50):
        data[0] = data[0] - 256
    if(data[8] > 50):
        data[8] = data[8] - 256
    print(data)
    return data

def write_bytes(addr, out_data):
    bus.write_i2c_block_data(addr, 0, out_data)
    return

def calculatePID(fork_pos):
    global last_error

    u_send = []
    u_max = 80
    u_min = 5
    kd = 20 #6
    kp = 2 #2
    error = PID_Data[0]
    #error = (error + 5) - 5
    speed = 20
    #distance = PIDData[7] * 5
    #distance = 1
    #error = error / 100
    #error = np.arcsin(error) # reglerfelet i radianer

    #td = (error - last_error) #/ distance # td kommer ha meter som enhet då det är en skillnad i reglerfelet
    u = int((kp*error + kd*(error-last_error))/2)
    #print(td)
    #print(u)
    u_left = speed + u
    u_right = speed - u


    if(u_left > u_max):
        u_left = u_max
    if(u_right > u_max):
        u_right = u_max
    if(u_left < u_min):
        u_left = u_min
    if(u_right < u_min):
        u_right = u_min

    u_left = u_left*2
    u_right = u_right*2
    u_left |= 1
    u_right |= 1
    u_send.append(u_right)
    u_send.append(u_left)
    u_send.append(fork_pos)
    #print(error)
    #print(u_send)
    last_error = error
    return u_send

def stop_truck(fork_pos):
    write_bytes(address_styr, [0, 0, fork_pos])
    time.sleep(0.8)
    return

def turn_ninty_degree_left(in_time, fork_pos): #30 31 har vi mätt upp till bra värden innan
    write_bytes(address_styr, [30, 31, fork_pos])
    time.sleep(in_time)
    stop_truck(fork_pos)
    return

def turn_ninty_degree_right(in_time, fork_pos):
    write_bytes(address_styr, [31, 30, fork_pos])
    time.sleep(in_time)
    stop_truck(fork_pos)
    return

def turn_one_eighty_left(in_time, fork_pos):
    turn_ninty_degree_left(in_time, fork_pos)
    turn_ninty_degree_left(in_time, fork_pos)
    return

def turn_one_eighty_right(in_time, fork_pos):
    turn_ninty_degree_right(in_time, fork_pos)
    turn_ninty_degree_right(in_time, fork_pos)
    return

def short_forward(in_time, fork_pos):
    write_bytes(address_styr, [31, 31, fork_pos])
    time.sleep(in_time)
    stop_truck(fork_pos)
    return

def short_reverse(in_time, fork_pos):
    write_bytes(address_styr, [30, 30, fork_pos])
    time.sleep(in_time)
    stop_truck(fork_pos)
    return

def set_fork_state_to(level):
    write_bytes(address_styr, [0, 0, level])
    time.sleep(2)
    return

def use_pid_control(fork_pos):
    u_send = calculatePID(fork_pos)
    write_bytes(address_styr, u_send)
    time.sleep(0.08)


# MAIN programmet
with SMBus(1) as bus:
    time.sleep(1)
    while True:
        #Should read manuellt/auto variable to see if we should do pid or not
        PID_Data = read_bytes(address_sensor, read_bytes_sensor)
        #PID_Data = [1, 1, 1, 1, 1, 1, 1, 1, 1]  #TEMP! Use line above!
        turn_variable = 0
        pallet = 10 # värde på detta får vi från gränssnittet
        turned_into_station = 0
        station = 2 # Värde på detta får vi från gränssnittet
        at_station = 0
        fork_position = 3
        pos = 0
        turn_left = 0
        turn_right = 0
        short_forward_left = 31
        short_forward_right = 31
        short_reverse_left = 30
        short_reverse_right = 30
        left_wheel_left_90_turn = 30
        right_wheel_left_90_turn = 31
        left_wheel_right_90_turn = 31
        right_wheel_right_90_turn = 30
        start_stop = 0
        time.sleep(0.1)
        write_bytes(address_styr, [0,0,3])
        no_turn = 0
        photo = 0
        while(PID_Data[6] == 1):
            #PID STUFF
            #----------------------------------------------------------
            PID_Data = read_bytes(address_sensor, read_bytes_sensor)                               ### DENNA RADEN MÅSTE ANVÄNDAS!
            while(PID_Data[3] == 1):
                stop_truck(fork_position)
                PID_Data = read_bytes(address_sensor, read_bytes_sensor)
            if (PID_Data[6] == 0):
                break
            # vi ska försöka skicka 1 byte som innehåller info för pwm för båda två hjulen
            use_pid_control(fork_position)
            #MSB -- första 3 bits är vänster hjul pwm, nästa 3 bits är pwm höger hjul, 2 bitar är direction, först direction vänster sen direction höger -- LSB
            #----------------------------------------------------------
            #Handle turn variable
            #----------------------------------------------------------
            #PID_Data = read_bytes(address_sensor, read_bytes_sensor)
            
            if(PID_Data[4] == 130):
                stop_truck(fork_position)
                print("Found Left turn")
                if(no_turn == 1):
                    fork_position = 3
                    print("on our way back to start stop")
                    short_forward(0.8,fork_position)
                    use_pid_control(fork_position)
                else:
                    turn_ninty_degree_left(1.7, fork_position)
                    turn_left = 1
                    print("turning left")
                    turn_right = 0
                    short_forward(0.8, fork_position)
                    use_pid_control(fork_position)
            elif(PID_Data[4] == 193):
                stop_truck(fork_position)
                print("Found 3 way junction")
                if(turned_into_station == 1): #if this then we have arrived at station and need to take the pallet
                    at_station = 1 #set variable to we know we're at the station and should move forks into correct position depending on variable pos
                    turned_into_station = 0 #reset variable so we enter else when going back to beginning
                    print("At station")

                elif(start_stop == 1):
                    print("found start and stop")
                    pos = 4
                    start_stop = 0
                    
                elif(photo == 1):
                    pos = find_pallet(pallet)
                    short_forward(0.8, fork_position)
                    #pos = 1 #find_pallet(pallet)

                    if(pos != 1 and pos != 2 and pos != 3):
                        print(f"{pos} not found")
                        time.sleep(1)
                        #pos = 1 #find_pallet(pallet)
                        pos = find_pallet(pallet)
                    short_forward(0.8, fork_position)
                    PID_Data = read_bytes(address_sensor, read_bytes_sensor)
                    time.sleep(0.1)
                    use_pid_control(fork_position)
                    
                else:
                    print("turning left on 3 way junction")
                    turn_ninty_degree_left(1.7, fork_position)
                    turn_left = 0
                    turn_right = 0
                    start_stop = 1
                    short_forward(0.8, fork_position)
                    use_pid_control(fork_position)
                    
            elif(PID_Data[4] == 67):
                stop_truck(fork_position)
                print("Found right turn")
                turn_right += 1

                if (turn_right == station):
                    turn_ninty_degree_right(2, fork_position)
                    turned_into_station = 1
                    print("Found correct turn")
                    short_forward(0.8, fork_position)
                    photo = 1
                    PID_Data = read_bytes(address_sensor, read_bytes_sensor)
                    time.sleep(0.1)
                    use_pid_control(fork_position)

                else:
                    #continue forward to next station
                    short_forward(0.8, fork_position)
                    use_pid_control(fork_position)
            else:
                print(PID_Data[4])
        #----------------------------------------------------------
        #Pallet locations 1, 2 ,3 | 4 lifts location 1, 5 lifts location 2, 6 lifts location 3
        #Pallet cases
        #----------------------------------------------------------
            if(pos == 1 or pos == 2 or pos == 3 or pos == 4):
                if(pos == 1):
                    if(at_station == 1):
                        print("at station with pos 1")
                        #Removing pallet 3
                        fork_position = 3
                        set_fork_state_to(fork_position)
                        short_forward(0.6, fork_position)
                        fork_position = 6
                        set_fork_state_to(fork_position)
                        short_reverse(1.2, fork_position)
                        turn_ninty_degree_right(2, fork_position)
                        short_forward(1.2, fork_position)
                        fork_position = 1
                        set_fork_state_to(fork_position)
                        short_reverse(1.2, fork_position)
                        turn_ninty_degree_left(1.7, fork_position)

                        #Removing pallet 2
                        fork_position = 2
                        set_fork_state_to(fork_position)
                        short_forward(1.2, fork_position)
                        fork_position = 5
                        set_fork_state_to(fork_position)
                        short_reverse(1.2, fork_position)
                        turn_ninty_degree_right(2, fork_position)
                        short_forward(1.2, fork_position)
                        fork_position = 1
                        set_fork_state_to(fork_position)
                        short_reverse(1.2, fork_position)

                        #Taking pallet 1
                        turn_ninty_degree_left(1.7, fork_position)
                        fork_position = 1
                        set_fork_state_to(fork_position)
                        short_forward(1.2, fork_position)
                        fork_position = 4
                        set_fork_state_to(fork_position)
                        short_reverse(1.2, fork_position)
                        fork_position = 5
                        set_fork_state_to(fork_position)
                        turn_one_eighty_right(fork_position)

                        #Start moving back to start
                        short_forward(0.6, fork_position)
                        use_pid_control(fork_position)
                        at_station = 0
                        no_turn = 1
                elif(pos == 2):
                    if(at_station == 1):
                        print("at station with pos 2")
                        #Removing pallet 3
                        #set_fork_state_to(3)
                        fork_position = 3
                        short_forward(0.55, fork_position)
                        fork_position = 6
                        set_fork_state_to(fork_position)
                        short_reverse(2, fork_position)
                        turn_ninty_degree_right(2.3, fork_position)
                        short_forward(1.2, fork_position)
                        fork_position = 1
                        set_fork_state_to(fork_position)
                        short_reverse(1.2, fork_position)

                        #Taking pallet 2
                        turn_ninty_degree_left(1.7, fork_position)
                        fork_position = 2
                        set_fork_state_to(fork_position)
                        short_forward(2, fork_position)
                        fork_position = 5
                        set_fork_state_to(fork_position)
                        short_reverse(2, fork_position)
                        fork_position = 5
                        set_fork_state_to(fork_position)
                        turn_one_eighty_right(1.7, fork_position)

                        #Start moving back to start
                        short_forward(0.8, fork_position)
                        PID_Data = read_bytes(address_sensor, read_bytes_sensor)
                        use_pid_control(fork_position)
                        at_station = 0
                        no_turn = 1
                elif(pos == 3):
                    if(at_station == 1):
                        print("at station with pos 3")
                        #Taking pallet 3
                        fork_position = 3
                        set_fork_state_to(fork_position)
                        short_forward(0.6, fork_position)
                        fork_position = 6
                        set_fork_state_to(fork_position)
                        short_reverse(1.2, fork_position)
                        fork_position = 5
                        set_fork_state_to(fork_position)
                        turn_one_eighty_right(fork_position)

                        #Start moving back to start
                        short_forward(0.8, fork_position)
                        use_pid_control(fork_position)
                        at_station = 0
                        no_turn = 1
                elif(pos == 4):
                    print("at start stop with pos 4")
                    short_forward(1, fork_position)
                    fork_position = 1
                    set_fork_state_to(fork_position)
                    short_reverse(1, fork_position)
                    fork_position = 3
                    set_fork_state_to(fork_position)
                    pos = 0
                    no_turn = 0
                else:
                    print("error in pallet cases")