from smbus2 import SMBus
import time
import numpy as np
import camera

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
# u = Kp*e + Kd*Td
# Td = ek - ek-1 / ds
# u = kp*(e + (kd*Td / Kp))


def read_bytes(addr, num_bytes):                        # läser read_bytes_sensor antal bytes från address address_sensor
    data = bus.read_i2c_block_data(addr, 0, num_bytes)  #[TEJP WHEELSPEED-R WHEELSPEED-L IR-SENSOR Turn variable MANUELLT/AUTO Pins]
    #data = 1
    #print(data)
    return data

def write_bytes(addr, out_data):
    bus.write_i2c_block_data(addr, 0, out_data)
    return
    
def calculatePID():
    global last_error

    u_send = []
    u_max = 80
    u_min = 5
    kd = 6
    kp = 2
    error = PID_Data[0]
    #error = (error + 5) - 5
    speed = 20
    #distance = PIDData[7] * 5
    #distance = 1
    #error = error / 100
    #error = np.arcsin(error) # reglerfelet i radianer

    #td = (error - last_error) #/ distance # td kommer ha meter som enhet då det är en skillnad i reglerfelet
    u = (int(kp*error + kd*(error-last_error)))/2
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
    #print(error)
    #print(u_send)
    last_error = error
    return u_send

def stop_truck():
    write_bytes(address_styr, [0, 0])
    time.sleep(1)
    return

def turn_ninty_degree_left(left, right): #30 31 har vi mätt upp till bra värden innan
    write_bytes(address_styr, [left, right])
    time.sleep(1.55)
    stop_truck()
    return

def turn_ninty_degree_right(left, right):
    write_bytes(address_styr, [left, right])
    time.sleep(1.55)
    stop_truck()
    return

def turn_one_eighty_left():
    turn_ninty_degree_left(left_wheel_left_90_turn, right_wheel_left_90_turn)
    turn_ninty_degree_left(left_wheel_left_90_turn, right_wheel_left_90_turn)
    return

def turn_one_eighty_right():
    turn_ninty_degree_right(left_wheel_right_90_turn, right_wheel_right_90_turn)
    turn_ninty_degree_right(left_wheel_right_90_turn, right_wheel_right_90_turn)
    return

def short_forward(left, right):
    write_bytes(address_styr, [left, right])
    time.sleep(0.1)
    stop_truck()
    return

def short_reverse(left, right):
    write_bytes(address_styr, [left, right])
    time.sleep(0.1)
    stop_truck()
    return

def set_fork_state_to(level):
    write_bytes(address_styr, level)
    time.sleep(0.1)
    return

def use_pid_control():
    u_send = calculatePID()
    write_bytes(address_styr, u_send)
    time.sleep(0.2)

# MAIN programmet
with SMBus(1) as bus:

    while True:
        #Should read manuellt/auto variable to see if we should do pid or not
        PID_Data = read_bytes(address_sensor, read_bytes_sensor)
        #PID_Data = [1, 1, 1, 1, 1, 1, 1, 1, 1]  #TEMP! Use line above!
        turn_variable = 0
        pallet = 0 # värde på detta får vi från gränssnittet
        turned_into_station = 0
        station = 0 # Värde på detta får vi från gränssnittet
        at_station = 0
        fork_position = 0
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

        while(PID_Data[6] == 1):
            #PID STUFF
            #----------------------------------------------------------
            PID_Data = read_bytes(address_sensor, read_bytes_sensor)                               ### DENNA RADEN MÅSTE ANVÄNDAS!
            # vi ska försöka skicka 1 byte som innehåller info för pwm för båda två hjulen
            use_pid_control()
            #MSB -- första 3 bits är vänster hjul pwm, nästa 3 bits är pwm höger hjul, 2 bitar är direction, först direction vänster sen direction höger -- LSB
            #----------------------------------------------------------

            #Handle turn variable
            #----------------------------------------------------------
            turn_variable = PID_Data[4]  #read variable from sensor module if we are approaching a turn 
            match turn_variable:
                case 130: # means that we are at the last pallet station and turning left into the lane 
                    stop_truck()
                    turn_ninty_degree_left(left_wheel_left_90_turn,right_wheel_left_90_turn)
                    turn_left = 1
                    turn_right = 0
                    use_pid_control()

                case 193: #this means that we are going from a pallet station to the lane, we want to turn left to turn back to the start # this could also mean we are at start
                    
                    stop_truck()

                    if(turned_into_station == 1): #if this then we have arrived at station and need to take the pallet
                        at_station = 1 #set variable to we know we're at the station and should move forks into correct position depending on variable pos
                        turned_into_station = 0 #reset variable so we enter else when going back to beginning

                    elif(start_stop == 1):
                        pos = 4
                        start_stop = 0

                    else:
                        turn_ninty_degree_left(left_wheel_left_90_turn,right_wheel_left_90_turn)
                        turn_left = 0
                        turn_right = 0
                        start_stop = 1
                        use_pid_control()

                case 67:  #means that we are on the lane and a turn into a pallet station is detected
                    stop_truck()
                    turn_right += 1

                    if (turn_right == station):
                        turn_ninty_degree_left(left_wheel_left_90_turn,right_wheel_left_90_turn)
                        turned_into_station = 1
                        pos = 1 #find_pallet(pallet)

                        if(pos != 1 or pos != 2 or pos != 3):
                            print(pos + "not found")
                            time.sleep(1)
                            pos = 1 #find_pallet(pallet)

                        use_pid_control()

                    else:
                        #continue forward to next station
                        use_pid_control()

                case _:
                    print("error in turn variable case")
        
        #----------------------------------------------------------
        #Pallet locations 1, 2 ,3 | 4 lifts location 1, 5 lifts location 2, 6 lifts location 3
        #Pallet cases
        #----------------------------------------------------------
        """if(pos == 1 or pos == 2 or pos == 3 or pos == 4):
            match pos:
                case 1: #if this case we need to move pallet nr 3 and pallet nr 2 to offloading station
                    if(at_station == 1):
                        #Removing pallet 3
                        set_fork_state_to(3)
                        short_forward(short_forward_left, short_forward_right)
                        set_fork_state_to(6)
                        short_reverse(short_reverse_left, short_reverse_right)
                        turn_ninty_degree_right(left_wheel_right_90_turn, right_wheel_right_90_turn)
                        short_forward(short_forward_left, short_forward_right)
                        set_fork_state_to(1)
                        short_reverse(short_reverse_left, short_reverse_right)

                        #Removing pallet 2
                        set_fork_state_to(2)
                        short_forward(short_forward_left, short_forward_right)
                        set_fork_state_to(5)
                        short_reverse(short_reverse_left, short_reverse_right)
                        turn_ninty_degree_right(left_wheel_right_90_turn, right_wheel_right_90_turn)
                        short_forward(short_forward_left, short_forward_right)
                        set_fork_state_to(1)
                        short_reverse(short_reverse_left, short_reverse_right)

                        #Taking pallet 1
                        turn_ninty_degree_left(left_wheel_left_90_turn, right_wheel_left_90_turn)
                        set_fork_state_to(1)
                        short_forward(short_forward_left, short_forward_right)
                        set_fork_state_to(4)
                        short_reverse(short_reverse_left, short_reverse_right)
                        turn_one_eighty_right()

                        #Start moving back to start
                        use_pid_control()
                        at_station = 0

                case 2: #if this case we need to move pallet nr 3 to offloading station and grab pallet nr 2 back to start
                    if(at_station == 1):
                        #Removing pallet 3
                        set_fork_state_to(3)
                        short_forward(short_forward_left, short_forward_right)
                        set_fork_state_to(6)
                        short_reverse(short_reverse_left, short_reverse_right)
                        turn_ninty_degree_right(left_wheel_right_90_turn, right_wheel_right_90_turn)
                        short_forward(short_forward_left, short_forward_right)
                        set_fork_state_to(1)
                        short_reverse(short_reverse_left, short_reverse_right)

                        #Taking pallet 2
                        turn_ninty_degree_left(left_wheel_left_90_turn, right_wheel_left_90_turn)
                        set_fork_state_to(2)
                        short_forward(short_forward_left, short_forward_right)
                        set_fork_state_to(5)
                        short_reverse(short_reverse_left, short_reverse_right)
                        turn_one_eighty_right()

                        #Start moving back to start
                        use_pid_control()
                        at_station = 0

                case 3: #if this case we just need to grab pallet nr 3 back to start
                    if(at_station == 1):
                        #Taking pallet 1
                        set_fork_state_to(3)
                        short_forward(short_forward_left, short_forward_right)
                        set_fork_state_to(6)
                        short_reverse(short_reverse_left, short_reverse_right)
                        turn_one_eighty_right()

                        #Start moving back to start
                        use_pid_control()
                        at_station = 0

                case 4: #Drop of pallet at start/stop location
                    short_forward(short_forward_left, short_forward_right)
                    set_fork_state_to(1)
                    short_reverse(short_reverse_left, short_reverse_right)
                    pos = 0

                case _:
                    print("error in pallet cases")
            
"""