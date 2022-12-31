from smbus2 import SMBus
import time
import numpy as np
from camera import *

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

def read_bytes(addr, num_bytes):                        # läser read_bytes_sensor antal bytes från address address_sensor
    data = bus.read_i2c_block_data(addr, 0, num_bytes)  #[TEJP WHEELSPEED-R WHEELSPEED-L IR-SENSOR Turn variable MANUELLT/AUTO Pins]
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
    speed = 20

    u = int((kp*error + kd*(error-last_error))/2)

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
        write_bytes(address_styr, [0,0,3]) #Set forks in correct position
        no_turn = 0
        photo = 0
        collision_on = 0
        turn_into_unloading = 0
        unload_complete = 0
        back_to_lane = 0
        transfer_complete = 0
        while(PID_Data[6] == 1): #add an and with start/stop?
            #if(transfer_complete == 1):
            #    while(pallet != 0):
            #        stop_truck(fork_position)## stå och vänta tills vi får information från gränssnittet att vi ska köra igen, reseta variabler?
            #PID STUFF
            #----------------------------------------------------------
            PID_Data = read_bytes(address_sensor, read_bytes_sensor)                               ### DENNA RADEN MÅSTE ANVÄNDAS!
            while(PID_Data[3] == 1 and collision_on == 1):
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
            
            if(PID_Data[4] == 130): #Left turn
                stop_truck(fork_position)
                print("Found Left turn")

                if(no_turn == 1):
                    fork_position = 3
                    print("on our way back to start stop")
                    short_forward(0.8,fork_position)
                    PID_Data = read_bytes(address_sensor, read_bytes_sensor)
                    use_pid_control(fork_position)

                elif(turn_into_unloading == 1):
                    turned_into_unloading = 1
                    turn_into_unloading = 0
                    print("turned into unloading")
                    fork_position = 3
                    turn_ninty_degree_left(1.7, fork_position)
                    short_forward(0.8,fork_position)
                    PID_Data = read_bytes(address_sensor, read_bytes_sensor)
                    use_pid_control(fork_position)
                elif(back_to_lane == 1):
                    no_turn = 1
                    start_stop = 1
                    turn_ninty_degree_left(1.7, fork_position)  
                    print("turning left")
                    short_forward(0.8, fork_position)
                    PID_Data = read_bytes(address_sensor, read_bytes_sensor)
                    use_pid_control(fork_position)

            elif(PID_Data[4] == 193): #Three way junction
                stop_truck(fork_position)
                print("Found 3 way junction")

                if(turned_into_station == 1): #if this then we have arrived at station and need to take the pallet
                    at_station = 1 #set variable to we know we're at the station and should move forks into correct position depending on variable pos
                    turned_into_station = 0 #reset variable so we enter else when going back to beginning
                    print("At station")
                    time.sleep(1)

                elif(turned_into_unloading == 1):
                    print("at unloading station")
                    turned_into_unloading = 0
                    fork_position = 1
                    set_fork_state_to(fork_position)
                    short_reverse(1.8, fork_position)
                    turn_one_eighty_right(1.7, fork_position)
                    short_forward(0.5, fork_position)
                    if(pos == 1):
                        fork_position = 1
                        print("fork pos changed to",fork_position)
                    elif(pos == 2):
                        fork_position = 2
                        print("fork pos changed to",fork_position)
                    unload_complete = 1
                    PID_Data = read_bytes_sensor(address_sensor, read_bytes_sensor)
                    use_pid_control(fork_position)
                    
                elif(start_stop == 1):
                    print("found start and stop")
                    pos = 4
                    start_stop = 0
                    
                elif(back_to_lane == 1):
                    no_turn = 1
                    start_stop = 1
                    turn_ninty_degree_left(1.7, fork_position)  
                    print("turning left")
                    short_forward(0.8, fork_position)
                    PID_Data = read_bytes(address_sensor, read_bytes_sensor)
                    use_pid_control(fork_position)   
                    
            elif(PID_Data[4] == 67): #Right turn
                stop_truck(fork_position)
                print("Found right turn")
                turn_right += 1 # borde lägga denna i elsen och initisera den till 1

                if (turn_right == station): #Turn right on the correct right turn
                    turn_ninty_degree_right(2, fork_position)
                    turned_into_station = 1
                    print("Fturn right == station")
                    short_forward(0.8, fork_position)
                    photo = 1
                    PID_Data = read_bytes(address_sensor, read_bytes_sensor)
                    time.sleep(0.1)
                    use_pid_control(fork_position)

                elif(photo == 1): #stop at right turn to take picture of pallet station
                    pos = find_pallet(pallet)
                    collision_on = 0
                    if(pos == 2 or pos == 1):
                        fork_position = 2
                        print("fork pos after photo", fork_position)
                    elif(pos == 3):
                        fork_position = 3
                        print("fork pos after photo", fork_position)
                    short_forward(0.8, fork_position)
                    if(pos != 1 and pos != 2 and pos != 3):
                        print(f"{pos} not found")
                        time.sleep(1)
                        pos = find_pallet(pallet)
                    short_forward(0.8, fork_position)
                    PID_Data = read_bytes(address_sensor, read_bytes_sensor)
                    time.sleep(0.1)
                    photo = 0
                    use_pid_control(fork_position)

                elif(unload_complete == 1): # reset for this variable is in pos cases
                    fork_position = 3
                    turn_ninty_degree_right(2, fork_position)
                    turned_into_station = 1
                    print("Turned right from unloading towards station")
                    short_forward(0.8, fork_position)
                    PID_Data = read_bytes(address_sensor, read_bytes_sensor)
                    use_pid_control(fork_position)

                else:#continue forward to next station
                    short_forward(0.8, fork_position)
                    print("continuing to on the lane")
                    use_pid_control(fork_position)
            else:
                print(PID_Data[4])

        #----------------------------------------------------------
        #Pallet locations 1, 2 ,3 | 4 lifts location 1, 5 lifts location 2, 6 lifts location 3
        #Pallet cases
        #----------------------------------------------------------
            if(pos == 1 or pos == 2 or pos == 3 or pos == 4):
                if(pos == 1): #pallet is at lowest position
                    if(at_station == 1):
                        print("at station with pos 1")
                        if(unload_complete == 1): #have unloaded both 2nd and 3rd pallet and just need to grab 1st
                            fork_position = 4
                            set_fork_state_to(fork_position)
                            print("unloaded pallet 2 and 3, grabing pallet 1, fork pos =", fork_position)
                            short_reverse(2,fork_position)
                            turn_one_eighty_right(1.7,fork_position)
                            at_station = 0 
                            fork_position = 3
                            unload_complete = 0
                            back_to_lane = 1
                            short_forward(0.8, fork_position)
                            PID_Data = read_bytes(address_sensor, read_bytes_sensor)
                            use_pid_control(fork_position)
                        else: #Need to grab 2nd and 3rd pallet and unload them
                            fork_position = 5
                            print("Need to remove pallet 2 and 3, fork pos =", fork_position)
                            set_fork_state_to(fork_position)
                            time.sleep(0.1)
                            short_reverse(2, fork_position)
                            turn_one_eighty_right(1.7, fork_position)
                            at_station = 0
                            turn_into_unloading = 1
                            short_forward(0.8, fork_position)
                            PID_Data = read_bytes(address_sensor, read_bytes_sensor)
                            use_pid_control(fork_position)
                        
                elif(pos == 2): #pallet is in the middle
                    if(at_station == 1):
                        print("at station with pos 2")
                        if(unload_complete == 1): #Have already unloaded pallet 3, need to grab second and then return to start 
                            fork_position = 5
                            set_fork_state_to(fork_position)
                            print("unloaded pallet 3, grabing pallet 2, fork pos =", fork_position)
                            short_reverse(2,fork_position)
                            turn_one_eighty_right(fork_position)
                            at_station = 0 
                            fork_position = 3
                            unload_complete = 0
                            back_to_lane = 1
                            short_forward(0.8, fork_position)
                            PID_Data = read_bytes(address_sensor, read_bytes_sensor)
                            use_pid_control(fork_position)

                        else: #need to grab 3rd pallet and unload it
                            fork_position = 6
                            set_fork_state_to(fork_position)
                            print("Need to remove pallet 3, fork pos =", fork_position)
                            short_reverse(2, fork_position)
                            turn_one_eighty_right(1.7,fork_position)
                            at_station = 0
                            turn_into_unloading = 1
                            short_forward(0.8, fork_position)
                            PID_Data = read_bytes(address_sensor, read_bytes_sensor)
                            use_pid_control(fork_position)

                elif(pos == 3): #pallet is at the top
                    if(at_station == 1):
                        print("at station with pos 3")#need to grab 3rd pallet 
                        fork_position = 6
                        set_fork_state_to(fork_position)
                        print("Need to grab pallet 3, fork pos =", fork_position)
                        short_reverse(2, fork_position)
                        turn_one_eighty_right(1.7,fork_position)
                        at_station = 0
                        turn_into_unloading = 0
                        unload_complete = 0
                        back_to_lane = 1
                        short_forward(0.8, fork_position)
                        PID_Data = read_bytes(address_sensor, read_bytes_sensor)
                        use_pid_control(fork_position)

                elif(pos == 4):
                    print("at start stop with pos 4")
                    fork_position = 1
                    set_fork_state_to(fork_position)
                    print("lower forks and drop off pallet, fork pos ==", fork_position)
                    short_reverse(2, fork_position)
                    pos = 0
                    no_turn = 0
                    turn_one_eighty_right(1.7,fork_position)
                    short_forward(0.8, fork_position)
                    PID_Data = read_bytes(address_sensor, read_bytes_sensor)
                    use_pid_control(fork_position)
                    fork_position = 3
                    print("done with delivery, set fork pos to =", fork_position)
                    set_fork_state_to(fork_position)
                    stop_truck(fork_position)
                    transfer_complete = 1
                    time.sleep(20)

                else:
                    print("error in pallet cases")