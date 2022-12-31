from smbus2 import SMBus
import time
import nympy as np
import camera

#I2C - init
global address_sensor = 0x40
global register_sensor = 0
global address_styr = 0x69
global register_styr = 0
global read_bytes_sensor = 9
global read_bytes_styr = 2
global last_error = 0
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
    
    return u_send
def stop_motor():

    return
def turn_ninty_degree_left(left, right): #30 31 har vi mätt upp till bra värden innan
    u_turn_left = [left,right]
    write_bytes(address_styr, u_turn_left)
    sleep(1.55)
    u_turn_left = [0,0]
    write_bytes(address_styr, u_turn_left)
    sleep(1)
    return
def turn_ninty_degree_right(left, right):
    u_turn_left = [left,right]
    write_bytes(address_styr, u_turn_left)
    sleep(1.55)
    u_turn_left = [0,0]
    write_bytes(address_styr, u_turn_left)
    sleep(1)
    return
def stop_truck():
    stop = [0,0]
    write_bytes(address_styr,stop)
    sleep(1)
    return
def short_forward(left,right):
    return
def short_reverse(left,right):
    return
# MAIN programmet
with SMBus(1) as bus:
    
    while True:
        #Should read manuellt/auto variable to see if we should do pid or not
        PID_Data = read_bytes(address_sensor, read_bytes_sensor)
        turn_variable = 0
        pallet = 0 # värde på detta får vi från gränssnittet
        turned_into_station = 0
        station = 0 # Värde på detta får vi från gränssnittet
        at_station = 0
        fork_position = 0
        turn_left = 0
        turn_right = 0
        left_wheel_left_90_turn = 30
        right_wheel_left_90_turn = 31
        left_wheel_right_90_turn = 31
        right_wheel_right_90_turn = 30
        while(PID_Data[6] == 1):
            #PID STUFF
            #----------------------------------------------------------
            PID_Data = read_bytes(address_sensor, read_bytes_sensor)
            u_send = calculatePID(PID_Data)
            write_bytes(address_styr, u_send) # vi ska försöka skicka 1 byte som innehåller info för pwm för båda två hjulen
            #MSB -- första 3 bits är vänster hjul pwm, nästa 3 bits är pwm höger hjul, 2 bitar är direction, först direction vänster sen direction höger -- LSB
            #----------------------------------------------------------
            #Handle turn variable
            #----------------------------------------------------------
            turn_variable = PID_Data[4]  #read variable from sensor module if we are approaching a turn 
            match turn_variable:
                case 130: # means that we are at the last pallet station and turning left into the lane 
                    #turn_left = 1
                    stop_truck()
                    #send data to turn 90 degress
                    turn_left = 1
                    turn_ninty_degree_left(left_wheel_left_90_turn,right_wheel_left_90_turn)
                    turn_right = 0
                    u_send = calculatePID
                    write_bytes(address_styr, u_send)

                case 193: #this means that we are going from a pallet station to the lane, we want to turn left to turn back to the start # this could also mean we are at start
                    #turn_left = 1
                    stop_truck()

                    #time.sleep(0.1)
                    if(turned_into_station == 1): #if this then we have arrived at station and need to take the pallet
                        stop_truck()
                        at_station = 1 #set variable to we know we're at the station and should move forks into correct position depending on variable pos
                        turned_into_station = 0 #reset variable so we enter else when going back to beginning
                    elif(start_stop == 1):
                        stop_truck()
                        pos = 4
                        start_stop = 0
                    else:
                        turn_ninty_degree_left(left_wheel_left_90_turn,right_wheel_left_90_turn)
                        turn_left = 0
                        turn_right = 0
                        start_stop = 1
                        u_send = calculatePID
                        write_bytes(address_styr, u_send) 

                case 67:  #means that we are on the lane and a turn into a pallet station is detected
                    #turn_right = 1 
                    #to turn right, 
                    stop_truck()
                    turn_right += 1
                    time.sleep(0.1)
                    #send data to turn 90 degress
                    if (turn_right == station):
                        turned_into_station = 1
                        turn_ninty_degree_left(left_wheel_left_90_turn,right_wheel_left_90_turn) #90 degree turn
                        stop_truck()
                        pos = find_pallet(pallet)
                        if(pos != 1 or pos != 2 or pos != 3):
                            print(pos + "not found")
                            time.sleep(1)
                            pos = find_pallet(pallet)
                        u_send = calculatePID
                        write_bytes(address_styr, u_send)
                    else:
                        #continue forward to next station
                        u_send = calculatePID
                        write_bytes(address_styr, u_send)

                case _:
                    #turn_left = 0
                    #turn_right = 0
                    print("error in turn variable case")
            
            #----------------------------------------------------------
            # pall location 1, 2 ,3 | 4 lifts location 1, 5 lifts location 2, 6 lifts location 3
            #Pallet cases
            #----------------------------------------------------------
            if(pos == 1 or pos == 2 or pos == 3 or pos == 4):
                match pos:
                    case 1:#if this case we need to move pallet nr 3 and pallet nr 2 to offloading station
                        if(at_station == 1):
                            fork_position = 2 #
                            write_bytes(address_styr, fork_position)
                            #do we need to send stop signal to AX12?
                            u_send = calculatePID
                            write_bytes(address_styr, u_send) #here we must make sure that we go slow and straight, maybe scale u?
                            time.sleep(0.01)

                            #not sure how we can make sure that we are in the correct position, hardcode and hope for the best?
                            stop_truck()
                            fork_position = 5 #not sure of value here, we need to set fork position to lift because we are underneath the 2nd pallet
                            write_bytes(address_styr, fork_position)
                            #do we need to send stop signal to AX12?
                            
                            u_send = calculatePID
                            u_send &= 252 #sets the direction of pwm signal to reverse (we dont affect the 6 highest bits)
                            write_bytes(address_styr, u_send) #maybe scale u? since we want slow reverse
                            
                            stop_truck()

                            #we have reversed, now we should be clear of the station, need to go left or right and drop of pallet
                            write_bytes(address_styr, turn_right) # turn right 
                            stop_truck()

                            #now we need to go forward, drop lower forks and return to station
                            write_bytes(address_styr, u_send_short_forward)
                            stop_truck()

                            #lower forks
                            fork_position = 1 # lower forks
                            write_bytes(address_styr, fork_position)
                            write_bytes(address_styr, u_send_short_reverse)
                            stop_truck()

                            #now we should be back at station but 90 degrees off, turn left and then pickup last pallet
                            turn_ninty_degree_left(30,31)
                            stop_truck()

                            #set fork in correct position for the pallet
                            fork_position = 1
                            write_bytes(address_styr, fork_position)
                            time.sleep(0.1)

                            #short forward
                            u_send = calculatePID
                            write_bytes(address_styr, u_send) #here we must make sure that we go slow and straight, maybe scale u?
                            time.sleep(0.01)

                            #not sure how we can make sure that we are in the correct position, hardcode and hope for the best?
                            stop_truck()

                            #underneath the pallet now we should raise it
                            fork_position = 4
                            write_bytes(address_styr, fork_position)
                            time.sleep(0.1)

                            #now we should have lifted the forks and have the pallet on the forks. Now we need 180 and return to the start
                            write_bytes(address_styr, turn_right)
                            time.sleep(0.1)
                            stop_truck()
                            write_bytes(address_styr, turn_right)
                            time.sleep(0.1)
                            stop_truck()
                            at_station = 0
                            #now we have done 180 turn
                            u_send = calculatePID
                            write_bytes(address_styr, u_send)
                            #return to main loop because we need to watch the case of when we are approaching the left turn into the lane

                    case 2:#if this case we need to move pallet nr 3 to offloading station and grab pallet nr 2 back to start
                        if(at_station == 1):
                            fork_position = 3 #
                            write_bytes(address_styr, fork_position)
                            #do we need to send stop signal to AX12?
                            u_send = calculatePID
                            write_bytes(address_styr, u_send) #here we must make sure that we go slow and straight, maybe scale u?
                            time.sleep(0.01)

                            #not sure how we can make sure that we are in the correct position, hardcode and hope for the best?
                            stop_truck()
                            fork_position = 6 #not sure of value here, we need to set fork position to lift because we are underneath the 3nd pallet
                            write_bytes(address_styr, fork_position)
                            #do we need to send stop signal to AX12?
                            time.sleep(0.1)
                            u_send = calculatePID
                            u_send &= 252 #sets the direction of pwm signal to reverse (we dont affect the 6 highest bits)
                            write_bytes(address_styr, u_send) #maybe scale u? since we want slow reverse
                            stop_truck()
                            #we have reversed, now we should be clear of the station, need to go left or right and drop of pallet
                            write_bytes(address_styr, turn_right) # turn right 
                            stop_truck()

                            #now we need to go forward, drop lower forks and return to station
                            write_bytes(address_styr, u_send_short_forward)
                            stop_truck()

                            #lower forks
                            fork_position = 1 # lower forks
                            write_bytes(address_styr, fork_position)
                            write_bytes(address_styr, u_send_short_reverse)
                            stop_truck()

                            #now we should be back at station but 90 degrees off, turn left and then pickup last pallet
                            turn_ninty_degree_left(30,31)
                            stop_truck()

                            #set fork in correct position for the pallet
                            fork_position = 1
                            write_bytes(address_styr, fork_position)
                            time.sleep(0.1)

                            #short forward
                            u_send = calculatePID
                            write_bytes(address_styr, u_send) #here we must make sure that we go slow and straight, maybe scale u?
                            time.sleep(0.01)

                            #not sure how we can make sure that we are in the correct position, hardcode and hope for the best?
                            stop_truck()

                            #underneath the pallet now we should raise it
                            fork_position = 4
                            write_bytes(address_styr, fork_position)
                            time.sleep(0.1)
                            at_station = 0
                            #now we should have lifted the forks and have the pallet on the forks. Now we need 180 and return to the start
                            write_bytes(address_styr, turn_right)
                            time.sleep(0.1)
                            stop_truck()
                            write_bytes(address_styr, turn_right)
                            stop_truck()
                            
                    case 3:#if this case we just need to grab pallet nr 3 back to start
                        if(at_station == 1):
                            fork_position = 3 #
                            write_bytes(address_styr, fork_position)
                            #do we need to send stop signal to AX12?
                            u_send = calculatePID
                            write_bytes(address_styr, u_send) #here we must make sure that we go slow and straight, maybe scale u?
                            time.sleep(0.01)

                            #not sure how we can make sure that we are in the correct position, hardcode and hope for the best?
                            stop_truck()
                            fork_position = 6 #not sure of value here, we need to set fork position to lift because we are underneath the 3nd pallet
                            write_bytes(address_styr, fork_position)
                            #do we need to send stop signal to AX12?
                            time.sleep(0.1)
                            u_send = calculatePID
                            u_send &= 252 #sets the direction of pwm signal to reverse (we dont affect the 6 highest bits)
                            write_bytes(address_styr, u_send) #maybe scale u? since we want slow reverse
                            stop_truck()
                            
                            #now we should have lifted the forks and have the pallet on the forks. Now we need 180 and return to the start
                            write_bytes(address_styr, turn_right)
                            time.sleep(0.1)
                            stop_truck()
                            write_bytes(address_styr, turn_right)
                            stop_truck()
                            at_station = 0
                            #now we have done 180 turn
                            u_send = calculatePID
                            write_bytes(address_styr, u_send)
                            #return to main loop because we need to watch the case of when we are approaching the left turn into the lane
                    case 4:
                            write_bytes(address_styr, u_send_short_forward)
                            time.sleep(0.1)
                            stop_truck()
                            fork_position = 1
                            write_bytes(address_styr, fork_position)
                            time.sleep(0.1)
                            write_bytes(address_styr, u_send_short_reverse)
                            time.sleep(0.1)
                            stop_truck()
                            pos = 0

                    case _:
                        print("error in pallet cases")
            
            
        
