from smbus2 import SMBus
from time import sleep
#f = open("PIDdata5.txt","w")
#I2C - init
address_sensor = 0x40
register_sensor = 0
address_styr = 0x69
register_styr = 0
read_bytes_sensor = 9
read_bytes_styr = 2
styr = [255, 0]                #[PWM GAFFEL]
#u_send =[0, 0]

last_error = 0
# u = Kp*e + Kd*Td
# Td = ek - ek-1 / ds
# u = kp*(e + (kd*Td / Kp))

def read_bytes(addr, num_bytes):                        # läser read_bytes_sensor antal bytes från address address_sensor
    data = bus.read_i2c_block_data(addr, 0, num_bytes)
    #data = []
    #for i in range(9):
    #    data.append(bus.read_byte(addr))
    #data.append(bus.read_byte(addr))
    #data.append(bus.read_byte(addr))
    #data.append(bus.read_byte(addr))
    #data.append(bus.read_byte(addr))
    #data.append(bus.read_byte(addr))
    #data.append(bus.read_byte(addr))
    #data.append(bus.read_byte(addr))
    #data.append(bus.read_byte(addr))
    #data.append(bus.read_byte(addr))
    if(data[0] >= 100):
        data[0] = data[0] - 256
    if(data[8] >= 100):
        data[8] = data[8] - 256
    #print(data)
    return data

def write_bytes(addr, out_data):
    bus.write_i2c_block_data(addr, 0, out_data)

def calculatePID(PIDdata):
    global last_error

    u_send = []
    u_max = 80
    u_min = 5
    kd = 1
    kp = 2
    error = PIDdata[0]
    #error = (error + 5) - 5
    speed = 20
    #distance = PIDData[7] * 5
    #distance = 1
    #error = error / 100
    #error = np.arcsin(error) # reglerfelet i radianer

    #td = (error - last_error) #/ distance # td kommer ha meter som enhet då det är en skillnad i reglerfelet
    u = int(kp*error + kd*(error-last_error))
    #print(td)
    print(u)
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

# MAIN programmet
with SMBus(1) as bus:
    while True:
        #Should read manuellt/auto variable to see if we should do pid or not
        sleep(1)
        PID_Data = read_bytes(address_sensor, read_bytes_sensor)

        while(PID_Data[6] != 0 or PID_Data[6] != 128):
            #PID STUFF
            #----------------------------------------------------------
            sleep(0.2)
            PID_Data = read_bytes(address_sensor, read_bytes_sensor)
            motor_speed = calculatePID(PID_Data)
            print(motor_speed)
            #sleep(0.1)
            #f.write(" PID: ")
            #f.write(str(PID_Data))
            write_bytes(address_styr, motor_speed)
            
            #f.write("\n")
            #motor_speed = []
            #motor_speed.append(0)
            #motor_speed.append(0)
            #write_bytes(address_styr, motor_speed)
            #testa att göra en 90 graders sväng
            #
            """u_left_turn = 73
            write_bytes(address_styr, u_left_turn)

            u_right_turn = 74"""


