from smbus2 import SMBus
import socket
import time

#I2C - init
address_sensor = 0x40
register_sensor = 0
address_styr = 0x69
register_styr = 0
read_bytes_sensor = 4
read_bytes_styr = 2
styr = [255, 0]                #[PWM GAFFEL]

#WIFI INIT
socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
HOST = "10.42.0.1"                          #IP för hotspoten som körs via raspberryn
PORT = 9879                                 #bara någon port som råkade gå att användas

def i2c_func(styr):
    #data = bus.read_i2c_block_data(address_sensor, register_sensor, read_bytes_sensor)   # läser read_bytes_sensor antal bytes från address address_sensor
    #print(data)                                #[TEJP WHEELSPEED-R WHEELSPEED-L IR-SENSOR MANUELLT/AUTO]

    time.sleep(0.1)

    data2 = bus.read_i2c_block_data(address_styr, register_styr, read_bytes_styr)
    #data2 = bus.read_byte(address_styr)
    bus.write_i2c_block_data(address_styr, register_styr, styr)
    print(data2)
    #return data
    return

#def wifi_func(data):
#    pallet = conn.recv(1)
#    P = conn.recv(1)
#    D = conn.recv(1)

#    if not pallet or not P or not I or not D:
#        break
#    print(data.decode())            #måste använda decode för att få data på ett vettigt sätt på grund av python 3
#    print(data2.decode())
#    conn.send(bin(data[0])[2:].zfill(8))                 #Tejp
#    conn.send(bin(data[1])[2:].zfill(8))                 #Wheelspeed-r
#    conn.send(bin(data[2])[2:].zfill(8))                 #wheelspeed-l
#    conn.send(bin(data[3])[2:].zfill(8))                 #ir-sensor
#    conn.send(bin(data[4])[2:].zfill(8))                 #manuellt
#    return

# MAIN programmet
if __name__ == "__main__":
    with SMBus(1) as bus:
        
        while True:
            i2c_func(styr)
            #print(styr[0])
            time.sleep(0.1)
    #with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as socket:
    #    socket.bind((HOST, PORT))             #skapar en möjlig connection
    #    socket.listen()                       #lyssnar efter en möjlig connection
    #    conn, addr =  socket.accept()          #accepterar connection
    #    with conn:                              #with används då det är säkrare, den stänger anslutningen när den går ur with
    #        print(f"Connected by {addr}")
    #        with SMBus(1) as bus:
    #            while True:

    #                sensor = i2c_func(styr)
                    #wifi_func(sensor)
    #                time.sleep(0.1)
