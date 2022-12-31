from smbus2 import SMBus
import socket
import time
import xlsxwriter

row = 0
excel = xlsxwriter.Workbook('data.xlsx')
worksheet = excel.add_worksheet('data')

#I2C - init
address_sensor = 0x40
register_sensor = 0
address_styr = 0x69
register_styr = 0
read_bytes_sensor = 8
read_bytes_styr = 2
styr = [255, 0]                #[PWM GAFFEL]

#WIFI INIT
#socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
HOST = "10.42.0.1"                          #IP för hotspoten som körs via raspberryn
PORT = 52014                                 #bara någon port som råkade gå att användas

#def i2c_func(styr):
    #data = bus.read_i2c_block_data(address_sensor, register_sensor, read_bytes_sensor)   # läser read_bytes_sensor antal bytes från address address_sensor
    #print(data)                                #[TEJP WHEELSPEED-R WHEELSPEED-L IR-SENSOR MANUELLT/AUTO]

    #time.sleep(0.1)

    #data2 = bus.read_i2c_block_data(address_styr, register_styr, read_bytes_styr)
    #data2 = bus.read_byte(address_styr)
    #bus.write_i2c_block_data(address_styr, register_styr, styr)
    #print(data2)
    #return data
    #return

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
    #with SMBus(1) as bus:
        
        #while True:
         #   i2c_func(styr)
            #print(styr[0])
          #  time.sleep(0.1)
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as socket:
        socket.bind((HOST, PORT))             #skapar en möjlig connection
        socket.listen()                       #lyssnar efter en möjlig connection
        conn, addr =  socket.accept()          #accepterar connection
        with conn:                              #with används då det är säkrare, den stänger anslutningen när den går ur with
            print(f"Connected by {addr}")
            #with SMBus(1) as bus:
            movement = [0, 0, 0, 0]
            pid_values = [0, 0, 0]
            fork_movement = [0, 0]
        
            with SMBus(1) as bus:
                while True:
                    try:
                        print("while start")
                            #sensor = i2c_func(styr)
                            #wifi_func(sensor)
                            #conn.recv()
                        #for i in range(3): 
                        movement[0] = conn.recv(1)                                        # väntar här till det att den har fått data
                        movement[0] = movement[0].decode()                                   # python 3 så decode måste användas
                        #movement[0] = int(movement[0], 2)
                        #print("movement 1")
                        #print(ord(movement[0]))                                   # Från basen 2 till 10 (binärt till decimalt)

                        movement[1] = conn.recv(1)                                        # väntar här till det att den har fått data
                        movement[1] = movement[1].decode()                                   # python 3 så decode måste användas
                        #movement[0] = int(movement[0], 2)
                        #print("movement 2")
                        #print(ord(movement[1]))

                        movement[2] = conn.recv(1)                                        # väntar här till det att den har fått data
                        movement[2] = movement[2].decode()                                   # python 3 så decode måste användas
                        #movement[0] = int(movement[0], 2)
                        #print("movement 3")
                        #print(ord(movement[2]))

                        movement[3] = conn.recv(1)                                        # väntar här till det att den har fått data
                        movement[3] = movement[3].decode()                                   # python 3 så decode måste användas
                        #movement[0] = int(movement[0], 2)
                        #print("movement 4")
                        print(ord(movement[0]))
                        print(ord(movement[1]))
                        print(ord(movement[2]))
                        print(ord(movement[3]))
                        
                        #for i in range(4):
                        #    movement[i] = ord(movement[i]) 

                        #i2c_func(styr)

                        if int(ord(movement[0])) == 1:
                            styr = 255
                            print("Fram")

                        elif int(ord(movement[1])) == 1:
                            styr = 252
                            print("Bakåt")

                        elif int(ord(movement[2])) == 1:
                            styr = 29
                            print("Vänster")

                        elif int(ord(movement[3])) == 1:
                            styr = 226
                            print("Höger")
                        
                        else:
                            styr = 0
                        
                        bus.write_byte(address_styr, styr)
                        data = bus.read_i2c_block_data(address_sensor, register_sensor, read_bytes_sensor)
                        worksheet.write(row, 0, data[0])
                        worksheet.write(row, 1, data[1])
                        worksheet.write(row, 2, data[2])
                        worksheet.write(row, 3, data[3])
                        worksheet.write(row, 4, data[4])
                        worksheet.write(row, 5, data[5])
                        worksheet.write(row, 6, data[6])
                        worksheet.write(row, 7, data[7])
                        row += 1
                        print(data)
                    #for i in range(2): 
                    #    pid_values[i] = conn.recv(8)                                        # väntar här till det att den har fått data
                    #    pid_values[i] = pid_values[i].decode()                                   # python 3 så decode måste användas
                    #    pid_values[i] = int(pid_values[i], 2)
                    #    print("pid")

                    #for i in range(1): 
                    #    fork_movement[i] = conn.recv(8)                                        # väntar här till det att den har fått data
                    #    fork_movement[i] = fork_movement[i].decode()                                   # python 3 så decode måste användas
                    #    fork_movement[i] = int(fork_movement[i], 2)
                    #    print("fork movement")

                        #wheelsped_r = socket.recv(BUFFER_SIZE)
                        #wheelsped_l = socket.recv(BUFFER_SIZE)
                        #ir_sens = socket.recv(BUFFER_SIZE)
                        #manual = socket.recv(BUFFER_SIZE)
                        


                        time.sleep(0.1)
                    except:
                        break
        excel.close()