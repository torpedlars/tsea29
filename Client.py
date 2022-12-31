# echo-client.py
# anslut både Raspberryn och laptop till den Hotspot som heter RPI som körs via raspberryn

import socket
import time

import xlsxwriter  # pip install xlsxwriter

HOST = "10.42.0.1"  # servens IP address
PORT = 9879  # Porten som de ska kommunicera genom, måste vara en ledig port
BUFFER_SIZE = 8 # number of bytes to send and recieve, den skickar typ en sträng så det blir en byte per tecken.
row = 0
column = 0

pallet = 0
pallplats = 0

p = 0
d = 0

excel = xlsxwriter.Workbook('data.xlsx')                            # denna kommer att skapa en ny excel fil och därmed ta bort den gamla med samma namn.
worksheet = excel.add_worksheet('Sensordata')                       # skapar ett nytt worksheet med namnet Sensordata

worksheet.write(row, column, "Tejp-diff")
worksheet.write(row, column+1, "Wheelspeed-R")
worksheet.write(row, column+3, "Wheelspeed-L")
worksheet.write(row, column+2, "IR-Sensor")
row +=1

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as socket:
    socket.connect((HOST, PORT))                                  # accepterar en uppkoppling som servern har öppnat. 
    while True:
        try:
            socket.send(str(pallet).encode())                     #skickar data om det finns
            socket.send(str(pallplats).encode())
            socket.send(str(p).encode())
            socket.send(str(d).encode())

            tejp = socket.recv(BUFFER_SIZE)                       # väntar här till det att den har fått data
            wheelsped_r = socket.recv(BUFFER_SIZE)
            wheelsped_l = socket.recv(BUFFER_SIZE)
            ir_sens = socket.recv(BUFFER_SIZE)
            manual = socket.recv(BUFFER_SIZE)
            tejp = tejp.decode()                                    # python 3 så decode måste användas
            tejp = int(tejp, 2)
            wheelsped_r = wheelsped_r.decode()
            wheelsped_r = int(wheelsped_r, 2)
            wheelsped_l = wheelsped_l.decode()
            wheelsped_l = int(wheelsped_l, 2)
            ir_sens = ir_sens.decode()
            ir_sens = int(ir_sens, 2)
            manual = manual.decode()
            manual = int(manual, 2)
            print(f"Tejp: {tejp}")
            print(f"Wheelspeed_r: {wheelsped_r}")
            print(f"Wheelspeed_l {wheelsped_l}")
            print(f"IR_Sensor: {ir_sens}")
            print(f"Manual mode: {manual}")
            print("")
        except:
            break
        worksheet.write(row, column, tejp)              # spara data i ett excelark, FÅR INTE HA EXCELARKET ÖPPET NÄR DENNA KÖRS
        worksheet.write(row, column+1, wheelsped_r)   
        worksheet.write(row, column+2, wheelsped_l)   
        worksheet.write(row, column+3, ir_sens)           
        row += 1
        time.sleep(0.1)                                 # För att skapa en ungefärlig frekvens

excel.close()                                           # måste stänga excel filen annars sparas inget.
