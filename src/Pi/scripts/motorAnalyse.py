import serial
from time import sleep
import threading
from datetime import date

def stopMovement():
    ans=sendSerial("D",True)

def sendSerial(commandString, readResponse=False):
    global send_lock, ser
    response = None
    #send_lock.acquire()
    print("sending serial command: " + str(commandString))
    try:
        ser.write(str(commandString) + "\r")
        if readResponse:
            response = str(ser.readline())
    except:
        print('***Serial write error ***')

    #send_lock.release()

    return response

def parser(string):
    msg=string[5:]
    left,right=msg.split("r")
    
    c,msg=right.split("=")
    right,c=msg.split("\n")

    return left, right

#init
print "initializing teensyCommunicator..."
try:
    ser = serial.Serial("/dev/ttyACM0", 9600, timeout=1)
    print('***Serial for Teensy opened***')
except:
    print('***Serial for Teensy could not be opened***')


# Writing to an excel  
# sheet using Python 
import xlwt 
from xlwt import Workbook 
  
# Workbook is created 
wb = Workbook() 
hFile = open("./motorMapping.h","w")
 
print("***************** Start Forward ******************\n")
# add_sheet is used to create sheet. 
sheet1 = wb.add_sheet('Forward') 

hFile.write("static float motorMapForward[200][3] = {\n")

sheet1.write(0, 0, 'Motor') 
sheet1.write(1, 0, 'Left Odom') 
sheet1.write(2, 0, 'Right Odom') 
 
j=0
for i in range(1, 151, 1):
    #stopMovement()
    j=j+1
    left_odom=right_odom=0.0
    ans=sendSerial("D " + str(i) + " " + str(i) + " 0", True)
    sleep(1)
    for n in range(0,5):
        ans=sendSerial("O", True)
        left,right = parser(ans)
        left_odom+=float(left)
        right_odom+=float(right)
        sleep(0.5)
    left_odom=left_odom/5.0
    right_odom=right_odom/5.0
    print("Right Odom:" + str(right_odom))
    print("Left Odom:" + str(left_odom))
    sheet1.write(0, j, i) 
    sheet1.write(1, j, float(left_odom)) 
    sheet1.write(2, j, float(right_odom))
    hFile.write("{"+str(i)+","+str(float(left_odom))+","+str(float(right_odom))+"},\n")

stopMovement()
hFile.write("};\n\n")

print("***************** Finished Forward******************\n")
print("***************** Start Backward ******************\n")

hFile.write("static float motorMapBackward[200][3] = {\n")

sheet2 = wb.add_sheet('Backwards') 

sheet2.write(0, 0, 'Motor') 
sheet2.write(1, 0, 'Left Odom') 
sheet2.write(2, 0, 'Right Odom') 
 
j=0
for i in range(-1, -151, -1):
    #stopMovement()
    j=j+1
    left_odom=right_odom=0.0
    ans=sendSerial("D " + str(i) + " " + str(i) + " 0", True)
    sleep(1)
    for n in range(0,5):
        ans=sendSerial("O", True)
        left,right = parser(ans)
        left_odom+=float(left)
        right_odom+=float(right)
        sleep(0.5)
    left_odom=left_odom/5.0
    right_odom=right_odom/5.0
    print("Right Odom:" + str(right_odom))
    print("Left Odom:" + str(left_odom))
    sheet2.write(0, j, i) 
    sheet2.write(1, j, float(left_odom)) 
    sheet2.write(2, j, float(right_odom))
    hFile.write("{"+str(i)+","+str(float(left_odom))+","+str(float(right_odom))+"},\n")

stopMovement()
hFile.write("};\n\n")


wb.save('Odom'+str(date.today())+'.xls') 
print("***************** Finished ******************")

