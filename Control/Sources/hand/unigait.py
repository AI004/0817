import time
import socket
import threading
import signal
import sys

global port
port = 2562

addr_l = '10.10.10.18'
addr_r = '10.10.10.38'

global cnt
cnt = 0
global sok_l,sok_r

regdict = {
    'ID' : 1000,
    'baudrate' : 1001,
    'clearErr' : 1004,
    'forceClb' : 1009,
    'angleSet' : 1486,
    'forceSet' : 1498,
    'speedSet' : 1522,
    'angleAct' : 1546,
    'forceAct' : 1582,
    'errCode' : 1606,
    'statusCode' : 1612,
    'temp' : 1618,
    'actionSeq' : 2320,
    'actionRun' : 2322
}

def writeRegister(ser, addr, id, add, num, val):
    global port
    sbytes = [0xEB, 0x90]
    sbytes.append(id) # id
    sbytes.append(num + 3) # len
    sbytes.append(0x12) # cmd
    sbytes.append(add & 0xFF)
    sbytes.append((add >> 8) & 0xFF) # add
    for i in range(num):
        sbytes.append(val[i])
    checksum = 0x00
    for i in range(2, len(sbytes)):
        checksum += sbytes[i]
    checksum &= 0xFF
    sbytes.append(checksum)
    ser.sendto(bytes(sbytes),(addr,port))
    time.sleep(0.01)
    recv = ser.recv(1024)
    # print(recv)


def readRegister(ser, addr, id, add, num, mute=False):
    sbytes = [0xEB, 0x90]
    sbytes.append(id) # id
    sbytes.append(0x04) # len
    sbytes.append(0x11) # cmd
    sbytes.append(add & 0xFF)
    sbytes.append((add >> 8) & 0xFF) # add
    sbytes.append(num)
    checksum = 0x00
    for i in range(2, len(sbytes)):
        checksum += sbytes[i]
    checksum &= 0xFF
    sbytes.append(checksum)

    ser.sendto(bytes(sbytes),(addr,port))

    time.sleep(0.01)
    recv = ser.recv(1024)
    if len(recv) == 0:
        return []
    num = (recv[3] & 0xFF) - 3
    val = []
    for i in range(num):
        val.append(recv[7 + i])
    if not mute:
        print('read register：', end='')
        for i in range(num):
            print(val[i], end=' ')
        print()
    return val


def write6(ser, addr, id, str, val):
    if str == 'angleSet' or str == 'forceSet' or str == 'speedSet':
        val_reg = []
        for i in range(6):
            val_reg.append(val[i] & 0xFF)
            val_reg.append((val[i] >> 8) & 0xFF)
        writeRegister(ser, addr, id, regdict[str], 12, val_reg)


def read6(ser, addr, id, str):
    if str == 'angleSet' or str == 'forceSet' or str == 'speedSet' or str == 'angleAct' or str == 'forceAct':
        val = readRegister(ser,addr, id, regdict[str], 12, True)
        if len(val) < 12:
            print('not read data')
            return
        val_act = []
        for i in range(6):
            val_act.append((val[2*i] & 0xFF) + (val[1 + 2*i] << 8))
        print('read value：', end='')
        for i in range(6):
            print(val_act[i], end=' ')
        print()
    elif str == 'errCode' or str == 'statusCode' or str == 'temp':
        val_act = readRegister(ser,addr, id, regdict[str], 6, True)
        if len(val_act) < 6:
            print('not read data')
            return
        print('read value：', end='')
        for i in range(6):
            print(val_act[i], end=' ')
        print()

def timer_thread():
    global cnt

    while True:
        time.sleep(1)
        print(cnt)
        cnt = 0

def hand_ctrl(lr,str,val):
    global sok_l,sok_r
    if lr == 'l':
        write6(sok_l, addr_l, 1, str, val)
    elif lr == 'r': 
        write6(sok_r, addr_r, 1, str, val)

def hand_get_angle(lr):
    global sok_l,sok_r
    if lr == 'l':
        read6(sok_l, addr_l, 1, 'angleAct')
    elif lr == 'r': 
         read6(sok_r, addr_r, 1, 'angleAct')

def hand_l_thread():
    global cnt
    
    hand_ctrl('l','speedSet',[800, 800, 800, 800, 800, 800])
    time.sleep(1)

    hand_ctrl('l','forceSet',[600, 600, 600, 600, 600, 600])
    time.sleep(1)

    hand_ctrl('l','angleSet',[1000, 1000, 1000, 1000, 1000, 1000])
    time.sleep(2)

    loop = True

    hand_ctrl('l','angleSet',[1000, 1000, 1000, 1000, 1000, 1000])
    time.sleep(1)
    hand_ctrl('l','angleSet',[0, 0, 0, 0, 1000, 1000])
    time.sleep(1)
    hand_ctrl('l','angleSet',[1000, 1000, 1000, 1000, 1000, 1000])
    time.sleep(1)
    hand_ctrl('l','angleSet',[1000, 1000, 1000, 1000, 0, 0])
    time.sleep(1)

def hand_r_thread():
    global cnt
    
    hand_ctrl('r','speedSet',[800, 800, 800, 800, 800, 800])
    time.sleep(1)

    hand_ctrl('r','forceSet',[600, 600, 600, 600, 600, 600])
    time.sleep(1)

    hand_ctrl('r','angleSet',[1000, 1000, 1000, 1000, 1000, 1000])
    time.sleep(2)

    loop = True

    hand_ctrl('r','angleSet',[1000, 1000, 1000, 1000, 1000, 1000])
    time.sleep(1)
    hand_ctrl('r','angleSet',[0, 0, 0, 0, 1000, 1000])
    time.sleep(1)
    hand_ctrl('r','angleSet',[1000, 1000, 1000, 1000, 1000, 1000])
    time.sleep(1)
    hand_ctrl('r','angleSet',[1000, 1000, 1000, 1000, 0, 0])
    time.sleep(1)

def sig_handler(signum, frame):
    #is_exit = True
    print('You pressed Ctrl + C!')
    sys.exit(0)



sok_l = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sok_l.settimeout(1)
sok_r = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sok_r.settimeout(1)

hand_ctrl('r','speedSet',[800, 800, 800, 800, 800, 800])
hand_ctrl('r','forceSet',[600, 600, 600, 600, 600, 600])
hand_ctrl('l','speedSet',[800, 800, 800, 800, 800, 800])
hand_ctrl('l','forceSet',[600, 600, 600, 600, 600, 600])

hand_ctrl('r','angleSet',[0, 0, 0, 0, 0, 830])
hand_ctrl('l','angleSet',[0, 0, 0, 0, 0, 860])
time.sleep(0.8)