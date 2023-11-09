import time
import serial
import struct
from matplotlib import pyplot as plt

class Comms():

    def __init__(self):
        self.port = serial.Serial('/dev/ttyS0', 9600)
        self.port.flushInput()
        self.port.flushOutput()
        self.stable = True      #will set to false if unreasonable position comms are rx
        self.trigger = None

    def sync(self):
        '''Synchronize serial tx/rx, will wait to recieve a start byte if data is avaialable
            on the port. returns True if synced, False otherwise'''
        if self.port.in_waiting == 0:
            return True
       
        while True:
            startByte = self.port.read(1)
            if startByte != b'T':
                continue
            return True
    
    def verify(self):
        '''ensure that last byte is expected delimiter'''
        if self.port.in_waiting == 0:
            self.stable = False
            self.trigger = 'no data'
            return False
        
        endByte = self.port.read(1)

        if endByte != b't':
            self.stable = False
            self.trigger = 'incorrect end byte'
            return False
        else:
            self.stable = True
            self.trigger = None
            return True

    def rxPos(self, positions=None):
        'rx position data from serial bus'
        self.sync()
        data = self.port.read(16)
        goodPass = self.verify()

        if not goodPass:        #break if end byte incorrect
            self.stable = False
            return positions
            
        P1, P2, P3, P4 = struct.unpack('<iiii', data)
        pos = [P1, P2, P3, P4]

        #verify that we are not getting crazy value jumps between calls
        if positions is not None:
            for i in range(4):
                if abs(pos[i] - positions[i]) > 500:
                    self.stable = False 
                    self.trigger = 'magnitude issue'

        return pos
    
    def txPos(self, desPos):

        data = struct.pack('<iiii', desPos[0], desPos[1], desPos[2], desPos[3])
        self.port.write(b'R')  #send start byte to teensy
        self.port.write(data)        #send position commands
        self.port.write(b'r')

        return








if __name__ == "__main__":
    posHist = []
    bus = Comms()
    print('bus init')
    time.sleep(1)

    pos = bus.rxPos()
    #print('got pass')
        

    count = 0
    loop = 0
    start = time.time()
    
    while True:
        count += 1
        if bus.port.in_waiting >= 18:
            pos = bus.rxPos(pos)
            posHist.append(pos)
            print(f'pos: {pos}, stable: {bus.stable}, trigger: {bus.trigger}')
        
        if time.time() - start > 10:
            bus.port.close()
            time.sleep(1)
            break

    