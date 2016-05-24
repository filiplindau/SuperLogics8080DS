'''
Created on 30 jan 2015

@author: Filip Lindau
'''

import serial


class Superlogics8080_control:
    def __init__(self, port):
        self.port = port
        try:
            self.close()
        except Exception:
            pass
        self.connected = False
        self.s = None
        self.connect()
        conf = self.getConfiguration()
        if conf is not None:
            print 'Configuration ok'
        else:
            print ''.join(('Configuration problem: ', str(conf)))
        
    def close(self):
        if self.s != None:
            try:
                self.s.close()
            except Exception:
                pass
        self.s = None
        self.connected = False
        
    def connect(self):
        self.close()
        self.s = serial.Serial(self.port, 9600, timeout=0.5, writeTimeout=0.5)
        self.connected = True
            
    def sendReceive(self, cmd):
        c = ''.join((cmd, '\r'))
        retries = 0
        writeReady = False
        while writeReady == False:
            try:
                self.s.write(c)
                writeReady = True
            except Exception:
                self.connect()
                retries += 1
                if retries > 4:
                    raise
        
        readBuf = ''
        readChr = ''
        maxLength = 15
        while readChr != '\r' and readBuf.__len__() < maxLength:
            readChr = self.s.read(1)
            if readChr == '':
                break
            if readChr != '\r':
                readBuf = ''.join((readBuf, readChr))
        if readBuf.__len__() >= maxLength:
            raise(serial.SerialException(''.join(('Received message too long: ', readBuf))))
        if readBuf.__len__() == 0:
            raise(serial.SerialException('Nothing received'))
        if readBuf[0] == '?':
            raise(ValueError('Invalid command'))
        return readBuf
    
    def getConfiguration(self, adr=01):
        """ Get configuration
        output:
            Tuple containing (address, mode, baud rate
        """
        cmd = '$' + str(adr).zfill(2) + '2'
        conf = self.sendReceive(cmd)
        retVal = None
        try:
            if conf[0] == '!':
                adr = conf[1:3]
                mode = conf[3:5]
                mode_dict = {'50': 'counter',
                             '51': 'frequency'}
                baud = conf[5:7].lower()
                baud_dict = {'03': 1200,
                             '04': 2400,
                             '05': 4800,
                             '06': 9600,
                             '07': 19200,
                             '08': 38400,
                             '09': 57600,
                             '0a': 115200}
                retVal = (int(adr), mode_dict[mode], baud_dict[baud])
            else:
                # Invalid command
                retVal = None
        except:
            retVal = None

        return retVal

    def setConfiguration(self, adr=01, mode='frequency', baudrate=9600):
        """
        Set configuration of 8080.

        Input:
            adr: address    (integer)
            mode: 'frequency' or 'counter' (string)
            baudrate: 1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200 (integer)
        """
        mode_dict = {'counter': '50',
                     'frequency': '51'}
        baud_dict = {1200: '03',
                     2400: '04',
                     4800: '05',
                     9600: '06',
                     19200: '07',
                     38400: '08',
                     57600: '09',
                     115200: '0A'}
        TT = mode_dict[mode.lower()]
        CC = baud_dict[baudrate]

        conf = str(adr)+str(adr).zfill(2)+TT+CC+'00'
        self.sendReceive(conf)

    def getInputMode(self):
        cmd = '$01B'
        resp = self.sendReceive(cmd)  
        inMode = int(resp[-1])
        if inMode == 0:
            s = 'ch1: non-isolated, ch2: non-isolated'
        elif inMode == 1:
            s = 'ch1: isolated, ch2: isolated'
        elif inMode == 2:
            s = 'ch1: non-isolated, ch2: isolated'
        elif inMode == 3:
            s = 'ch1: isolated, ch2: non-isolated'
        else:
            s = 'unknown mode'
        return (inMode, s)

    def getFrequency(self, adr=01):
        cmd = '#' + str(adr).zfill(2) + '0'
        resp = self.sendReceive(cmd)  
        freq = int(resp[1:], 16)      
        return freq

    def getStatus(self, adr=01):
        cmd = '~' + str(adr).zfill(2) + '0'
        resp = self.sendReceive(cmd)
        status = int(resp[-2:])
        if status == 0:
            return 'ok'
        elif status == 4:
            return 'host down'
        else:
            return 'unknown status'

    def resetStatus(self, adr=01):
        self.readStatus(adr)
        cmd = '~' + str(adr).zfill(2) + '1'
        resp = self.sendReceive(cmd)

    def getTriggerLevels(self, adr=01):
        cmd = '$' + str(adr).zfill(2) + '1L'
        resp = self.sendReceive(cmd)
        low_level = float(resp[3:])/10

        cmd = '$' + str(adr) + '1H'
        resp = self.sendReceive(cmd)
        high_level = float(resp[3:]) / 10

        return high_level, low_level

    def setTriggerLevels(self, adr=01, low_level=0.5, high_level=2.4):
        low_i = int(low_level*10)
        if low_i < 0:
            low_i = 0
        elif low_i > 50:
            low_i = 50
        cmd = '$' + str(adr).zfill(2) + '1L' + str(low_i).zfill(2)
        self.sendReceive(cmd)

        high_i = int(high_level*10)
        if high_i < 0:
            high_i = 0
        elif high_i > 50:
            high_i = 50
        cmd = '$' + str(adr).zfill(2) + '1H' + str(high_i).zfill(2)
        self.sendReceive(cmd)


if __name__ == '__main__':
    sc = Superlogics8080_control('/dev/ttyUSB1')