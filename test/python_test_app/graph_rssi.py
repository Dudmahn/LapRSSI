#
# LapRSSI - RF Based Lap Timing Device
# Copyright (C) 2017 Steve Lilly
#
# This file is part of LapRSSI.
#
# LapRSSI is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# LapRSSI is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with LapRSSI.  If not, see <http://www.gnu.org/licenses/>.
#

import pyqtgraph as pg
import time
import serial
import pyttsx3
import winsound

# Configuration
#comPort = 'COM65'
comPort = 'COM50'
historySeconds = 150.0


# Initialize TTS engine
tts = pyttsx3.init()

# Initialize graphing engine
win = pg.GraphicsWindow(title='LapRSSI Plotter')
win.resize(1000,600)
win.setWindowTitle('LapRSSI Plotter')
pg.setConfigOptions(antialias=True)
p1 = win.addPlot(title='RSSI')
p1.enableAutoRange('x', True)
p1.setYRange(0, 1200, padding=0.0)

legend = p1.addLegend(offset=(1,1))

curTime = 0.000

freq1 = 0
freq2 = 0
freq3 = 0
freq4 = 0
freq5 = 0

rssi1 = []
rssi2 = []
rssi3 = []
rssi4 = []
rssi5 = []

lap1 = []
lap2 = []
lap3 = []
lap4 = []
lap5 = []

threshHi1 = 0
threshHi2 = 0
threshHi3 = 0
threshHi4 = 0
threshHi5 = 0

thresh1 = []
thresh2 = []
thresh3 = []
thresh4 = []
thresh5 = []

def addPoint(a, t, p):
    a.append({'x': t, 'y': p})

def trimOldPoints(a):
    global curTime
    #while((len(a) > 0) and (a[0]['x'] < (curTime - historySeconds))):
    #    a.pop(0)

def processSerialMsg(msg):
    global curTime
    global threshHi1, threshHi2, threshHi3, threshHi4, threshHi5
    global freq1, freq2, freq3, freq4, freq5
    #print(msg)
    msg = msg.rstrip('\r\n')
    fields = msg.split('\t')

    if (fields[0] != '%RSS') and (fields[0] != '%HRT'):
        # Print non-repeating messages to the console
        print(msg)

    if fields[0] == '@FRA':
        if len(fields) == 9:
            freq1 = int(fields[1])
            freq2 = int(fields[2])
            freq3 = int(fields[3])
            freq4 = int(fields[4])
            freq5 = int(fields[5])

    elif fields[0] == '%RSS':
        if len(fields) == 11:
            curTime = float(fields[2]);
            if fields[3]:
                addPoint(rssi1, curTime, int(fields[3]))
                if threshHi1 > 0:
                    addPoint(thresh1, curTime, threshHi1)
            if fields[4]:
                addPoint(rssi2, curTime, int(fields[4]))
                if threshHi2 > 0:
                    addPoint(thresh2, curTime, threshHi2)
            if fields[5]:
                addPoint(rssi3, curTime, int(fields[5]))
                if threshHi3 > 0:
                    addPoint(thresh3, curTime, threshHi3)
            if fields[6]:
                addPoint(rssi4, curTime, int(fields[6]))
                if threshHi4 > 0:
                    addPoint(thresh4, curTime, threshHi4)
            if fields[7]:
                addPoint(rssi5, curTime, int(fields[7]))
                if threshHi5 > 0:
                    addPoint(thresh5, curTime, threshHi5)
            updatePlot()


    elif fields[0] == '%LAP':
        if len(fields) == 9:
            curTime = float(fields[2])
            idx = int(fields[3])
            lapCount = int(fields[4])
            lapTime = float(fields[5])
            peakRssi = int(fields[6])
            threshHi = int(fields[7])
            threshLo = int(fields[8])
            if(idx == 0):
                threshHi1 = threshHi
                addPoint(lap1, curTime, peakRssi)
            if(idx == 1):
                threshHi2 = threshHi
                addPoint(lap2, curTime, peakRssi)
            if(idx == 2):
                threshHi3 = threshHi
                addPoint(lap3, curTime, peakRssi)
            if(idx == 3):
                threshHi4 = threshHi
                addPoint(lap4, curTime, peakRssi)
            if(idx == 4):
                threshHi5 = threshHi
                addPoint(lap5, curTime, peakRssi)
            updatePlot()

            # Play beep
            beepThreadPool.start(beepWorkerThread())

            # Announce lap time
            ####speechThreadPool.start(speechWorkerThread(idx + 1, lapCount, lapTime))


def updatePlot():
    global legend

    trimOldPoints(rssi1)
    trimOldPoints(rssi2)
    trimOldPoints(rssi3)
    trimOldPoints(rssi4)
    trimOldPoints(rssi5)

    trimOldPoints(lap1)
    trimOldPoints(lap2)
    trimOldPoints(lap3)
    trimOldPoints(lap4)
    trimOldPoints(lap5)

    trimOldPoints(thresh1)
    trimOldPoints(thresh2)
    trimOldPoints(thresh3)
    trimOldPoints(thresh4)
    trimOldPoints(thresh5)

    # Clear and re-add the legend each time around
    legend.scene().removeItem(legend)
    legend = p1.addLegend(offset=(1,1))

    p1.plot(rssi1, pen=(255,0,0),   name='Pilot 1: ' + str(freq1), clear=True)
    p1.plot(rssi2, pen=(0,255,0),   name='Pilot 2: ' + str(freq2), clear=False)
    p1.plot(rssi3, pen=(0,0,255),   name='Pilot 3: ' + str(freq3), clear=False)
    p1.plot(rssi4, pen=(180,180,0), name='Pilot 4: ' + str(freq4), clear=False)
    p1.plot(rssi5, pen=(180,0,180), name='Pilot 5: ' + str(freq5), clear=False)

    p1.plot(lap1, pen=None, symbolPen=(255,0,0), symbolBrush=(255,0,0), symbol='x', clear=False)
    p1.plot(lap2, pen=None, symbolPen=(0,255,0), symbolBrush=(0,255,0), symbol='x', clear=False)
    p1.plot(lap3, pen=None, symbolPen=(0,0,255), symbolBrush=(0,0,255), symbol='x', clear=False)
    p1.plot(lap4, pen=None, symbolPen=(180,180,0), symbolBrush=(180,180,0), symbol='x', clear=False)
    p1.plot(lap5, pen=None, symbolPen=(180,0,180), symbolBrush=(180,0,180), symbol='x', clear=False)

    p1.plot(thresh1, pen=(150,0,0), clear=False)
    p1.plot(thresh2, pen=(0,150,0), clear=False)
    p1.plot(thresh3, pen=(0,0,150), clear=False)
    p1.plot(thresh4, pen=(100,100,0), clear=False)
    p1.plot(thresh5, pen=(100,0,100), clear=False)


class serialThread(pg.QtCore.QThread):
    rxSerialMsgSignal = pg.QtCore.Signal(object)
    def run(self):
        try:
            ser = serial.Serial(comPort, 115200, timeout=1)
        except:
            print('Error opening com port ', comPort)

        print('graph_rssi.py serialThread starting up...')

        # Reset the race timer
        ser.write('#RAC\r\n'.encode('utf-8'))
        time.sleep(1.000)

        ser.reset_input_buffer()

        # Set frequencies
        #ser.write('#FRA\t5800\t5800\t5800\t5800\t5800\t5800\t5800\t5800\r\n'.encode('utf-8'))   # All Fatshark 4
        ser.write('#FRA\t5658\t5695\t5760\t5800\t5880\t5917\t5917\t5917\r\n'.encode('utf-8'))   # IMD6C
        #ser.write('#FRA\t5658\t5695\t5732\t5769\t5806\t5843\t5880\t5917\r\n'.encode('utf-8'))   # Raceband 8
        #ser.write('#FRA\t5658\t5658\t5658\t5658\t5658\t5658\t5658\t5658\r\n'.encode('utf-8'))   # All Raceband 1
        time.sleep(0.250)
        
        # Configure the timing device. Parameters are:
        #    rssi_report_interval
        #    cal_offset
        #    cal_thresh
        #    trig_thresh
        ser.write('#CFG\t250\t140\t300\t100\r\n'.encode('utf-8'))
        time.sleep(0.250)
        
        # Query version
        ser.write('?VER\r\n'.encode('utf-8'))
        time.sleep(0.250)

        # Query configuration
        ser.write('?CFG\r\n'.encode('utf-8'))
        time.sleep(0.250)

        # Query frequency assignment
        ser.write('?FRA\r\n'.encode('utf-8'))
        time.sleep(0.250)

        # Query initial RSSIs
        ser.write('?RSS\r\n'.encode('utf-8'))
        time.sleep(0.250)
        
        # Read data from serial port
        while True:
            msg = str(ser.readline(), 'utf-8')
            if msg:
                self.rxSerialMsgSignal.emit(msg)


class beepWorkerThread(pg.QtCore.QRunnable):
    @pg.QtCore.pyqtSlot()
    def run(self):
        winsound.PlaySound('sounds/beep.wav', winsound.SND_FILENAME)


class speechWorkerThread(pg.QtCore.QRunnable):
    def __init__(self, p, l, t):
        super(speechWorkerThread, self).__init__()
        self.pilot = p
        self.lap = l
        self.time = t

    @pg.QtCore.pyqtSlot()
    def run(self):
        msg = 'pilot ' + str(self.pilot)
        if self.lap > 0:
            # Avoid saying lap count and time for hole shot
            msg = msg + ', lap ' + str(self.lap) + ', ' + "{:.3f}".format(self.time)
        tts.say(msg)
        tts.runAndWait()
        

beepThreadPool = pg.QtCore.QThreadPool()
beepThreadPool.setMaxThreadCount( 1 );

speechThreadPool = pg.QtCore.QThreadPool()
speechThreadPool.setMaxThreadCount( 1 );

serialThread = serialThread()
serialThread.rxSerialMsgSignal.connect(processSerialMsg)
serialThread.start()


