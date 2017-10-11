//
// LapRSSI - RF Based Lap Timing Device
// Copyright (C) 2017 Steve Lilly
//
// This file is part of LapRSSI.
//
// LapRSSI is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// LapRSSI is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with LapRSSI.  If not, see <http://www.gnu.org/licenses/>.
//

#include <errno.h>
#include <EEPROM.h>
#include <limits.h>
#include <ADC.h>
#include <RingBufCPP.h>
#include <SPI.h>  // include the new SPI library

////////////////////////////////////////////////////////////////////////////////
// Defines / Macros
////////////////////////////////////////////////////////////////////////////////
#define FIRMWARE_VERSION                  "1.5"
#define PROTOCOL_VERSION                  "1.4"

#define EEPROM_MAGIC_NUMBER               0xEE
#define EEPROM_VERSION                    1

#define MAX_RX_NODES                      8
#define ADC_RESOLUTION                    10
#define ADC_MAX                           ((1 << ADC_RESOLUTION) - 1)
#define ADC_DETECT_THRESH                 200           // RSSI value must be above this level to detect an RX5808 module present
#define ADC_OFFSET                        520
#define ADC_MULTIPLIER                    9
#define ADC_DIVIDER                       4
#define ADC_FILTER_BITS                   5     // 2^5 = 32 samples

#define SERIAL_BAUD_RATE                  19200
#define SERIAL_TX_MSG_MAX_LEN             64            // Max size of a single serial message
#define SERIAL_TX_MSG_QUEUE_LEN           10            // Queue up to 10 serial messages before dropping them

#define HEARTBEAT_REPORT_INTERVAL         1000
#define RSSI_REPORT_INTERVAL_DEFAULT      1000
#define RSSI_REPORT_INTERVAL_MIN          250

// RTC6715 Registers
#define RTC6715_SYNTH_REG_A               0x00
#define RTC6715_SYNTH_REG_B               0x01
#define RTC6715_SYNTH_REG_C               0x02
#define RTC6715_SYNTH_REG_D               0x03
#define RTC6715_VCO_SWITCH_CAP_CTRL_REG   0x04
#define RTC6715_DFC_CTRL_REG              0x05
#define RTC6715_6M_AUDIO_DEMOD_CTRL_REG   0x06
#define RTC6715_65M_AUDIO_DEMOD_CTRL_REG  0x07
#define RTC6715_RECEIVER_CTRL_REG_1       0x08
#define RTC6715_RECEIVER_CTRL_REG_2       0x09
#define RTC6715_POWER_DOWN_CTRL_REG       0x0A
#define RTC6715_STATE_REG                 0x0F

// Definitions for restart CPU macro
#define CPU_RESTART_ADDR (uint32_t *)0xE000ED0C
#define CPU_RESTART_VAL 0x5FA0004
#define CPU_REBOOT() (*CPU_RESTART_ADDR = CPU_RESTART_VAL);


////////////////////////////////////////////////////////////////////////////////
// Type Definitions
////////////////////////////////////////////////////////////////////////////////

// Global configuration parameters
typedef struct {
  int rssiReportIntervalMs;
  int calOffset;
  int calThresh;
  int trigThresh;
  int rssiFilterConstant;
} configParams_t;

// Per receiver node state
typedef struct {
  int enabled;
  int freq;

  bool autoCalibrationMode;
  bool crossing;
  int rssiTrigger;

  int rssiRaw;          // it's actually filtered, but we call it raw
  int rssiPeakRaw;
  uint32_t rssiPeakRawTimestamp;

  float rssiSmoothed;   // longer term "smoothed" RSSI
  int rssiPeakSmoothed;
  
  int rssiPeakSmoothedForRSS; // peak smoothed RSSI value since last RSS message

  // Last lap state
  int lapCount;
  int lapTimestamp;
  int lapTime;
  int lapPeakRssiRaw;
  int lapPeakRssiSmoothed;
} rxNodeState_t;

// Structure used to queue up serial messages for transmission
typedef struct {
  int8_t len;
  char buf[SERIAL_TX_MSG_MAX_LEN];
} serialTxMsg_t;

////////////////////////////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////////////////////////////

// Serial debug stream
#define DebugSerial   Serial

// GPIO related variables
const int ledPin = 13;

// SPI related variables
const int spiSCKPin = 14;         // SCK on alternate pin
const int spiMOSIPin = 11;
const int spiMISOPin = 12;
const int spiSSPins[MAX_RX_NODES] = { 2, 3, 4, 5, 6, 7, 8, 23 };
const SPISettings spiSettings(4000000, LSBFIRST, SPI_MODE0);

// ADC related variables
ADC *adc;
const int rssiPins[MAX_RX_NODES] = { A1, A2, A3, A4, A5, A6, A7, A8 };

// UART related variables
RingBufCPP<serialTxMsg_t, SERIAL_TX_MSG_QUEUE_LEN> serialTxMsgQueue;

// Global state parameters
int raceNumber = 0;
int heartbeatCounter = 1;
int measurementCount = 0;
elapsedMillis heartbeatReportTimer;
elapsedMillis rssiReportTimer;
elapsedMillis raceTimer;
rxNodeState_t rxNodes[MAX_RX_NODES];
configParams_t cfg = {
  .rssiReportIntervalMs = RSSI_REPORT_INTERVAL_DEFAULT,
  .calOffset = 160,
  .calThresh = 300,
  .trigThresh = 100,
  .rssiFilterConstant = 25    // in thousanths
};

// Define vtx frequencies in mhz and their hex code for setting the rx5808 module
#define FREQ_MIN          5645
#define FREQ_MAX          5945

const int vtxFreqTable[] = {
  5865, 5845, 5825, 5805, 5785, 5765, 5745, 5725, // Band A
  5733, 5752, 5771, 5790, 5809, 5828, 5847, 5866, // Band B
  5705, 5685, 5665, 5645, 5885, 5905, 5925, 5945, // Band E
  5740, 5760, 5780, 5800, 5820, 5840, 5860, 5880, // Band F
  5658, 5695, 5732, 5769, 5806, 5843, 5880, 5917  // Band C / Raceband
};
const uint16_t vtxHexTable[] = {
  0x2A05, 0x299B, 0x2991, 0x2987, 0x291D, 0x2913, 0x2909, 0x289F, // Band A
  0x2903, 0x290C, 0x2916, 0x291F, 0x2989, 0x2992, 0x299C, 0x2A05, // Band B
  0x2895, 0x288B, 0x2881, 0x2817, 0x2A0F, 0x2A19, 0x2A83, 0x2A8D, // Band E
  0x2906, 0x2910, 0x291A, 0x2984, 0x298E, 0x2998, 0x2A02, 0x2A0C, // Band F
  0x281D, 0x288F, 0x2902, 0x2914, 0x2987, 0x2999, 0x2A0C, 0x2A1E  // Band C / Raceband
};
#define NUM_VTX_TABLE_ENTRIES   (sizeof(vtxFreqTable) / sizeof(int))

int installedNodes[] = { 0,    0,    0,    0,    0,    0,    0,    0 };
const int defaultFrequencies[] =  { 5658, 5695, 5732, 5769, 5806, 5843, 5880, 5917 };    // Raceband 8

// This array contains the moving average (sum) for each receiver node. It is accessed
// by the adc0 interrupt, and also by the backgroup loop. In order to prevent reading
// corrupted data, the backgroup loop must disable interrupts when accessing it.
volatile int rssiValMovingAvgSums[MAX_RX_NODES];

////////////////////////////////////////////////////////////////////////////////
// Functions
////////////////////////////////////////////////////////////////////////////////

//
// Setup routine
//
void setup() {
  int i;

  // Debug UART (USB Serial) setup
  DebugSerial.begin(115200);
  delay(1000);
  DebugSerial.println("LapRSSI starting up...");
  
  // Hardware UART setup
  Serial1.begin(SERIAL_BAUD_RATE);
  while (!Serial1) ; // wait until serial port is running
  
  // GPIO pin setup
  pinMode(ledPin, OUTPUT);
  for (i = 0; i < MAX_RX_NODES; i++) {
    pinMode(rssiPins[i], INPUT_PULLDOWN); // setup rssi pin initially with pulldown to detect if modules are installed
  }

  // SPI pins
  for (i = 0; i < MAX_RX_NODES; i++) {
    pinMode(spiSSPins[i], OUTPUT);
    digitalWrite (spiSSPins[i], HIGH);  // set SPI SS pin to idle (high)
  }

  // SPI setup
  SPI.setSCK(spiSCKPin);
  SPI.setMOSI(spiMOSIPin);
  SPI.setMISO(spiMISOPin);
  SPI.begin();

  // ADC Setup
  adc = new ADC();
  adc->setResolution(ADC_RESOLUTION, ADC_0);
  adc->setReference(ADC_REFERENCE::REF_1V2, ADC_0);                 // Use internal 1.2V reference with a 100K pull-down on the RSSI pin
  adc->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED, ADC_0);
  adc->setSamplingSpeed(ADC_SAMPLING_SPEED::HIGH_SPEED, ADC_0);
  adc->setAveraging(1, ADC_0);                                      // Disable averaging, because it causes additional delay

  // Delay to let pulldown settle before attempting to detect presence of modules
  delay(10);

  for (i = 0; i < MAX_RX_NODES; i++) {
    // Initialize default frequency
    rxNodes[i].freq = defaultFrequencies[i];
    
    // Determine if this RX5808 module is installed
    if (adc->analogRead(rssiPins[i], ADC_0) >= ADC_DETECT_THRESH) {
      installedNodes[i] = 1;
      rxNodes[i].enabled = 1; // may be overridden below when restoring from EEPROM
    }
    pinMode(rssiPins[i], INPUT);  // remove pulldown
  }

  // Restore configuration, enabled/disable status, and frequencies from EEPROM
  restoreFromEEPROM();

  // Initialize receiver nodes
  for (i = 0; i < MAX_RX_NODES; i++) {
    if (installedNodes[i]) {
      initRxModuleRegisters(i);
      setRxModuleFreq(i, rxNodes[i].freq);
    }
  }
 
  // Start ADC read on the first channel. The ISR routine adc0_isr() will be invoked when the
  // conversion is complete. The ISR will then read the result and trigger a conversion on the next channel.
  adc->enableInterrupts(ADC_0);
  adc->startSingleRead(rssiPins[0], ADC_0);

  // Schedule heartbeat timer to offset with RSSI reports
  heartbeatReportTimer = HEARTBEAT_REPORT_INTERVAL;

  // Reset all receiver nodes
  resetAllRxNodes();

  // Reset race timer
  raceTimer = 0;
}

//
// Main loop
//
void loop() {
  int i;
  int sums[MAX_RX_NODES];
  static uint32_t raceTimerLast = 0;

  //
  // RSSI peak detection algorithm, performed once at the top of every millisecond
  //
  if (raceTimerLast != raceTimer) {
    raceTimerLast = raceTimer;

    //digitalWrite(ledPin, HIGH);

    // Copy RSSI sums to a local array so that we can work with the data without the ADC ISR overwriting it
    cli();    // disable interrupts
    for (i = 0; i < MAX_RX_NODES; i++) {
      sums[i] = rssiValMovingAvgSums[i];
    }
    sei();    // enable interrupts

    for (i = 0; i < MAX_RX_NODES; i++) {
      // Divide the moving average RSSI sum (with rounding) to be the moving average
      rxNodes[i].rssiRaw = (sums[i] + (sums[i] >> (ADC_FILTER_BITS - 1))) >> ADC_FILTER_BITS;
      rxNodes[i].rssiSmoothed = (rxNodes[i].rssiRaw * (cfg.rssiFilterConstant / 1000.0)) + (rxNodes[i].rssiSmoothed * (1.0 - (cfg.rssiFilterConstant / 1000.0)));

      // Update peak smoothed RSSI value for the RSS message
      rxNodes[i].rssiPeakSmoothedForRSS = max(rxNodes[i].rssiPeakSmoothedForRSS, rxNodes[i].rssiSmoothed);

      if (rxNodes[i].enabled) {
        if (rxNodes[i].rssiTrigger > 0) {
          if ((! rxNodes[i].crossing) && (rxNodes[i].rssiSmoothed > rxNodes[i].rssiTrigger)) {
            rxNodes[i].crossing = true;
            //Serial1.print("%DBG\trxNode ");
            //Serial1.print(i);
            //Serial1.print(" crossing ----------------");
            //Serial1.println();
          }

          // Find the peak rssi and the time it occured during a crossing event
          if (rxNodes[i].rssiRaw > rxNodes[i].rssiPeakRaw) {
            rxNodes[i].rssiPeakRaw = rxNodes[i].rssiRaw;
            rxNodes[i].rssiPeakRawTimestamp = raceTimer;
          }

          if (rxNodes[i].crossing) {
            int trigThresh = cfg.trigThresh;

            if (rxNodes[i].autoCalibrationMode) {
              rxNodes[i].rssiTrigger = max(rxNodes[i].rssiTrigger, rxNodes[i].rssiSmoothed - cfg.calOffset);
              // when calibrating, use a larger threshold
              trigThresh = cfg.calThresh;
            }

            rxNodes[i].rssiPeakSmoothed = max(rxNodes[i].rssiPeakSmoothed, rxNodes[i].rssiSmoothed);

            // Make sure the threshold does not put the trigger below 0 RSSI
            // See if we have left the gate
            if ((rxNodes[i].rssiTrigger > trigThresh) &&
              (rxNodes[i].rssiSmoothed < (rxNodes[i].rssiTrigger - trigThresh))) {

              // Crossing complete
              rxNodes[i].lapPeakRssiSmoothed = rxNodes[i].rssiPeakSmoothed;
              rxNodes[i].lapPeakRssiRaw = rxNodes[i].rssiPeakRaw;
              rxNodes[i].lapTime = rxNodes[i].rssiPeakRawTimestamp - rxNodes[i].lapTimestamp;
              rxNodes[i].lapTimestamp = rxNodes[i].rssiPeakRawTimestamp;

              rxNodes[i].crossing = false;
              rxNodes[i].autoCalibrationMode = false;
              rxNodes[i].rssiPeakRaw = 0;
              rxNodes[i].rssiPeakSmoothed = 0;

              // Send LAP message
              sendMsgLAP(i);

              // Update lap count (do this after the LAP report, so the hole shot is reported at lap 0)
              rxNodes[i].lapCount++;
            }
          }
        }
      }
    }

    //digitalWrite(ledPin, LOW);
  }

  // Process incoming comms messages
  rxMessageStateMachine();

  // Periodic Heartbeat report
  if (heartbeatReportTimer >= HEARTBEAT_REPORT_INTERVAL) {
    heartbeatReportTimer = 0;
    sendMsgHRT();
    heartbeatCounter++;
    measurementCount = 0;
  }

  // Periodic RSSI report
  if ((cfg.rssiReportIntervalMs) && (rssiReportTimer >= (uint32_t) cfg.rssiReportIntervalMs)) {
    rssiReportTimer = 0;
    sendMsgRSS(true);
  }

  // Send a queued serial message, if space is available
  handleQueuedSerialTxMsg();
}

// Send a single queued serial message, but only if space is available in the transmit buffer
// This avoids blocking, to prevent delays in the peak detection algorithm.
// Returns true if there are messages still waiting to be send, false if the tx message queue is empty.
bool handleQueuedSerialTxMsg() {
  if (! serialTxMsgQueue.isEmpty()) {
    serialTxMsg_t *pMsg = serialTxMsgQueue.peek(0);
    if (Serial1.availableForWrite() >= pMsg->len) {
      serialTxMsg_t msg;
      serialTxMsgQueue.pull(&msg);
      Serial1.print(msg.buf);
    }
  }

  return ! serialTxMsgQueue.isEmpty();
}

// Restore all settings from EEPROM, including config settings, rx node enabled state, and rx node frequencies
// If the EEPROM contents is determined to be invalid via a simple magic number and version check, then the
// EEPROM is intitialized with default values.
void restoreFromEEPROM() {
  int i, j;
  int idx;
  byte val;
  int freq;
  bool writeDefaults = false;
  
  DebugSerial.println("Restoring settings from EEPROM");

  // Checkl EEPROM magic number
  val = EEPROM.read(0);
  if (val != EEPROM_MAGIC_NUMBER) {
    DebugSerial.println("EEPROM magic number mismatch");
    writeDefaults = true;
  }
  else {
    // Check EEPROM version
    val = EEPROM.read(1);
    if (val != EEPROM_VERSION) {
      DebugSerial.println("EEPROM version mismatch");
      writeDefaults = true;
    }
  }
  
  // Clear the EEPROM settings area, if needed
  if (writeDefaults) {
    DebugSerial.println("Writing defaults to EEPROM");
    
    EEPROM.write(0, EEPROM_MAGIC_NUMBER);
    EEPROM.write(1, EEPROM_VERSION);
    
    saveConfigToEEPROM();
    saveRxEnableToEEPROM();
    saveRxFrequenciesToEEPROM();
  }

  // Restore config settings
  idx = 2;
  for (i = 0; i < sizeof(cfg); i++) {
    val = EEPROM.read(idx++);
    ((byte *)(&cfg))[i] = val;
  }

  // Restore receiver enable/disable
  for (i = 0; i < MAX_RX_NODES; i++) {
    val = EEPROM.read(idx++);
    if ((val != 0) && installedNodes[i]) {
      rxNodes[i].enabled = 1;
    }
    else {
      rxNodes[i].enabled = 0;
    }
  }
  
  // Restore frequencies
  for (i = 0; i < MAX_RX_NODES; i++) {
    freq = EEPROM.read(idx++);
    freq |= EEPROM.read(idx++) << 8;
    for (j = 0; j < NUM_VTX_TABLE_ENTRIES; j++) {
      if (freq == vtxFreqTable[j]) {
        rxNodes[i].freq = freq;
        break;
      }
    }
  }

  DebugSerial.print("rssiReportIntervalMs: ");
  DebugSerial.println(cfg.rssiReportIntervalMs);
  DebugSerial.print("calOffset: ");
  DebugSerial.println(cfg.calOffset);
  DebugSerial.print("calThresh: ");
  DebugSerial.println(cfg.calThresh);
  DebugSerial.print("trigThresh: ");
  DebugSerial.println(cfg.trigThresh);
  DebugSerial.print("rssiFilterConstant: ");
  DebugSerial.println(cfg.rssiFilterConstant);

  for (i = 0; i < MAX_RX_NODES; i++) {
    DebugSerial.print("node ");
    DebugSerial.print(i);
    DebugSerial.print(": enabled=");
    DebugSerial.print(rxNodes[i].enabled);
    DebugSerial.print(", freq=");
    DebugSerial.println(rxNodes[i].freq);
  }
}

void saveConfigToEEPROM() {
  int i;
  int idx;
  
  DebugSerial.println("Saving configuration to EEPROM");
  idx = 2;
  for (i = 0 ; i < sizeof(cfg); i++) {
    EEPROM.write(idx++, ((byte *)(&cfg))[i]);
  }
}

void saveRxEnableToEEPROM() {
  int i;
  int idx;

  DebugSerial.println("Saving rx enables to EEPROM");
  idx = 2 + sizeof(cfg);
  for (i = 0; i < MAX_RX_NODES; i++) {
    EEPROM.write(idx++, rxNodes[i].enabled);
  }
}

void saveRxFrequenciesToEEPROM() {
  int i;
  int idx;

  DebugSerial.println("Saving rx frequencies to EEPROM");
  idx = 2 + sizeof(cfg) + MAX_RX_NODES;
  for (i = 0; i < MAX_RX_NODES; i++) {
    EEPROM.write(idx++, rxNodes[i].freq & 0xff);
    EEPROM.write(idx++, (rxNodes[i].freq >> 8) & 0xff);
  }
}

// State machine to handle incoming messages
void rxMessageStateMachine() {
  const int RX_MSG_BUF_SIZE = 64;
  enum rxState_e { RX_STATE_IDLE, RX_STATE_RECEIVING, RX_STATE_WAIT_NEWLINE, RX_STATE_ERROR };
  static rxState_e rxState = RX_STATE_IDLE;
  static int rxMsgLen = 0;
  static char rxMsgBuf[RX_MSG_BUF_SIZE];
  char ch;
  bool done = false;

  while ((! done) && Serial1.available()) {
    ch = Serial1.read();

    if ((ch == '#') || (ch == '?')) {
      // Begin new message
      rxMsgLen = 0;
      rxMsgBuf[rxMsgLen++] = ch;
      rxState = RX_STATE_RECEIVING;
    }
    else {
      if (rxState == RX_STATE_RECEIVING) {
        if (rxMsgLen < RX_MSG_BUF_SIZE) {
          if (ch == '\r' && (rxMsgLen >= 4)) {
            rxMsgBuf[rxMsgLen] = 0;  // NULL terminate the message
            rxState = RX_STATE_WAIT_NEWLINE;
          }
          else {
            rxMsgBuf[rxMsgLen++] = ch;
          }
        }
        else {
          rxState = RX_STATE_ERROR;
        }
      }
      else if (rxState == RX_STATE_WAIT_NEWLINE) {
        if (ch == '\n') {

          // We got one, now let's see what it contains!
          processRxMessage(rxMsgBuf, rxMsgLen);

          rxState = RX_STATE_IDLE;
          done = true;  // only process 1 message then we're done
        }
        else {
          rxState = RX_STATE_ERROR;
        }
      }
      else if (rxState == RX_STATE_ERROR) {
        // Nothing to do here
      }
    }
  }
}

void processRxMessage(char *msg, int len) {
  int i;

  if (msg[0] == '?' && (len == 4)) {
    // Process queries
    if (! strncmp(&msg[1], "VER", 3)) {
      ////////////////////////////////////////////////////////////////////// ?VER
      sendMsgVER();
    }
    else if (! strncmp(&msg[1], "FRA", 3)) {
      ////////////////////////////////////////////////////////////////////// ?FRA
      sendMsgFRA();
    }
    else if (! strncmp(&msg[1], "REN", 3)) {
      ////////////////////////////////////////////////////////////////////// ?REN
      sendMsgREN();
    }
    else if (! strncmp(&msg[1], "CFG", 3)) {
      ////////////////////////////////////////////////////////////////////// ?CFG
      sendMsgCFG();
    }
    else if (! strncmp(&msg[1], "RSS", 3)) {
      ////////////////////////////////////////////////////////////////////// ?RSS
      sendMsgRSS(false);
    }
  }
  else if(msg[0] == '#' && (len >= 4)) {
    #define MAX_FIELDS    16
    int numFields;
    const char *fields[MAX_FIELDS];

    // Split the message into fields
    numFields = splitFields(&msg[5], fields, MAX_FIELDS);

    //for (i = 0; i < numFields; i++) {
    //  Serial1.print("field ");
    //  Serial1.print(i);
    //  Serial1.print(":");
    //  Serial1.println(fields[i]);
    //}

    // Process commands
    if (! strncmp(&msg[1], "FRA", 3)) {
      ////////////////////////////////////////////////////////////////////// #FRA
      if (numFields == MAX_RX_NODES) {
        int freq;
        for (i = 0; i < MAX_RX_NODES; i++) {
          if (installedNodes[i]) {
            if (convertToInt(fields[i], freq, FREQ_MIN, FREQ_MAX, false)) {
              setRxModuleFreq(i, freq);
            }
          }
        }
        saveRxFrequenciesToEEPROM();
        sendMsgFRA();
      }
    }
    else if (! strncmp(&msg[1], "REN", 3)) {
      ////////////////////////////////////////////////////////////////////// #REN
      if (numFields == MAX_RX_NODES) {
        for (i = 0; i < MAX_RX_NODES; i++) {
          if (installedNodes[i]) {
            // Only allow enable on nodes that are installed
            convertToInt(fields[i], rxNodes[i].enabled, 0, 1, true);
          }
        }
        saveRxEnableToEEPROM();
        sendMsgREN();
      }
    }
    else if (! strncmp(&msg[1], "CFG", 3)) {
      ////////////////////////////////////////////////////////////////////// #CFG
      if (numFields == 5) {
        convertToInt(fields[0], cfg.rssiReportIntervalMs, RSSI_REPORT_INTERVAL_MIN, 10000, true);
        convertToInt(fields[1], cfg.calOffset, 0, ADC_MAX, true);
        convertToInt(fields[2], cfg.calThresh, 0, ADC_MAX, true);
        convertToInt(fields[3], cfg.trigThresh, 0, ADC_MAX, true);
        convertToInt(fields[4], cfg.rssiFilterConstant, 0, 1000, true);
        saveConfigToEEPROM();
        sendMsgCFG();
      }
    }
    else if (! strncmp(&msg[1], "RAC", 3)) {
      ////////////////////////////////////////////////////////////////////// #RAC
      if (numFields == 1) {

        resetAllRxNodes();      // reset all receiver nodes
        raceNumber++;           // increment race number
        raceTimer = 0;          // reset race timer
        sendMsgRAC();
      }
    }
    else if (! strncmp(&msg[1], "RBT", 3)) {
      ////////////////////////////////////////////////////////////////////// #RBT
      // Reboot CPU
      sendMsgRBT();
      while (handleQueuedSerialTxMsg());  // make sure all pending message have been transmitted
      DebugSerial.println("Rebooting...");
      delay(100);
      CPU_REBOOT();
    }
    else if (! strncmp(&msg[1], "DFT", 3)) {
      ////////////////////////////////////////////////////////////////////// #DFT
      // Revert to default settings and reboot
      // Write invalid magic number to EEPROM, this will cause defaults to be written upon the next bootup
      EEPROM.write(0, 0);
      sendMsgDFT();
      while (handleQueuedSerialTxMsg());  // make sure all pending message have been transmitted
      DebugSerial.println("Rebooting...");
      delay(100);
      CPU_REBOOT();
    }
  }

#if 0
  Serial1.print("%DBG");
  Serial1.print("\t");
  Serial1.print("Got message len=");
  Serial1.print(len);
  Serial1.print(":");
  Serial1.print(msg);
  Serial1.println();
#endif
}

void sendMsgVER() {
  serialTxMsg_t msg;
  int len;
  
  len = sprintf(msg.buf, "%s\t%s\t%s\r\n",
                "@VER",
                PROTOCOL_VERSION,
                FIRMWARE_VERSION);
                
  msg.len = len;
  serialTxMsgQueue.add(msg);
}

void sendMsgFRA() {
  serialTxMsg_t msg;
  int len;
  int i;
    
  len = sprintf(msg.buf, "%s", "@FRA");
  
  for (i = 0; i < MAX_RX_NODES; i++) {
    len += sprintf(msg.buf + len, "\t%d", rxNodes[i].freq);
  }
  
  len += sprintf(msg.buf + len, "\r\n");
  
  msg.len = len;
  serialTxMsgQueue.add(msg);
}

void sendMsgREN() {
  serialTxMsg_t msg;
  int len;
  int i;
    
  len = sprintf(msg.buf, "%s", "@REN");
  
  for (i = 0; i < MAX_RX_NODES; i++) {
    len += sprintf(msg.buf + len, "\t%d", rxNodes[i].enabled);
  }
  
  len += sprintf(msg.buf + len, "\r\n");
  
  msg.len = len;
  serialTxMsgQueue.add(msg);
}

void sendMsgCFG() {
  serialTxMsg_t msg;
  int len;
  
  len = sprintf(msg.buf, "%s\t%d\t%d\t%d\t%d\t%d\r\n",
                "@CFG",
                cfg.rssiReportIntervalMs,
                cfg.calOffset,
                cfg.calThresh,
                cfg.trigThresh,
                cfg.rssiFilterConstant);
                
  msg.len = len;
  serialTxMsgQueue.add(msg);
}

void sendMsgRAC() {
  serialTxMsg_t msg;
  int len;
  
  len = sprintf(msg.buf, "%s\t%d\t%lu.%.3lu\r\n",
                "@RAC",
                raceNumber,
                raceTimer / 1000, raceTimer % 1000);
                
  msg.len = len;
  serialTxMsgQueue.add(msg);
}

void sendMsgHRT() {
  serialTxMsg_t msg;
  int len;
  
  len = sprintf(msg.buf, "%s\t%d\t%lu.%.3lu\t%d\r\n",
                "%HRT",
                raceNumber,
                raceTimer / 1000, raceTimer % 1000,
                heartbeatCounter);
                
  msg.len = len;
  serialTxMsgQueue.add(msg);
}

void sendMsgRSS(bool event) {
  serialTxMsg_t msg;
  int len;
  int i;
    
  len = sprintf(msg.buf, "%s\t%d\t%lu.%.3lu",
                event ? "%RSS" : "@RSS",
                raceNumber,
                raceTimer / 1000, raceTimer % 1000);
                
  for (i = 0; i < MAX_RX_NODES; i++) {
    len += sprintf(msg.buf + len, "\t");
    if (rxNodes[i].enabled) {
      len += sprintf(msg.buf + len, "%d", (int) rxNodes[i].rssiPeakSmoothedForRSS);
      rxNodes[i].rssiPeakSmoothedForRSS = 0;  // reset peak for next message
    }
  }
  
  len += sprintf(msg.buf + len, "\r\n");
  
  msg.len = len;
  serialTxMsgQueue.add(msg);
}

void sendMsgLAP(int rxNode) {
  serialTxMsg_t msg;
  int len;
  
  len = sprintf(msg.buf, "%s\t%d\t%d.%.3d\t%d\t%d\t%d.%.3d\t%d\t%d\t%d\r\n",
                "%LAP",
                raceNumber,
                rxNodes[rxNode].lapTimestamp / 1000, rxNodes[rxNode].lapTimestamp % 1000,
                rxNode,
                rxNodes[rxNode].lapCount,
                rxNodes[rxNode].lapTime / 1000, rxNodes[rxNode].lapTime % 1000,
                rxNodes[rxNode].lapPeakRssiSmoothed,
                rxNodes[rxNode].rssiTrigger,
                rxNodes[rxNode].rssiTrigger - cfg.trigThresh);
                
  msg.len = len;
  serialTxMsgQueue.add(msg);
}

void sendMsgRBT() {
  serialTxMsg_t msg;
  int len;
  
  len = sprintf(msg.buf, "%s\r\n",
                "@RBT");
                
  msg.len = len;
  serialTxMsgQueue.add(msg);
}

void sendMsgDFT() {
  serialTxMsg_t msg;
  int len;
  
  len = sprintf(msg.buf, "%s\r\n",
                "@DFT");
                
  msg.len = len;
  serialTxMsgQueue.add(msg);
}

// Reset all race timer related data for all receiver nodes
void resetAllRxNodes() {
  int i;

  for (i = 0; i < MAX_RX_NODES; i++) {
    rxNodes[i].autoCalibrationMode = true;
    rxNodes[i].crossing = false;
    rxNodes[i].rssiTrigger = 1;

    rxNodes[i].rssiPeakRaw = 0;
    rxNodes[i].rssiPeakRawTimestamp = 0;

    rxNodes[i].rssiPeakSmoothed = 0;

    rxNodes[i].rssiPeakSmoothedForRSS = 0;

    rxNodes[i].lapCount = 0;
    rxNodes[i].lapTimestamp = 0;
    rxNodes[i].lapTime = 0;
    rxNodes[i].lapPeakRssiRaw = 0;
    rxNodes[i].lapPeakRssiSmoothed = 0;
  }
}

// Convert the specified srtring into an int value, performing rance checks on min and max.
bool convertToInt(const char *str, int &result, int min, int max, bool allowZero) {
  bool success = false;
  long n;
  char *endPtr;

  errno = 0;
  n = strtol(str, &endPtr, 10);
  if ((errno != ERANGE) && (*endPtr == 0) && (str != endPtr)) {
    if (((allowZero && (n == 0))) || ((n >= min) && (n <= max))) {
      result = (int) n;
      success = true;
    }
  }

  return success;
}

// Split the provided message into tab-delimited fields, up to the specified number of fields.
int splitFields(char *msg, const char *fields[], int maxFields) {
  char *tok;
  int n = 0;

  while ((n < maxFields) && (msg != NULL)) {
    tok = strsep(&msg, "\t");
    fields[n] = tok;
    n++;
  }

  return n;
}

// Initialize receiver module registers
void initRxModuleRegisters(int rxNode) {

  // Attempt at disabling auto gain control to obtain a more even response
  //writeSPIReg(rxNode, RTC6715_RECEIVER_CTRL_REG_1, 0b00000000000000000000);
  //writeSPIReg(rxNode, RTC6715_RECEIVER_CTRL_REG_2, 0b00010010000000011000);
}

// Set the frequency for the specified receiver node
void setRxModuleFreq(int rxNode, int frequency) {
  int i;

  for (i = 0; i < NUM_VTX_TABLE_ENTRIES; i++) {
    if (frequency == vtxFreqTable[i]) {
      rxNodes[rxNode].freq = frequency;
      writeSPIReg(rxNode, RTC6715_SYNTH_REG_B, vtxHexTable[i]);
      break;
    }
  }
}

// Write a SPI register in the specified receiver node
void writeSPIReg(int rxNode, uint8_t reg, uint32_t val) {
  uint32_t data;

  data = reg | (1 << 4) | (val << 5);

  SPI.beginTransaction(spiSettings);
  digitalWrite(spiSSPins[rxNode], LOW);
  SPI.transfer(data & 0xFF);
  SPI.transfer((data >> 8) & 0xFF);
  SPI.transfer((data >> 16) & 0xFF);
  SPI.transfer((data >> 24) & 0xFF);
  digitalWrite(spiSSPins[rxNode], HIGH);
  SPI.endTransaction();
}

//
// ADC interrupt routine
//    Retrieve the ADC reading for the current node, perform filtering, and then start
//    the ADC conversion for the next channel.
//    Keep processing to a minimum in here. No lengthy calculations, no printing, etc.
//
void adc0_isr() {
  volatile static int curRxNode = 0;
  volatile int rssiValRaw;
  volatile int sum;

  //digitalWrite(ledPin, LOW);

  // Read the raw ADC value for the current channel
  rssiValRaw = adc->adc0->readSingle();

  measurementCount++;

  // Scale the ADC value into the range of 0 to 1023
  if (rssiValRaw >= ADC_OFFSET) {
    rssiValRaw -= ADC_OFFSET;
  }
  else {
    rssiValRaw = 0;
  }
  rssiValRaw = rssiValRaw * ADC_MULTIPLIER / ADC_DIVIDER;
  if (rssiValRaw > ADC_MAX) {
    rssiValRaw = ADC_MAX;
  }

  // Maintain a moving average of ADC samples for this node
  sum = rssiValMovingAvgSums[curRxNode];
  sum = sum + rssiValRaw - ((sum + (sum >> (ADC_FILTER_BITS - 1))) >> ADC_FILTER_BITS);
  rssiValMovingAvgSums[curRxNode] = sum;

  // Select the next receiver node
  curRxNode++;
  if (curRxNode == MAX_RX_NODES) {
    curRxNode = 0;
  }

  // Start the conversion on the selected node
  adc->startSingleRead(rssiPins[curRxNode], ADC_0);

  //digitalWrite(ledPin, HIGH);
}

