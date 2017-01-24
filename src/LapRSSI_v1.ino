int heartbeat = 1;
float timer_value = 0;
float timer_save[8];
int pilot_rssi[8];
int rssi_threshold[8] = {550,550,550,550,550,550,550,550};
int pin[8] = {0,1,2,3,4,5,6,7};
int pilot;
int pilot_frequency[8];
int scan = 0;
int cal;
int rssi_save[8];
int ss_Pin[8];
#include "channels.h"
#include "pinAssignments.h"

elapsedMillis beat;
elapsedMillis beat1;
IntervalTimer myTimer;

void setup()
{
  for (int i = 0; i < 8; i++)
  {
    pinMode(i, OUTPUT);
    digitalWrite(i, HIGH);
  }
  pinMode(spiDataPin, OUTPUT);
  pinMode(spiClockPin, OUTPUT);
  Serial.begin(9600); //opens serial port, sets data rate to 9600 bps
  myTimer.begin(timer, 1000);  // increment timer every 0.001 seconds
}

void timer(void)
{
  timer_value = timer_value + 0.001;
  for (int i = 0; i < 8; i++)
  {
    pilot_rssi[i] = analogRead(i);
    if (pilot_rssi[i] > rssi_threshold[i]) 
    {
      rssi_save[i] = pilot_rssi[i];
      timer_save[i] = timer_value;
    }
  }
}

void loop()
{
  if (beat >= 1000)
  {
    Serial.print(" #");
    Serial.print("\t"); 
    Serial.print(heartbeat);
    Serial.print("\t");
    Serial.print(timer_value);
    Serial.print("\t");
    Serial.println(LapRSSI);
    heartbeat = heartbeat + 1;
    beat = 0;
  }
  if (beat1 >= 500)
  {
  for  (int i = 0; i < 8; i++)
  {
    if (timer_save[i] != 0)
    {
      Serial.print(" @");
      Serial.print("\t");
      Serial.print(heartbeat);
      Serial.print("\t");
      Serial.print(i);
      Serial.print("\t");
      Serial.print(timer_save[i], 2);
      Serial.print("\t");
      Serial.print(rssi_save[i]);
      Serial.print("\t");
      Serial.print("Freq=");
      Serial.print(pilot_frequency[i]);
      Serial.print("\t");
      Serial.print("rssi");
      Serial.print(rssi_threshold[i]);
      Serial.print("\t");
      Serial.println("LapRSSI");
      interrupts();
      timer_save[i] = 0;
    }
  }
  beat1 = 0;
  }
  if (Serial.available() > 0) 
  {
    if (Serial.peek() == '@') // @ pilot pilot_frequency rssi_threshold
    {
      Serial.read();
      pilot = Serial.parseInt();
      pilot_frequency[pilot] = Serial.parseInt();
      rssi_threshold[pilot] = Serial.parseInt();
      // find the channel in the list and then change the frequency for that receiver.
      for  (int i = 0; i < 48; i++)
        {
          if (pilot_frequency[pilot] == channelFreqTable[i])
            {
              setChannelModule(i);
            }
        }
    }
    if (Serial.peek() == '&') // frequency scanner.
    {
      Serial.read();
      Serial.read();
      scan = Serial.parseInt();
      if (scan == 1)
      {
        // insert scan code here.
      }
    }
    if (Serial.peek() == '$') // pilot RSSI calibration
    {
      Serial.read();
      pilot = Serial.parseInt();
      Serial.read();
      cal = 0;
      for  (int i = 0; i < 5; i++)
      {
        pilot_rssi[pilot] = analogRead(pin[pilot]);
        cal = (cal + pilot_rssi[pilot]);
        delay (1000);
        Serial.print(cal);
      }
      cal = (cal / 5);
      Serial.print(" $");
      Serial.print("\t"); 
      Serial.print(pilot);
      Serial.print("\t");
      Serial.print(cal);
      Serial.print("\t");
      Serial.println("LapRSSI");
      rssi_threshold[pilot] = cal;
    }
  }
  while (Serial.available() > 0) 
  {
    Serial.read();
  }
}


void setupSPIpins() {
    // SPI pins for RX control
    pinMode (pilot, OUTPUT);
    pinMode (spiDataPin, OUTPUT);
    pinMode (spiClockPin, OUTPUT);

}

void SERIAL_SENDBIT1() {
    digitalWrite(spiClockPin, LOW);
    delayMicroseconds(1000);

    digitalWrite(spiDataPin, HIGH);
    delayMicroseconds(1000);
    digitalWrite(spiClockPin, HIGH);
    delayMicroseconds(1000);

    digitalWrite(spiClockPin, LOW);
    delayMicroseconds(1000);
}

void SERIAL_SENDBIT0() {
    digitalWrite(spiClockPin, LOW);
    delayMicroseconds(1000);

    digitalWrite(spiDataPin, LOW);
    delayMicroseconds(1000);
    digitalWrite(spiClockPin, HIGH);
    delayMicroseconds(1000);

    digitalWrite(spiClockPin, LOW);
    delayMicroseconds(1000);
}

void SERIAL_ENABLE_LOW() {
    delayMicroseconds(1000);
    digitalWrite(pilot, LOW);
    delayMicroseconds(1000);
}

void SERIAL_ENABLE_HIGH() {
    delayMicroseconds(1000);
    digitalWrite(pilot, HIGH);
    delayMicroseconds(1000);
}

void setChannelModule(uint8_t channel) {
    uint8_t i;
    uint16_t channelData;

    channelData = pgm_read_word_near(channelTable + channel);

    // bit bang out 25 bits of data
    // Order: A0-3, !R/W, D0-D19
    // A0=0, A1=0, A2=0, A3=1, RW=0, D0-19=0
    SERIAL_ENABLE_HIGH();
    delayMicroseconds(1000);
    SERIAL_ENABLE_LOW();

    SERIAL_SENDBIT0();
    SERIAL_SENDBIT0();
    SERIAL_SENDBIT0();
    SERIAL_SENDBIT1();

    SERIAL_SENDBIT0();

    // remaining zeros
    for (i = 20; i > 0; i--) {
        SERIAL_SENDBIT0();
    }

    // Clock the data in
    SERIAL_ENABLE_HIGH();
    delayMicroseconds(1000);
    SERIAL_ENABLE_LOW();

    // Second is the channel data from the lookup table
    // 20 bytes of register data are sent, but the MSB 4 bits are zeros
    // register address = 0x1, write, data0-15=channelData data15-19=0x0
    SERIAL_ENABLE_HIGH();
    SERIAL_ENABLE_LOW();

    // Register 0x1
    SERIAL_SENDBIT1();
    SERIAL_SENDBIT0();
    SERIAL_SENDBIT0();
    SERIAL_SENDBIT0();

    // Write to register
    SERIAL_SENDBIT1();

    // D0-D15
    //   note: loop runs backwards as more efficent on AVR
    for (i = 16; i > 0; i--) {
        // Is bit high or low?
        if (channelData & 0x1) {
            SERIAL_SENDBIT1();
        }
        else {
            SERIAL_SENDBIT0();
        }
        // Shift bits along to check the next one
        channelData >>= 1;
    }

    // Remaining D16-D19
    for (i = 4; i > 0; i--) {
        SERIAL_SENDBIT0();
    }

    // Finished clocking data in
    SERIAL_ENABLE_HIGH();
    delayMicroseconds(1);

    digitalWrite(pilot, LOW);
    digitalWrite(spiClockPin, LOW);
    digitalWrite(spiDataPin, LOW);
    digitalWrite(pilot, HIGH);
}
