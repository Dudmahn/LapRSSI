#include "channels.h"
#include "pinAssignments.h"
#include "rx5808spi.h"
int heartbeat = 1;
float timer_value = 0;
float timer_save[8];
int pilot_rssi[8];
int rssi_threshold[8] = {550,550,550,550,550,550,550,550};
int pin[8] = {0,1,2,3,4,5,6,7};
int pilot;
int pilot_frequency[8];
int pilot_number[8];
int scan = 0;
int cal;
int rssi_save[8];

elapsedMillis beat;
elapsedMillis beat1;
IntervalTimer myTimer;

void setup()
{
  pinMode(spiDataPin, OUTPUT);
  pinMode(slaveSelectPin, OUTPUT);
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
    Serial.println("LapRSSI");
    heartbeat = heartbeat + 1;
    beat = 0;
  }
  if (beat1 >= 500)
  {
  for  (int i = 0; i < 7; i++)
  {
    if (timer_save[i] != 0)
    {
      Serial.print(" @");
      Serial.print("\t");
      Serial.print(heartbeat);
      Serial.print("\t");
      Serial.print(pilot_number[i]);
      Serial.print("\t");
      Serial.print(timer_save[i], 2);
      Serial.print("\t");
      Serial.print(rssi_save[i]);
      Serial.print("\t");
      Serial.print("Freq=");
      Serial.print(pilot_frequency[i]);
      Serial.print("\t");
      Serial.print("rssiT");
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
