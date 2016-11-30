IntervalTimer myTimer;
int heartbeat;
float timervalue = 1;
float timersave[8];
int rssi[8];
int pilotrssithreshold[8] = {350,350,350,350,350,350,350,350,};
int pin[8] = {9,8,7,6,5,4,3,2};
int pilotnumber;
int scan = 0;

void setup()
{
  Serial.begin(9600); //opens serial port, sets data rate to 9600 bps
  myTimer.begin(timer, 1000);  // increment timer every 0.001 seconds
  heartbeat = 1;
}

void timer(void)
{
  timervalue = timervalue + 0.001;
  for (int i = 0; i < 8; i++){
  {
    rssi[i] = analogRead(pin[i]);
    if (rssi[i] > pilotrssithreshold[i]) {timersave[i] = timervalue;}
  }
}

void loop()
{
  for (int i = 0; i < 7; i++){
    if (i == 6) 
    {
      Serial.print(" #");
      Serial.print("\t");
      Serial.print(heartbeat);
      Serial.print("\t");
      Serial.print(timervalue);
      Serial.print("\t");
      Serial.println("LapRSSI");
      delay(1000);
      heartbeat = heartbeat + 1;
     }
    else
    { 
      if (timersave[i] > 0)
      {
        noInterrupts();
        Serial.print(" @");
        Serial.print("\t");  
        Serial.print(heartbeat);
        Serial.print("\t");
        Serial.print(pilotnumber[i]);
        Serial.print("\t");
        Serial.print(timersave[i], 3); // needs time stamp
        Serial.print("\t");
        Serial.print(rssi[i]);
        Serial.print("\t");
        Serial.println("LapRSSI");
        timersave[i] = 0;
        interrupts();
        delay(1000);
        heartbeat = heartbeat + 1;
      }
    }
  }
  if (Serial.available() > 0) {
    if (Serial.peek() == '@') {
      Serial.read();
      pilot = Serial.parseInt();
      pilotfrequency = Serial.parseInt();
      pilotrssithreshold = Serial.parseInt();
    }
    If (Serial.peek() == '&') {
      Serial.read();
      Serial.read();
      scan = Serial.parseInt();
        if (scan == 1) {
          // insert scan code here. 
        }
    }
    while (Serial.available() > 0) {
      Serial.read();
    }
  }
}
