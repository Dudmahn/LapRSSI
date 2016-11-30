IntervalTimer myTimer;
int heartbeat;
float timer_value = 1;
float timer_save[8];
int pilot_rssi[8];
int pilot_rssi_threshold[8] = {350,350,350,350,350,350,350,350,};
int pin[8] = {9,8,7,6,5,4,3,2};
int pilot_number;
int scan = 0;

void setup()
{
  Serial.begin(9600); //opens serial port, sets data rate to 9600 bps
  myTimer.begin(timer, 1000);  // increment timer every 0.001 seconds
  heartbeat = 1;
}

void timer(void)
{
  timer_value = timer_value + 0.001;
  for (int i = 0; i < 8; i++){
  {
    pilot_rssi[i] = analogRead(pin[i]);
    if (pilot_rssi[i] > pilot_rssi_threshold[i]) {timer_save[i] = timer_value;}
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
      Serial.print(timer_value);
      Serial.print("\t");
      Serial.println("LapRSSI");
      delay(1000);
      heartbeat = heartbeat + 1;
     }
    else
    { 
      if (timer_save[i] > 0)
      {
        noInterrupts();
        Serial.print(" @");
        Serial.print("\t");  
        Serial.print(heartbeat);
        Serial.print("\t");
        Serial.print(pilot_number[i]);
        Serial.print("\t");
        Serial.print(timer_save[i], 3); // needs time stamp
        Serial.print("\t");
        Serial.print(pilot_rssi[i]);
        Serial.print("\t");
        Serial.println("LapRSSI");
        timer_save[i] = 0;
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
      pilot_frequency = Serial.parseInt();
      pilot_rssi_threshold = Serial.parseInt();
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
