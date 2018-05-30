const int RPI_DATA_REQUEST_PIN   = 22; // Active high
const int ENCODER_PIN            = 3;  // Analog

bool toWrite = true;

void setup() {
  pinMode(RPI_DATA_REQUEST_PIN, INPUT);
  //pinMode (ENCODER_PIN, INPUT);
  Serial3.begin(9600);  
  Serial.begin(9600);
}

void send_analog_data()
{
   int pwm_in = analogRead(ENCODER_PIN);
   Serial.println (pwm_in);
   Serial3.println(pwm_in);
}

void loop() {  
  // poll request pin
  // better to prevent this from sending data multiple times?
  if (toWrite == true && digitalRead(RPI_DATA_REQUEST_PIN) == HIGH)
  {
      send_analog_data();
      toWrite = false;
  }
  if(toWrite == false && digitalRead(RPI_DATA_REQUEST_PIN) == LOW)
  {
    toWrite = true;
  }
}


