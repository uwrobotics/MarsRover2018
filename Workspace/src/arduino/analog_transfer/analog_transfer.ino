const int RPI_DATA_REQUEST_PIN   = 22; // Active high
const int ENCODER_PIN            = 3;  // Analog

void setup() {
  pinMode(RPI_DATA_REQUEST_PIN, INPUT);
  pinMode (ENCODER_PIN, INPUT);
  Serial3.begin(9600);  
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
  if (digitalRead(RPI_DATA_REQUEST_PIN) == HIGH)
  {
      send_analog_data();
  }
}


