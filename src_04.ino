int TRIG_PIN_LEAVE = 26;
int TRIG_PIN_ARRIVE = 14;
int ECHO_PIN_LEAVE = 27;
int ECHO_PIN_ARRIVE = 25;

const unsigned int MAX_DIST = 23323;


void setup()
{

    pinMode(TRIG_PIN_ARRIVE, OUTPUT);
    pinMode(TRIG_PIN_LEAVE, OUTPUT);
    pinMode(ECHO_PIN_ARRIVE, LOW);
    pinMode(ECHO_PIN_LEAVE,LOW);
	Serial.begin(115200);
}

float getDistance(int pin_trig, int pin_echo)
{
  digitalWrite(pin_trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(pin_trig, LOW);

  unsigned long t1;
  unsigned long t2;
  unsigned long pulse_width;
  float distance = 0.f;
  while ( digitalRead(pin_echo) == 0){}
  // Measure how long the echo pin was held high (pulse width)
  // Note: the micros() counter will overflow after ~70 min
  t1 = micros();
  while ( digitalRead(pin_echo) == 1);
  t2 = micros();
  // Wait for pulse on echo pin
  pulse_width = t2 - t1;

  // Calculate distance in centimeters and meters. The constants
  // are found in the datasheet, and calculated from the assumed speed 
  //of sound in air at sea level (~340 m/s).
  distance = pulse_width * 0.00017;
  return distance;
}


void loop()
{
  float distance_leave,distance_arrive;

  // Hold the trigger pin high for at least 10 us
  
  distance_arrive = getDistance(TRIG_PIN_ARRIVE,ECHO_PIN_ARRIVE);
  distance_leave  = getDistance(TRIG_PIN_LEAVE,ECHO_PIN_LEAVE);

  if (distance_arrive < 0.5 && distance_leave > 0.5)
    {
      Serial.println("Car Arriving");
    }
  else if (distance_arrive < 0.5 && distance_leave < 0.5)
    {
      Serial.println("Car on Top");
    }
  else if (distance_arrive > 0.5 && distance_leave < 0.5)
    {
      Serial.println("Car Leaving");
    }
  else
    {
      Serial.println("No car detected");
    }
  


  // Wait at least 60ms before next measurement
  delay(60);
	
}
