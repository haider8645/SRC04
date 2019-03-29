#include <state_machine.h>
int TRIG_PIN_LEAVE = 26;
int TRIG_PIN_ARRIVE = 14;
int ECHO_PIN_LEAVE = 27;
int ECHO_PIN_ARRIVE = 25;
int LED_INDICATOR_CAR_DETECTION = 13;

const unsigned int MAX_DIST = 23323;

StateMachine sm;
StateMachine::Event sensor_arrive_event = StateMachine::Event::UNDEFINED;
StateMachine::Event sensor_leave_event = StateMachine::Event::UNDEFINED;

int sensor_arrive_last_state = 0;
int sensor_leave_last_state = 0;

int sensor_arrive_current_state  = 0;
int sensor_leave_current_state  = 0;

void setup()
{
    pinMode(TRIG_PIN_ARRIVE, OUTPUT);
    pinMode(TRIG_PIN_LEAVE, OUTPUT);
    pinMode(ECHO_PIN_ARRIVE, LOW);
    pinMode(ECHO_PIN_LEAVE,LOW);
    pinMode(LED_INDICATOR_CAR_DETECTION,OUTPUT);
	  Serial.begin(115200);
    sm.Start();
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
  distance_arrive = getDistance(TRIG_PIN_ARRIVE,ECHO_PIN_ARRIVE);
  distance_leave  = getDistance(TRIG_PIN_LEAVE,ECHO_PIN_LEAVE);

  if (distance_arrive >= 0.2 && distance_arrive <= 0.5)
   {
       sensor_arrive_current_state = HIGH;
   }
   else
   {
       sensor_arrive_current_state = LOW;
   }
   
   if (distance_leave >= 0.2 && distance_leave <= 0.5)
   {
       sensor_leave_current_state = HIGH;
   }
   else
   {
       sensor_leave_current_state = LOW;
   }


   Serial.println("*******************");
   Serial.print("Arrive: ");
   Serial.println(sensor_arrive_current_state);
   Serial.print("Leave: ");
   Serial.println(sensor_leave_current_state);

    if (sensor_arrive_last_state == LOW && sensor_arrive_current_state == HIGH)
        sensor_arrive_event = StateMachine::Event::SENSOR_LOW_TO_HIGH;
    else if (sensor_arrive_last_state == HIGH && sensor_arrive_current_state == LOW)
        sensor_arrive_event = StateMachine::Event::SENSOR_HIGH_TO_LOW;

    if (sensor_leave_last_state == LOW && sensor_leave_current_state == HIGH)
        sensor_leave_event = StateMachine::Event::SENSOR_LOW_TO_HIGH;
    else if (sensor_leave_last_state == HIGH && sensor_leave_current_state == LOW)
        sensor_leave_event = StateMachine::Event::SENSOR_HIGH_TO_LOW;

    sm.SetEvent(sensor_arrive_event,sensor_leave_event);
    sm.SetSensorState(static_cast<StateMachine::SensorState>(sensor_arrive_current_state),static_cast<StateMachine::SensorState>(sensor_leave_current_state));
    bool state_changed = sm.GoToNextState();

    StateMachine::State now = sm.GetCurrentState();
    Serial.print("Current State of SM: ");
    Serial.println(static_cast<int>(now));

    
    if (state_changed)
      {
        //if state has changed then check the following
        StateMachine::State previous_state = sm.GetPreviousState();
        StateMachine::State current_state = sm.GetCurrentState();

        if (previous_state == StateMachine::State::VEHICLE_ON_TOP &&
            current_state == StateMachine::State::VEHICLE_LEAVING)
        {
            //push to cloud
            //vehicle_counter++
            Serial.println("Car detected");
            digitalWrite(LED_INDICATOR_CAR_DETECTION,HIGH);
            delay(500);
            digitalWrite(LED_INDICATOR_CAR_DETECTION,LOW);   
         }
      }
    
    
    //save sensor values for next iteration
    sensor_arrive_last_state = sensor_arrive_current_state;
    sensor_leave_last_state = sensor_leave_current_state;
 


  // Wait at least 60ms before next measurement
  delay(60);
	
}
