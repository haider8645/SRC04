#include <algorithm>
#include <state_machine.h>
#include <moving_average_filter.h>
#include <median_filter.h>

#define LED_INDICATOR_CAR_DETECTION   13
#define TRIG_PIN_ARRIVE               14
#define ECHO_PIN_ARRIVE               25
#define TRIG_PIN_LEAVE                26
#define ECHO_PIN_LEAVE                27

#define DETECTION_UPPER_THRESHOLD 0.5 // in meters
#define DETECTION_LOWER_THRESHOLD 0.2

#define MINIMUM_DETECTION_DISTANCE 0.0// in meters
#define MAXIMUM_DETECTION_DISTANCE 4.0

//const unsigned int MAX_DIST = 23323;

StateMachine sm;
StateMachine::Event sensor_arrive_event = StateMachine::Event::UNDEFINED;
StateMachine::Event sensor_leave_event  = StateMachine::Event::UNDEFINED;

MovingAverageFilter maf_arrive(5);
MovingAverageFilter maf_leave(5);

MedianFilter mf_arrive(3);
MedianFilter mf_leave(3);

bool sensor_arrive_last_state     = LOW;
bool sensor_arrive_current_state  = LOW;
bool sensor_leave_last_state      = LOW;
bool sensor_leave_current_state   = LOW;

void setup()
{
    pinMode(TRIG_PIN_ARRIVE, OUTPUT);
    pinMode(TRIG_PIN_LEAVE,  OUTPUT);
    pinMode(ECHO_PIN_ARRIVE,    LOW);
    pinMode(ECHO_PIN_LEAVE,     LOW);
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

  if (distance >= MAXIMUM_DETECTION_DISTANCE)
    distance = MAXIMUM_DETECTION_DISTANCE;
  else if (distance <= MINIMUM_DETECTION_DISTANCE)
    distance = MINIMUM_DETECTION_DISTANCE;

  return distance;//clamp(distance,MINIMUM_DETECTION_DISTANCE,MAXIMUM_DETECTION_DISTANCE);
}


void loop()
{
  float distance_leave,distance_arrive;
  distance_arrive = getDistance(TRIG_PIN_ARRIVE,ECHO_PIN_ARRIVE);
  distance_leave  = getDistance(TRIG_PIN_LEAVE,ECHO_PIN_LEAVE);

  mf_arrive.newValue(distance_arrive);
  mf_leave.newValue(distance_leave);

  maf_arrive.newValue(mf_arrive.getResult());
  maf_leave.newValue(mf_leave.getResult());

  float distance_arrive_filtered  = maf_arrive.getResult();
  float distance_leave_filtered   = maf_leave.getResult();

  Serial.print("Distance arrive filtered: ");
  Serial.println(distance_arrive_filtered);

  Serial.print("Distance leave filtered: ");
  Serial.println(distance_leave_filtered);

   if (distance_arrive_filtered >= DETECTION_LOWER_THRESHOLD && distance_arrive_filtered <= DETECTION_UPPER_THRESHOLD)
   {
       sensor_arrive_current_state = HIGH;
   }
   else
   {
       sensor_arrive_current_state = LOW;
   }

   if (distance_leave_filtered >= DETECTION_LOWER_THRESHOLD && distance_leave_filtered <= DETECTION_UPPER_THRESHOLD)
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
    else
        sensor_arrive_event = StateMachine::Event::SENSOR_NO_CHANGE;

    if (sensor_leave_last_state == LOW && sensor_leave_current_state == HIGH)
        sensor_leave_event = StateMachine::Event::SENSOR_LOW_TO_HIGH;
    else if (sensor_leave_last_state == HIGH && sensor_leave_current_state == LOW)
        sensor_leave_event = StateMachine::Event::SENSOR_HIGH_TO_LOW;
    else
        sensor_leave_event = StateMachine::Event::SENSOR_NO_CHANGE;

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
