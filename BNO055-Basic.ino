
#define UPDATE_HZ 1

// ######################## Sensor stuff
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define BNO055_SAMPLERATE_DELAY_MS (100)
Adafruit_BNO055 bno = Adafruit_BNO055(55);


void displaySensorDetails(void)
{
  sensor_t sensor;
  bno.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}



// Now we need to define the "state" of the world.
class Telescope_state {
  
  public:
    float  site_time    = 0;            // Current time GMT
    float  local_az     = 0;            // Measured from true North
    float  local_alt    = 0;           // elevation
    
    float  Xg            = 0;
    float  Yg            = 0;
    float  Zg            = 0;
    
    uint8_t  sys        = 0;
    uint8_t  gyro       = 0;
    uint8_t  accel      = 0;
    uint8_t  mag        = 0;
    
    float  baro         = 0;
    float  temperature  = 0;
    
    unsigned long    update_time  = 0;            // How long did the ticker take?
    unsigned long    current_time  = 0;
    unsigned long    current_time_updated = 0;    // Millis of the last time we updated the time.
    
  
} state;








// Now work on the interrupt routine to update the structure
#include <Ticker.h>

Ticker updateState;

const float alpha = 0.5;
float fXg, fYg, fZg;

void updateStateProcess(){
  
  unsigned long start = millis();

  sensors_event_t event;
  bno.getEvent(&event);

  state.local_az = event.orientation.x;
  state.local_alt = event.orientation.y;
  
  bno.getCalibration(&state.sys, &state.gyro, &state.accel, &state.mag);
  
  state.update_time = millis() - start;
}





void setup() {
  Serial.begin(115200);
  
  // Dropping reset for a few seconds.. we'll see if this helps
  Serial.println("Resetting BNO055");
  pinMode(16, OUTPUT);
  digitalWrite(16, LOW);
  delay(1000);
  digitalWrite(16, HIGH);
  Serial.println("Done with reset.");
  
  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
  delay(1000);
  
  displaySensorDetails();

  updateState.attach(1.0/UPDATE_HZ, updateStateProcess);

 

}


int current = 0;


void loop() {
  
//  if ( millis() > current + 1000*(1.0/UPDATE_HZ)) {  
  if ( millis() > current + 1000) {  

    
    Serial.print("Update Time: ");
    Serial.println( state.update_time );
    
    Serial.print("=================== AZ: ");
    Serial.println(state.local_az,10);
    Serial.print("=================== Alt: ");
    Serial.println(state.local_alt,10);
    
    Serial.print(F("Calibration: "));
    Serial.print(state.sys, DEC);
    Serial.print(F(" "));
    Serial.print(state.gyro, DEC);
    Serial.print(F(" "));
    Serial.print(state.accel, DEC);
    Serial.print(F(" "));
    Serial.println(state.mag, DEC);
    
    current = millis();
  
  }

}
