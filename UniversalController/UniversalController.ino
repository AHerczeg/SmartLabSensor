#include "MPU9150.h"
#include "Si1132.h"
#include "Si70xx.h"
#include "math.h"
#include "rest_client.h"

//// ***************************************************************************

//// Initialize application variables
#define RAD_TO_DEGREES 57.2957795131
#define DEG_TO_RADIANS 0.0174533
#define PI 3.1415926535
#define ACCEL_SCALE 2 // +/- 2g

int SENSORDELAY = 100;  //// 500; //3000; // milliseconds (runs x1)
int EVENTSDELAY = 1000; //// milliseconds (runs x10)
int OTAUPDDELAY = 7000; //// milliseconds (runs x1)
int SLEEP_DELAY = 0;    //// 40 seconds (runs x1) - should get about 24 hours on 2100mAH, 0 to disable and use RELAX_DELAY instead
String SLEEP_DELAY_MIN = "15"; // seconds - easier to store as string then convert to int
String SLEEP_DELAY_STATUS = "OK"; // always OK to start with
int RELAX_DELAY = 5; // seconds (runs x1) - no power impact, just idle/relaxing

// Variables for the I2C scan
byte I2CERR, I2CADR;

//// ***************************************************************************
//// ***************************************************************************

int I2CEN = D2;
int ALGEN = D3;
int LED = D7;

int SOUND = A0;
double SOUNDV = 0; //// Volts Peak-to-Peak Level/Amplitude

int POWR1 = A1;
int POWR2 = A2;
int POWR3 = A3;
double POWR1V = 0; //Watts
double POWR2V = 0; //Watts
double POWR3V = 0; //Watts

int SOILT = A4;
double SOILTV = 0; //// Celsius: temperature (C) = Vout*41.67-40 :: Temperature (F) = Vout*75.006-40

int SOILH = A5;
double SOILHV = 0; //// Volumetric Water Content (VWC): http://www.vegetronix.com/TechInfo/How-To-Measure-VWC.phtml

bool BMP180OK = false;
double BMP180Pressure = 0;    //// hPa
double BMP180Temperature = 0; //// Celsius
double BMP180Altitude = 0;    //// Meters

bool Si7020OK = false;
double Si7020Temperature = 0; //// Celsius
double Si7020Humidity = 0;    //// %Relative Humidity

bool Si1132OK = false;
double Si1132UVIndex = 0; //// UV Index scoring is as follows: 1-2  -> Low, 3-5  -> Moderate, 6-7  -> High, 8-10 -> Very High, 11+  -> Extreme
double Si1132Visible = 0; //// Lux
double Si1132InfraRd = 0; //// Lux

MPU9150 mpu9150;
bool ACCELOK = false;
int cx, cy, cz, ax, ay, az, gx, gy, gz;
double tm; //// Celsius
int oldXSpeed = 0;
int oldYSpeed = 0;
int oldZSpeed = 0;
int oldXTilt = 0;
int oldYTilt = 0;


bool lock = true;

int maxDegree = 0;

int mode = 0;
int lastMode = 0;

int colour = 1;
int lastColour = 0;

int lastBrightness = 0;

int lastZone = 2;

bool lastKettle = false;

bool colourChange = false;

bool angleChange = false;

unsigned long totalTime = 0;

int totalBattery = 0;

int loopCounter = 0;

Timer sleepTimer(60000, startSleep);

bool isSleeping = false;

RestClient client = RestClient("sccug-330-04.lancs.ac.uk",8000);

const char* path = "/Bulb/2/1";

typedef struct {
    int pin;
    int delay;
} LED_PARAM;

LED_PARAM ledParams[1] = {
    {LED, 10000}
};

Thread* ledThread;

Thread* shakeThread;

os_thread_return_t shakeDetect(void* param){
  for(;;){


    //// powers up sensors
    digitalWrite(I2CEN, HIGH);
    digitalWrite(ALGEN, HIGH);

    //// allows sensors time to warm up
    delay(SENSORDELAY);     //// delay timer



    readMPU9150();          //// reads compass, accelerometer and gyroscope data

    float speed = abs(ax + ay + az - oldXSpeed - oldYSpeed - oldZSpeed);

    String tempStr = "";
    String sensorString = "";
    sensorString = tempStr + speed;
    //Serial.println(sensorString);

    int currentX = (int)getXtiltY(ax, ay);

    if(lock){
      if (speed > 35000) {
        sleepTimer.reset();
        if(isSleeping)
          endSleep();
        sensorString = tempStr + "Shake at speed  " + speed;
        Serial.println(sensorString);
        if(mode > 3)
          mode = 0;
        else
          mode++;
        delay(1500);
      }

      if(gx < -1000 || gx > 3000){
        if (currentX > 160 && currentX < 190){
          sleepTimer.reset();
          if(isSleeping)
            endSleep();
          Serial.println("Unlock");
          lock = false;
          delay(2000);
        }
      }

    } else {
      if(gx < -1000 || gx > 3000){
        if(currentX > 320 || currentX < 10){
          sleepTimer.reset();
          if(isSleeping)
            endSleep();
          Serial.println("Lock");
          lock = true;
          delay(2000);
        }
      }
    }

    oldXSpeed = ax;
    oldYSpeed = ay;
    oldZSpeed = az;
  }
}

os_thread_return_t ledBlink(void* param){
    LED_PARAM *p = (LED_PARAM*)param;

    // Start never ending loop
    for(;;){
      switch(mode){
        case 1: digitalWrite(LED, HIGH);
                delay(200);
                digitalWrite(LED, LOW);
                delay(1000);
                break;

        case 2: digitalWrite(LED, HIGH);
                delay(200);
                digitalWrite(LED, LOW);
                delay(200);
                digitalWrite(LED, HIGH);
                delay(200);
                digitalWrite(LED, LOW);
                delay(1000);
                break;

        case 3: digitalWrite(LED, HIGH);
                delay(200);
                digitalWrite(LED, LOW);
                delay(200);
                digitalWrite(LED, HIGH);
                delay(200);
                digitalWrite(LED, LOW);
                delay(200);
                digitalWrite(LED, HIGH);
                delay(200);
                digitalWrite(LED, LOW);
                delay(1000);
                break;
        case 4: digitalWrite(LED, HIGH);
                delay(100);
                digitalWrite(LED, LOW);
                delay(100);
                digitalWrite(LED, HIGH);
                delay(100);
                digitalWrite(LED, LOW);
                delay(100);
                digitalWrite(LED, HIGH);
                delay(100);
                digitalWrite(LED, LOW);
                delay(100);
                digitalWrite(LED, HIGH);
                delay(100);
                digitalWrite(LED, LOW);
                delay(1000);
                break;

        default: digitalWrite(LED, LOW);
                 delay(500);
      }

    }


}
//// ***************************************************************************

//// SYSTEM_MODE(SEMI_AUTOMATIC);

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void setPinsMode()
{
    pinMode(I2CEN, OUTPUT);
    pinMode(ALGEN, OUTPUT);
    pinMode(LED, OUTPUT);

    pinMode(SOUND, INPUT);

    pinMode(POWR1, INPUT);
    pinMode(POWR2, INPUT);
    pinMode(POWR3, INPUT);

    pinMode(SOILT, INPUT);
    pinMode(SOILH, INPUT);

    ledThread = new Thread("led", ledBlink, &ledParams[0]);
    shakeThread = new Thread("shake", shakeDetect);
}

void setup()
{
    // opens serial over USB
    Serial.begin(9600);

    // Set I2C speed
    // 400Khz seems to work best with the Photon with the packaged I2C sensors
    Wire.setSpeed(CLOCK_SPEED_400KHZ);

    Wire.begin();  // Start up I2C, required for LSM303 communication

    // diables interrupts
    noInterrupts();

    // initialises the IO pins
    setPinsMode();

    // initialises MPU9150 inertial measure unit
    initialiseMPU9150();
}

void initialiseMPU9150()
{
  ACCELOK = mpu9150.begin(mpu9150._addr_motion); // Initialize MPU9150

  if (ACCELOK)
  {
      // Clear the 'sleep' bit to start the sensor.
      mpu9150.writeSensor(mpu9150._addr_motion, MPU9150_PWR_MGMT_1, 0);

      /// Set up compass
      mpu9150.writeSensor(mpu9150._addr_compass, 0x0A, 0x00); //PowerDownMode
      mpu9150.writeSensor(mpu9150._addr_compass, 0x0A, 0x0F); //SelfTest
      mpu9150.writeSensor(mpu9150._addr_compass, 0x0A, 0x00); //PowerDownMode

      mpu9150.writeSensor(mpu9150._addr_motion, 0x24, 0x40); //Wait for Data at Slave0
      mpu9150.writeSensor(mpu9150._addr_motion, 0x25, 0x8C); //Set i2c address at slave0 at 0x0C
      mpu9150.writeSensor(mpu9150._addr_motion, 0x26, 0x02); //Set where reading at slave 0 starts
      mpu9150.writeSensor(mpu9150._addr_motion, 0x27, 0x88); //set offset at start reading and enable
      mpu9150.writeSensor(mpu9150._addr_motion, 0x28, 0x0C); //set i2c address at slv1 at 0x0C
      mpu9150.writeSensor(mpu9150._addr_motion, 0x29, 0x0A); //Set where reading at slave 1 starts
      mpu9150.writeSensor(mpu9150._addr_motion, 0x2A, 0x81); //Enable at set length to 1
      mpu9150.writeSensor(mpu9150._addr_motion, 0x64, 0x01); //overvride register
      mpu9150.writeSensor(mpu9150._addr_motion, 0x67, 0x03); //set delay rate
      mpu9150.writeSensor(mpu9150._addr_motion, 0x01, 0x80);

      mpu9150.writeSensor(mpu9150._addr_motion, 0x34, 0x04); //set i2c slv4 delay
      mpu9150.writeSensor(mpu9150._addr_motion, 0x64, 0x00); //override register
      mpu9150.writeSensor(mpu9150._addr_motion, 0x6A, 0x00); //clear usr setting
      mpu9150.writeSensor(mpu9150._addr_motion, 0x64, 0x01); //override register
      mpu9150.writeSensor(mpu9150._addr_motion, 0x6A, 0x20); //enable master i2c mode
      mpu9150.writeSensor(mpu9150._addr_motion, 0x34, 0x13); //disable slv4
    }
    else
    {
      Serial.println("Unable to start MPU5150");
    }
}

void loop(void)
{

    //// prints device version and address

    //Serial.print("Device version: "); Serial.println(System.version());
    //Serial.print("Device ID: "); Serial.println(System.deviceID());
    //Serial.print("WiFi IP: "); Serial.println(WiFi.localIP());

    //// ***********************************************************************


    unsigned long start = millis();

    //// powers up sensors
    digitalWrite(I2CEN, HIGH);
    digitalWrite(ALGEN, HIGH);

    //// allows sensors time to warm up
    delay(SENSORDELAY);     //// delay timer

    //// ***********************************************************************

    readMPU9150();          //// reads compass, accelerometer and gyroscope data

    if(lastMode != mode)
      delay(1000);

    lastMode = mode;

    String tempStr = "";
    String sensorString;
    String responseString = "";

    int brightness = lastBrightness;

    int currentZone = lastZone;

    bool currentKettle = lastKettle;

    int currentZ = abs(getZtiltX(az, ax)-180);

    switch(mode){
      //Zone
      case 1:

              if(currentZ < 30 && !lock && !angleChange){
                angleChange = true;
                currentZone++;
                if(currentZone > 3)
                  currentZone = 0;
                Serial.println("Zone UP");
              } else if (currentZ > 120 && !lock && !angleChange){
                angleChange = true;
                currentZone--;
                if(currentZone < 0)
                  currentZone = 3;
                Serial.println("Zone DOWN");
              } else if (currentZ > 70 && currentZ < 100 && !lock && angleChange){
                angleChange = false;
              }
              if(currentZone != lastZone){
                sleepTimer.reset();
                if(isSleeping)
                  endSleep();
                lastZone = currentZone;
                tempStr = "";
                switch(currentZone){
                  case 0: path = "/Bulb/0/1";
                          break;
                  case 1: path = "/Bulb/1/1";
                          break;
                  case 2: path = "/Bulb/2/1";
                          break;
                  case 3: path = "/Bulb/3/1";
                          break;
                }
                sensorString = tempStr + "{\"Brightness\":100}";
                client.post(path, (const char*) sensorString, &responseString);
                sensorString = tempStr + "{\"Brightness\":0}";
                client.post(path, (const char*) sensorString, &responseString);
                sensorString = tempStr + "{\"Brightness\":100}";
                client.post(path, (const char*) sensorString, &responseString);
                Serial.println(path);
                Serial.println(responseString);
              }
              break;
        // Colour
        case 2:
                sensorString = "{\"Colour\":";
                if(currentZ < 30 && !colourChange && !lock){
                  colourChange = true;
                  colour++;
                  if(colour > 12)
                    colour = 1;
                  //Serial.println("colourChange: true");
                } else if (currentZ > 70 && colourChange){
                  colourChange = false;
                  //Serial.println("colourChange: false");
                }
                if(colourChange){
                  switch(colour){
                    case 1: sensorString = tempStr + sensorString + 0 + "}";
                            break;
                    case 2: sensorString = tempStr + sensorString + 15 + "}";
                            break;
                    case 3: sensorString = tempStr + sensorString + 30 + "}";
                            break;
                    case 4: sensorString = tempStr + sensorString + 45 + "}";
                            break;
                    case 5: sensorString = tempStr + sensorString + 60 + "}";
                            break;
                    case 6: sensorString = tempStr + sensorString + 90 + "}";
                            break;
                    case 7: sensorString = tempStr + sensorString + 120 + "}";
                            break;
                    case 8: sensorString = tempStr + sensorString + 180 + "}";
                            break;
                    case 9: sensorString = tempStr + sensorString + 240 + "}";
                            break;
                    case 10: sensorString = tempStr + sensorString + 270 + "}";
                            break;
                    case 11: sensorString = tempStr + sensorString + 310 + "}";
                            break;
                    case 12: sensorString = tempStr + sensorString + 345 + "}";
                            break;
                  }
                  if(lastColour != colour){
                    sleepTimer.reset();
                    if(isSleeping)
                      endSleep();
                    lastColour = colour;
                    Serial.println(sensorString);
                    client.post(path, (const char*) sensorString, &responseString);
                    Serial.println(responseString);
                  }
                  lastColour = colour;
                }
                break;
        // Brightness
        case 3:
                sensorString = "{\"Brightness\":";
                if(currentZ < 30 && !lock && !angleChange){
                  angleChange = true;
                  brightness += 25;
                  if(brightness > 100)
                    brightness = 100;
                  Serial.println("Brightness UP");
                } else if (currentZ > 120 && !lock && !angleChange){
                  angleChange = true;
                  brightness -= 25;
                  if(brightness < 0)
                    brightness = 0;
                  Serial.println("Brightness DOWN");
                } else if (currentZ > 70 && currentZ < 100 && !lock && angleChange){
                  angleChange = false;
                }
                if(brightness != lastBrightness){
                  sleepTimer.reset();
                  if(isSleeping)
                    endSleep();
                  lastBrightness = brightness;
                  sensorString = tempStr + sensorString + brightness + "}";
                  Serial.println(sensorString);
                  client.post(path, (const char*) sensorString, &responseString);
                  Serial.println(responseString);
                }

                break;
        // Kettle
        case 4:
                path = "/Kettle";
                if(currentZ < 30 && !lock && !angleChange){
                  angleChange = true;
                  currentKettle = true;
                  Serial.println("Kettle ON");
                } else if (currentZ > 120 && !lock && !angleChange){
                  angleChange = true;
                  currentKettle = false;
                  Serial.println("Kettle OFF");
                } else if (currentZ > 70 && currentZ < 100 && !lock && angleChange){
                  angleChange = false;
                }
                if(currentKettle != lastKettle){
                  sleepTimer.reset();
                  if(isSleeping)
                    endSleep();
                  if(currentKettle){
                    Serial.println(sensorString);
                    sensorString = tempStr + sensorString + "{\"Power\":1}";
                    Serial.println(path);
                    Serial.println(sensorString);
                    client.post(path, (const char*) sensorString, &responseString);
                    Serial.println(responseString);
                  } else {
                    Serial.println(sensorString);
                    sensorString = tempStr + sensorString + "{\"Power\":0}";
                    Serial.println(path);
                    Serial.println(sensorString);
                    client.post(path, (const char*) sensorString, &responseString);
                    Serial.println(responseString);
                  }
                }
                break;
    }

    //String tilts = tempStr + "XtiltY: " + getXtiltY(ax, ay) + "   gx: " + gx;
    //Serial.println(tilts);

    //oldXTilt = currentX;

    unsigned long end = millis();

    if(end-start > 0){
        totalTime += (end-start);
        if(!isSleeping){
          totalBattery += ((end-start) * 18);
        } else {
          totalBattery += ((end-start) * 2);
        }
        loopCounter++;
    }

    String averageTime = tempStr + "Average runtime: " + (totalTime/loopCounter) + "ms";
    //Serial.println(averageTime);

    String averageBattery = tempStr + "Average battery usage: " + ((totalBattery * 3.3)/loopCounter) + "Watts";
    //Serial.println(averageBattery);

    delay(500);

}

void readMPU9150()
{
    //// reads the MPU9150 sensor values. Values are read in order of temperature,
    //// compass, Gyro, Accelerometer

    tm = ( (double) mpu9150.readSensor(mpu9150._addr_motion, MPU9150_TEMP_OUT_L, MPU9150_TEMP_OUT_H) + 12412.0 ) / 340.0;
    cx = mpu9150.readSensor(mpu9150._addr_motion, MPU9150_CMPS_XOUT_L, MPU9150_CMPS_XOUT_H);  //Compass_X
    cy = mpu9150.readSensor(mpu9150._addr_motion, MPU9150_CMPS_YOUT_L, MPU9150_CMPS_YOUT_H);  //Compass_Y
    cz = mpu9150.readSensor(mpu9150._addr_motion, MPU9150_CMPS_ZOUT_L, MPU9150_CMPS_ZOUT_H);  //Compass_Z
    ax = mpu9150.readSensor(mpu9150._addr_motion, MPU9150_ACCEL_XOUT_L, MPU9150_ACCEL_XOUT_H);
    ay = mpu9150.readSensor(mpu9150._addr_motion, MPU9150_ACCEL_YOUT_L, MPU9150_ACCEL_YOUT_H);
    az = mpu9150.readSensor(mpu9150._addr_motion, MPU9150_ACCEL_ZOUT_L, MPU9150_ACCEL_ZOUT_H);
    gx = mpu9150.readSensor(mpu9150._addr_motion, MPU9150_GYRO_XOUT_L, MPU9150_GYRO_XOUT_H);
    gy = mpu9150.readSensor(mpu9150._addr_motion, MPU9150_GYRO_YOUT_L, MPU9150_GYRO_YOUT_H);
    gz = mpu9150.readSensor(mpu9150._addr_motion, MPU9150_GYRO_ZOUT_L, MPU9150_GYRO_ZOUT_H);


}



//// returns accelaration along x-axis, should be 0-1g
float getAccelX(float x)
{
  return x/pow(2,15)*ACCEL_SCALE;
}

//// returns accelaration along z-axis, should be 0-1g
float getAccelY(float y)
{
  return y/pow(2,15)*ACCEL_SCALE;
}

//// returns accelaration along z-axis should be 0-1g
float getAccelZ(float z)
{
  return z/pow(2,15)*ACCEL_SCALE;
}

//// returns the vector sum of the acceleration along x, y and z axes
//// in g units
float getAccelXYZ(float x, float y, float z)
{
  x = getAccelX(x);
  y = getAccelY(y);
  z = getAccelZ(z);

  return sqrt(x*x+y*y+z*z);
}

//// returns tilt along x axis in radians - uses accelerometer
float getXtiltY(float accelX, float accelY)
{
   float tilt = atan2(accelX,accelY)*RAD_TO_DEGREES; //*RAD_TO_DEGREES;
   if(tilt < 0)
   {
      tilt = tilt+360.0;
   }

   return tilt;
}

float getXtiltZ(float accelX, float accelZ)
{
   float tilt = atan2(accelX,accelZ)*RAD_TO_DEGREES; //*RAD_TO_DEGREES;
   if(tilt < 0)
   {
      tilt = tilt+360.0;
   }

   return tilt;
}

//// returns tilt along y-axis in radians
float getYtiltZ(float accelY, float accelZ)
{
   float tilt = atan2(accelY,accelZ)*RAD_TO_DEGREES;
   if(tilt < 0)
   {
     tilt = tilt+360.0;
   }

   return tilt;
}

//// returns tilt along y-axis in radians
float getYtiltX(float accelY, float accelX)
{
   float tilt = atan2(accelY,accelX)*RAD_TO_DEGREES;
   if(tilt < 0)
   {
     tilt = tilt+360.0;
   }

   return tilt;
}


//// returns tilt along y-axis in radians
float getZtiltY(float accelZ, float accelY)
{
   float tilt = atan2(accelZ,accelY)*RAD_TO_DEGREES;
   if(tilt < 0)
   {
     tilt = tilt+360.0;
   }

   return tilt;
}

//// returns tilt along y-axis in radians
float getZtiltX(float accelZ, float accelX)
{
   float tilt = atan2(accelZ,accelX)*RAD_TO_DEGREES;
   if(tilt < 0)
   {
     tilt = tilt+360.0;
   }

   return tilt;
}

void startSleep(){
  sleepTimer.stop();
  isSleeping = true;
  System.sleep(1);
}

void endSleep(){
  WiFi.on();
  WiFi.connect();
  Spark.connect();
  Particle.process();
  delay(500);
  isSleeping = false;
  sleepTimer.reset();
}
