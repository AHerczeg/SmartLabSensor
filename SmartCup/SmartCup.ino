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

int SENSORDELAY = 500;  //// 500; //3000; // milliseconds (runs x1)
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

int POWR1 = A1;
int POWR2 = A2;
int POWR3 = A3;

double POWR1V = 0; //Watts
double POWR2V = 0; //Watts
double POWR3V = 0; //Watts


bool Si1132OK = false;
double Si1132UVIndex = 0; //// UV Index scoring is as follows: 1-2  -> Low, 3-5  -> Moderate, 6-7  -> High, 8-10 -> Very High, 11+  -> Extreme
double Si1132Visible = 0; //// Lux
double Si1132InfraRd = 0; //// Lux

MPU9150 mpu9150;
bool ACCELOK = false;
int cx, cy, cz, ax, ay, az, gx, gy, gz;
float oldZ = 0;
float oldY = 0;

int oldAngle = 0;
double tm; //// Celsius

int maxDegree = 0;

Timer sleepTimer(60000, startSleep);

bool isSleeping = false;

RestClient client = RestClient("sccug-330-04.lancs.ac.uk",8000);

const char* path = "/Cup";

String coreID;

float lPercentage = 100;

bool cupDone = false;

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
    pinMode(POWR1, INPUT);
    pinMode(POWR2, INPUT);
    pinMode(POWR3, INPUT);
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

    System.enableReset();

    coreID = getCoreID();


    sleepTimer.start();
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
    //// powers up sensors
    digitalWrite(I2CEN, HIGH);
    digitalWrite(ALGEN, HIGH);

    //// allows sensors time to warm up
    delay(SENSORDELAY);     //// delay timer

    //// ***********************************************************************

    readMPU9150();          //// reads compass, accelerometer and gyroscope data

    path = "/Cup";

    float currentZ = abs(getZtiltX(az, ax)-180);
    float currentY = abs(getYtiltX(ay, ax)-180);
    String responseString = "";

    if(abs(oldZ - currentZ) > 2){
      Serial.println(oldY);
      Serial.println(currentY);
      sleepTimer.reset();
      if(isSleeping)
        endSleep();
      String tempStr = "";
      String sensorString = tempStr+"{\"CoreID\":\"" + coreID + "\"";
      oldZ = currentZ;
      oldY = currentY;
      if(oldZ > 90)
        oldZ = 90;
      if(oldY > 90)
        oldY = 90;
      if(oldZ < 5)
        oldZ = 0;
      if(oldY < 5)
        oldY = 0;
      if(oldZ >= oldY)
        sensorString = sensorString + ", \"Angle\": " + oldZ;
      else
        sensorString = sensorString + ", \"Angle\": " + oldY;

      getLiquidPercentage();

      sensorString = sensorString + ", \"Liquid\": " + lPercentage;
      sensorString = sensorString + "}";
      Serial.println(sensorString);
      client.post(path, (const char*) sensorString);
      Serial.println(responseString);
      if(lPercentage <= 0 && !cupDone){
        cupDone = true;
        path = "/FullCups";
        sensorString = tempStr+"{\"CoreID\":\"" + coreID + "\"}";
        Serial.println(sensorString);
        client.post(path, (const char*) sensorString);
        Serial.println(responseString);
      }
    }
}


void readMPU9150()
{
    tm = ( (double) mpu9150.readSensor(mpu9150._addr_motion, MPU9150_TEMP_OUT_L, MPU9150_TEMP_OUT_H) + 12412.0 ) / 340.0;
    ax = mpu9150.readSensor(mpu9150._addr_motion, MPU9150_ACCEL_XOUT_L, MPU9150_ACCEL_XOUT_H);
    ay = mpu9150.readSensor(mpu9150._addr_motion, MPU9150_ACCEL_YOUT_L, MPU9150_ACCEL_YOUT_H);
    az = mpu9150.readSensor(mpu9150._addr_motion, MPU9150_ACCEL_ZOUT_L, MPU9150_ACCEL_ZOUT_H);
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
float getZtiltX(float accelZ, float accelX)
{
   float tilt = atan2(accelZ,accelX)*RAD_TO_DEGREES;
   if(tilt < 0)
   {
     tilt = tilt+360.0;
   }

   return tilt;
}

String getCoreID(){
  String coreIdentifier = "";
  char id[12];
  memcpy(id, (char *)ID1, 12);
  char hex_digit;
  for (int i = 0; i < 12; ++i) {
    hex_digit = 48 + (id[i] >> 4);
    if (57 < hex_digit)
      hex_digit += 39;
    coreIdentifier = coreIdentifier + hex_digit;
    hex_digit = 48 + (id[i] & 0xf);
    if (57 < hex_digit)
      hex_digit += 39;
    coreIdentifier = coreIdentifier + hex_digit;
  }
  return coreIdentifier;
}

float volume (float r, float L, float h){

  float c = 2 * pow(h*(2*r - h),0.5); /** chord length **/
  float l = PI /90 * r * 180/ PI * acos(1 - h/r); /** arc length **/
  float a = 0.5 * (r * l - c * (r - h)); /** liquid cross sectional area **/
  float volume = a * L;
  return volume;
}


void startSleep(){
  sleepTimer.stop();
  String nameString = coreID + " Sleep start";
  Particle.publish("photonSensorData",nameString, PRIVATE);
  isSleeping = true;
  System.sleep(1);
}

void endSleep(){
  WiFi.on();
  WiFi.connect();
  Spark.connect();
  Particle.process();
  delay(500);
  String nameString = coreID + " Sleep end";
  Particle.publish("photonSensorData",nameString, PRIVATE);
  isSleeping = false;
  sleepTimer.reset();
}

float getLiquidPercentage(){
  if(abs(getYtiltX(ay, ax)-180) > maxDegree || abs(getZtiltX(az, ax)-180) > maxDegree){
    if(abs(getZtiltX(az, ax)-180) >= abs(getYtiltX(ay, ax)-180)){
      maxDegree = abs(getZtiltX(az, ax)-180);
    } else {
      maxDegree = abs(getYtiltX(ay, ax)-180);
    }

    if(maxDegree >= 90){
      maxDegree = 90;
      lPercentage = 0;
    } else {
      int n = 100;
      float r = 5; /** radius **/
      float L = 15; /** tank length **/
      float h = L * sin((90 - maxDegree) *  PI / 180); /** liquid level **/
      float slope = maxDegree *  PI / 180; /** tank slope in radians**/
      float Vt = PI * pow(r,2) * L; /** total volume in the tank **/
      float h0 = h / cos(slope);
      float dh = (L/n) * tan(slope);
      float v = 0;
      for (int i = 0;i < n;i++){
        if (h0 > 0) {
          float he = h0;
          if (h0 > r * 2)
            he = r * 2;
          float ve = volume(r, L/n, he);
          v += ve;
        }
        h0 = h0 - dh;
      }
      lPercentage = round((v/Vt)*100);
    }
  }
}
