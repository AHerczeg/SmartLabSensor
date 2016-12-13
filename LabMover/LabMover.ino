#include "MPU9150.h"
#include "Si1132.h"
#include "Si70xx.h"
#include "math.h"
#include "rest_client.h"


//// ***************************************************************************
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
double THRESHOLD = 0.5; //Threshold for temperature and humidity changes
int THRES_LS = 5; // Threshold for light and sound changes

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

double oldTmp = 0;
double oldHmd = 0;
double oldVisible = 0;
double oldSound = 0;
int oldPos = 0;

bool Si7020OK = false;
double Si7020Temperature = 0; //// Celsius
double Si7020Humidity = 0;    //// %Relative Humidity

bool Si1132OK = false;
double Si1132UVIndex = 0; //// UV Index scoring is as follows: 1-2  -> Low,
                          //// 3-5  -> Moderate, 6-7  -> High,
                          //// 8-10 -> Very High, 11+  -> Extreme
double Si1132Visible = 0; //// Lux
double Si1132InfraRed = 0; //// Lux

float strength = 0;
float old_strength = 0;
float alpha = 0.09;

MPU9150 mpu9150;
bool ACCELOK = false;
int cx, cy, cz, ax, ay, az, gx, gy, gz;
double tm; //// Celsius
int limit_3 = 50;
int limit_2 = 45;

Si1132 si1132 = Si1132();


RestClient client = RestClient("sccug-330-04.lancs.ac.uk",8000);

const char* path = "/LocTracking";

unsigned long totalTime = 0;

int totalBattery = 0;

int loopCounter = 0;

Timer sleepTimer(60000, startSleep);

bool isSleeping = false;

//// ***************************************************************************


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
    Particle.subscribe("wifiZone3", updateLimit3);
    Particle.subscribe("wifiZone2", updateLimit2);
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

    //// powers up sensors
    digitalWrite(I2CEN, HIGH);
    digitalWrite(ALGEN, HIGH);

    //// allows sensors time to warm up
    delay(SENSORDELAY);

    //// ***********************************************************************

    unsigned long start = millis();

    String tempStr = "";
    String sensorString = tempStr+"{\"CoreID\":\"" + getCoreID() + "\"";
    bool change = false;

    int reading = -(WiFi.RSSI()); //RSSI value is negative, this makes comparisons more readable
    int f = 0;

    if(reading > 0) // WiFi.RSSI returns positive values as error codes, they need to be ignored
    {
      if(old_strength > 0)
        strength = alpha * reading + (1 - alpha)* old_strength; // Simple low pass filter
      else
        strength = reading;
    }

    f = round(strength); // Floats comparison are unreliable, so we use an int

    if(f > 0)
    {
      old_strength = strength; //  If the value is usable, replace old_strength
      int pos = 0;
      if(f < limit_2)
        pos=1;
      else if((limit_3 >= f) && (f >= limit_2))
        pos = 2;
      else if(f > limit_3)
        pos = 3;

      if(pos > 0 && oldPos != pos) // Avoid creating messages unless position has changed
      {
        sensorString = sensorString + ", \"RoomID\":" + pos;
        oldPos = pos;
        change = true;
      }
    }
    sensorString = sensorString + "}";

    //Serial.println(sensorString);
    String responseString = "";

    if(change)
      client.post(path, (const char*) sensorString, &responseString);

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
    Serial.println(averageTime);

    String averageBattery = tempStr + "Average battery usage: " + ((totalBattery * 3.3)/loopCounter) + "Watts";
    Serial.println(averageBattery);

    sleepTimer.reset();
}

void updateLimit3(const char *event, const char *data)
{
  String s = String(data);
  limit_3 = -(s.toInt());
  //limit_2 = limit_3 - 5;
}

void updateLimit2(const char *event, const char *data)
{
  String s = String(data);
  limit_2 = s.toInt();
  //limit_3 = limit_2 + 5; // Limit 2 and 3 are 5 units apart
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
