
  void setup()
  {
    // Open serial over USB
    Serial.begin(9600);

    //Subscribes to sensorData
    Particle.subscribe("Team3Motion", printPhotonSensorData);
  }

  void loop(void)
  {
    //do nothing
  }

  //when sensor is received, photon relays it to visualiser through Serial print statement
  void printPhotonSensorData(const char *event, const char *data)
  {
      String dataString = String(data);
      Serial.println(dataString);
  }
