
  int lastStrength;
  void setup()
  {
    // Open serial over USB
    Serial.begin(9600);
    Serial.println("Testing...");

    //Subscribes to sensorData
    Particle.subscribe("photonSensorData", printPhotonSensorData, MY_DEVICES);

    lastStrength = WiFi.RSSI();
  }

  void loop(void)
  {
    int x = WiFi.RSSI();

    if(x != lastStrength){
      Particle.publish("wifiZone3", (String) x);
      lastStrength = x;
    }
    delay(1000);
    Particle.publish("wifiZone3", (String) x);
    delay(5000);

  }

  //when sensor is received, photon relays it to visualiser through Serial print statement
  void printPhotonSensorData(const char *event, const char *data)
  {
      String dataString = String(data);
      Serial.println(dataString);
  }
