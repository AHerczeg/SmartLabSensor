
  void setup()
  {
    // Open serial over USB
    Serial.begin(9600);
    Serial.println("Testing...");

    //Subscribes to sensorData
    Particle.subscribe("photonSensorData", printPhotonSensorData, MY_DEVICES);
  }

  void loop(void)
  {
    int x = WiFi.RSSI();
    Particle.publish("wifiZone2", (String) x);
    delay(5000);
  }

  //when sensor is received, photon relays it to visualiser through Serial print statement
  void printPhotonSensorData(const char *event, const char *data)
  {
      String dataString = String(data);
      Serial.println(dataString);
  }
