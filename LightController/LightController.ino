//String connection = "192.168.0.100:8899"
UDP Udp;

void setup(){
  //Register Remote Function
  Particle.function("changeLight", changeLight);
  //Inishalised UDP with local port
  Udp.begin(8899);

}
void loop(void){
  //Do nothing here...
}

int changeLight(String command){
  //Convert Input String in formate of "int#int#int" to 3 bytes
  int i = command.indexOf('#');
  String value = command.substring(0,i);
  char char1 = char(value.toInt());
  int j = command.indexOf('#',i + 1);
  value = command.substring(i + 1,j);
  char char2 = char(value.toInt());
  value = command.substring(j + 1,command.length());
  char char3 = char(value.toInt());
  //form message
  char message[3];
  message[0] = char1;
  message[1] = char2;
  message[2] = char3;
  //Send message and return result
  return Udp.sendPacket(message, 3, IPAddress(192,168,0,100), 8899);
}
