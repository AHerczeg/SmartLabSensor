//String connection = "192.168.0.100:8899"
UDP Udp;
int IPs[4];

void setup(){
  //Register Remote Function
  Particle.function("changeLight", changeLight);
  //Inishalised UDP with local port
  Udp.begin(8899);
  IPs[0] = 123;
  IPs[1] = 102;
  IPs[2] = 101;
  IPs[3] = 121;
}
void loop(void){
  //Do nothing here...
}

int changeLight(String command){
  //Convert Input String in formate of "int#int#int#int" to 3 bytes followed by the blub ID
  int i = command.indexOf('#');
  String value = command.substring(0,i);
  char char1 = char(value.toInt());
  int j = command.indexOf('#',i + 1);
  value = command.substring(i + 1,j);
  char char2 = char(value.toInt());
  i = command.indexOf('#', j + 1);
  value = command.substring(j + 1, i);
  char char3 = char(value.toInt());
  value = command.substring(i + 1, command.length());
  int bulbID = value.toInt();
  //form message
  char message[3];
  message[0] = char1;
  message[1] = char2;
  message[2] = char3;
  //Send message and return result
  return Udp.sendPacket(message, 3, IPAddress(192,168,0, IPs[bulbID] ), 8899);
}
