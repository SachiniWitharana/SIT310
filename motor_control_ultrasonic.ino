
const int echoPinRightSensor = 2;
const int trigPinRightSensor = 4;

const int echoPinFrontSensor = 11;
const int trigPinFrontSensor = 12;

const int echoPinLeftSensor = 5;
const int trigPinLeftSensor = 6;

long time_duration;
int distance_between;

int zumomotorright = 9;                                                              
int zumomotorrightdir  = 7;
int zumomotorleft = 10;
int zumomotorleftdir  = 8;

const byte num = 11;
char receivedChars[num]; 
String received; 
boolean newDataRec = false;

void setup() {
  pinMode(zumomotorright, OUTPUT);                                                      
  pinMode(zumomotorleft, OUTPUT);     
  pinMode(zumomotorrightdir, OUTPUT);  
  pinMode(zumomotorleftdir, OUTPUT);  

  pinMode(trigPinLeftSensor, OUTPUT);
  pinMode(echoPinLeftSensor, INPUT); 
  pinMode(trigPinFrontSensor, OUTPUT); 
  pinMode(echoPinFrontSensor, INPUT); 
  pinMode(trigPinRightSensor, OUTPUT); 
  pinMode(echoPinRightSensor, INPUT); 
  Serial.begin(9600);
}

void loop() 
{
  int left_sensor = ultrasonic(echoPinLeftSensor,trigPinLeftSensor);
  int front_sensor = ultrasonic(echoPinFrontSensor,trigPinFrontSensor);
  int right_sensor = ultrasonic(echoPinRightSensor,trigPinRightSensor);
  String message_sensor = "";
  message_sensor = message_sensor + "[" + left_sensor + "cm," + front_sensor + "cm," + right_sensor + "cm]\n";
  Serial.print(message_sensor);
  delay(100); 
  recdWithEndMarker();
  processRobot();              
}

int ultrasonic(int echoPin, int trigPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  time_duration = pulseIn(echoPin, HIGH);

  distance_between= time_duration*0.034/2;
  return distance_between;
}

void processRobot() {
 if (newDataRec == true)
 {
  String instruction_received = received.substring(0,5);
  String dataRec = received.substring(6,20);
  if(instruction_received == "MOVEF") forward(dataRec.toInt());
  if(instruction_received == "MOVEB") backward(dataRec.toInt());
  if(instruction_received == "TURNL") tleft(dataRec.toInt());
  if(instruction_received == "TURNR") tright(dataRec.toInt());
  if(instruction_received == "STOP") stop();
  newDataRec = false;
 }
}

void recdWithEndMarker() 
{
 static byte nndx = 0;
 char endMarker = '\n';
 char rc;
 
 while (Serial.available() > 0 && newDataRec == false) 
 {
  rc = Serial.read();

  if (rc != endMarker) 
  {
    receivedChars[nndx] = rc;
    nndx++;
    if (nndx >= num) 
    {
      nndx = num - 1;
    }
 }
 else 
  {
  receivedChars[ndx] = '\0'; // terminate the string
  received = String(receivedChars);
  ndx = 0;
  newDataRec = true;
  }
 }
}


void forward(int time)
{
  digitalWrite(zumomotorrightdir, LOW);
  analogWrite(zumomotorright,100); 
  digitalWrite(zumomotorleftdir, LOW);
  analogWrite(zumomotorleft, 100); 
}

void backward(int time)
{
  digitalWrite(zumomotorrightdir, HIGH);
  analogWrite(zumomotorright,180); 
  digitalWrite(zumomotorleftdir, HIGH);
  analogWrite(zumomotorleft, 180);
  delay(time);
  stop();
}

void tleft(int time)
{
  digitalWrite(zumomotorrightdir, LOW);
  analogWrite(zumomotorright,90);
  digitalWrite(zumomotorleftdir, HIGH);
  analogWrite(zumomotorleft, 90);
  delay(time);
  stop();
}

void tright(int time)
{
  digitalWrite(zumomotorrightdir, HIGH);
  analogWrite(zumomotorright, 90); 
  digitalWrite(zumomotorleftdir, LOW);
  analogWrite(zumomotorleft, 90);
  delay(time);
  stop();
}

void stop()
{
analogWrite(zumomotorright, 0); 
analogWrite(zumomotorleft, 0);   
}
