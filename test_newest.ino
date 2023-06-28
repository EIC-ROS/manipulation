#include <SPI.h>
#include <Ethernet.h>
#include <ezButton.h>

//define HSPI Channel and Motor Driver Pin
#define CLK 18
#define SS1 5 //W5500
#define DirPin 17
#define StepPin 16
#define ENC_IN_A 34 // pulse
#define ENC_IN_B 35 // direction
#define MISO 19
#define MOSI 23

//36V 4A
boolean Direction = true; 
volatile long wheel_pulse_count = 0;
float pitch = 0.4; //in cm
int step = 0;
float currentposition = 0;
float previousposition = 0;
float encPos = 0;
unsigned long t = 0;
int topState = 1; //state of top limit switch , 0 is pressed
int bottomState = 1; //state of bottom limit switch , 0 is pressed
int i = 0;
bool ishittingLimitswitch = false;
float receivedData;
float previousData;
int dataSize;

//----you can adjust these parameters-----//

const uint16_t StepPeriodUs = 800; //tlarger number provide less speed but more torque is applied
int maximumHeight = 17; // maximum height of lift in cm
int minimumHeight = 0; // manimum height of life in cm
int rebouncestep = 400; //amount of step(opposite direction) when lift is hitting limitswitch
int rev = 400; //step per round 400 step/rev for 1/2 microstep , 800step for 1/4 microstep  , more resolution means less torque , more precise.

//tested value
//for 1/2 microstep , StepPeriodUs = 800-1200 
//for 1 microstep, StepPeriodUs = 2500

const float pulse_to_cm = pitch/rev;

//---------Ethernet----------//
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED }; 
// MAC address of your W5500 module
// EthernetServer sendServer(8080);
EthernetServer receiveServer(9090);
byte Enet = 1;
byte ip[] = {192,168,1,101};

//--------limit switch--------//
ezButton topSwitch(0);  // create ezButton object that attach to pin 0;
ezButton bottomSwitch(4);  // create ezButton object that attach to pin 4;


void printNumber(int number, int period) {  // this function can be used to print integer instead of Serial.print()
  static unsigned long previousTime = 0;
  unsigned long currentTime = millis();
  // Print the message every "period" miliseconds
  if (currentTime - previousTime >= period) {
    Serial.println(number);
    previousTime = currentTime;
  }
}

void printMessage(const char* message, int period) { // this function can be used to print message instead of Serial.print()
  static unsigned long previousTime = 0;
  unsigned long currentTime = millis();
  // Print the message every "period" miliseconds
  if (currentTime - previousTime >= period) {
    Serial.println(message);
    previousTime = currentTime;
  }
}

void resetPosition() // this function will move lift down untill it hit bottom limit swtich
{
  while (true) {
    bottomSwitch.loop();
    int bottomState = bottomSwitch.getState();
    printNumber(bottomState, 1000);
    movedown();
    if (bottomState == 0){
      break;
    }
  }
  step = rebouncestep;
  while (step != 0){
    moveup();
  }
  step = 0;
}
void wheel_pulse() // this function will count pulse generated from encoder
{
  // Read the value for the encoder for the right wheel
  int val = digitalRead(ENC_IN_B);
  if (val == LOW) {
    Direction = false; // Reverse
  }
  else {
    Direction = true; // Forward
  }

  if (Direction) {
    wheel_pulse_count++;
  }
  else {
    wheel_pulse_count--;
  }
  //printNumber(wheel_pulse_count,100);
}


void setDirection(bool dir) //this function set direaction for stepper motor
{
  // The STEP pin must not change for at least 200 nanoseconds before and after
  // changing the DIR pin.
  delayMicroseconds(10);
  digitalWrite(DirPin, dir);
  delayMicroseconds(10);
}

void moveup()// this function will moveup the lift
{
  setDirection(1);
  printMessage("UP",2000);
  stepMotor();
  step--;
}

void movedown()// this function will movedown the lift
{
  setDirection(0);
  printMessage("DOWN",2000);
  stepMotor();
  step++;
}

bool checkNoise(float previousData,float currentData) // this function will filter out the unappropiate noise from socket by compare current data and previous data
{
  int score = 0;
  int n = 3;
  for (int j = 0; j<n; j++){
    if (previousData == currentData){
      score++;
    }
    //delay(100);
  }
  if(score == n){ // if current data matched previous data n times
    return true;
  }
  else
  {
    return false;
  }
}

void computeStep() 
{
  currentposition = receivedData;
  Serial.print("currentPosition :");
  Serial.println(currentposition);
  Serial.print("previousPosition :");
  Serial.println(previousposition);
  step = ((currentposition - previousposition)/pitch)*rev;
  previousposition = currentposition;
}

// Custom implementation of ntohf
float ntohf_custom(float value) {
  uint32_t temp = ntohl_custom(*((uint32_t*)&value));
  return *((float*)&temp);
}

// Custom implementation of htonf
float htonf_custom(float value) {
  return ntohf_custom(value);
}

// Custom implementation of ntohl
uint32_t ntohl_custom(uint32_t value) {
  return ((value & 0xFF) << 24) | ((value & 0xFF00) << 8) | ((value >> 8) & 0xFF00) | ((value >> 24) & 0xFF);
}

// Custom implementation of htonl
uint32_t htonl_custom(uint32_t value) {
  return ntohl_custom(value);
}

void Ethernetconnect() //For Ethenet connection
{
 // using for check client status
  EthernetClient receiveClient = receiveServer.available(); //receiveClient already boolean
  // EthernetClient sendClient = sendServer.available();
  // send and receive
  if (receiveClient.connected()){
    dataSize = receiveClient.read((uint8_t*)&receivedData, sizeof(receivedData));
    // Convert endianness of received data
    receivedData = ntohf_custom(receivedData); 
    if (checkNoise(previousData, receivedData) == true){
      if (ishittingLimitswitch == false)
      {
       computeStep();
      }
      else
      {
        if (currentposition == maximumHeight && receivedData <= maximumHeight)
        {
          previousposition = maximumHeight;
          computeStep();
          ishittingLimitswitch = false;
        }
        else if (currentposition == minimumHeight && receivedData >= minimumHeight)
        {
          previousposition = minimumHeight;
          computeStep();
          ishittingLimitswitch = false;
        }
      }
    }
    previousData = receivedData;
    Serial.print("currentposition : ");
    Serial.println(currentposition);
    // Serial print received data
    Serial.print("Received data: ");
    Serial.println(receivedData, 2);

    //Send a float to the client
    // float responseValue = 10.00;  // Modify to send a single float
    // responseValue = htonf_custom(responseValue);
    // receiveClient.write((uint8_t*)&responseValue, sizeof(responseValue));
    // //End of receiving data
    // // read data
    // float read_Data = encPos;// Modify to send a single float
    // // send data from esp32
    // // serial print for check
    // Serial.print("Send data: ");
    // Serial.println(read_Data);
    // // sending from client to server
    // sendClient.write((byte*)&read_Data, sizeof(read_Data));
    delay(49);
  }
  //End
}

void drivemotor() 
{ //For driving motor
  while(step != 0) //rotate motor untill move into given distance
  {
    //Serial.println(step);
    // push switch = 1 , not push = 0
    topSwitch.loop();
    bottomSwitch.loop();
    int topState = topSwitch.getState();
    int bottomState = bottomSwitch.getState();
    // Serial.print("topState ");
    // Serial.println(topState);
    // Serial.print("bottomState ");
    // Serial.println(bottomState);
    //Dir 0 is down (A Green-Black, B Red-Blue)
    if (step > 0 && topState == 1)
    {
        //Serial.println("MOVE");
      moveup();
    }
    else if (step < 0 && bottomState == 1)
    { 
      movedown();
    }
    else
    {
      if (topState == 0) // hitting top limit switch
      {
        step = -rebouncestep;
        while (step != 0)
        {
          movedown();
        }
        currentposition = 25;
        ishittingLimitswitch = true;
        
      }
      if (bottomState == 0) // hitting bottom limit switch
      {
        step = rebouncestep;
        while (step != 0)
        {
          moveup();
        }
         currentposition = 0;
         ishittingLimitswitch = true;
      }
    }
    // Encoder
    //encPos = pulse_to_cm * wheel_pulse_count; //multiply 0.4 if using 1/2 microstep , 0.8 for 1/4 microstep
    //printNumber(encPos,1000);
  }
}

void stepMotor()
{
  digitalWrite(StepPin, HIGH);
  delayMicroseconds(StepPeriodUs);
  digitalWrite(StepPin, LOW);
  delayMicroseconds(StepPeriodUs);
}
void setup() {
  //-----Ethernet setup-----//
  Ethernet.init(5);
  // set serial begin
  Serial.begin(115200);
  while (!Serial){
    // wait for serial port connect
  }
  Serial.println("Ethernet");
  // if (Ethernet.begin(mac) == 0) {
  //   Serial.println("Failed to configure Ethernet using DHCP");
  //   Enet = 0;
  // }
  Ethernet.begin(mac,ip);
  delay(1000);

  if (Enet == 1) {
    Serial.print("Ethernet IP address: ");
    Serial.println(Ethernet.localIP());
  }
  else {
    Serial.println("Unable to connect to Ethernet");
  }

  // sendServer.begin();
  receiveServer.begin();
  // finish setting client

  //------pinmode setting-----------//
  pinMode(ENC_IN_A, INPUT_PULLUP);
  pinMode(ENC_IN_B, INPUT);
  pinMode(StepPin, OUTPUT);
  digitalWrite(StepPin, LOW);
  pinMode(DirPin, OUTPUT);

  //setiing SPI protocol
  SPI.begin();

  //----limit swtich-----/
  topSwitch.setDebounceTime(50); // set debounce time to 50 milliseconds
  bottomSwitch.setDebounceTime(50); // set debounce time to 50 milliseconds
  //attachInterrupt(digitalPinToInterrupt(ENC_IN_A), wheel_pulse, RISING);

  t = millis(); //start timing
  resetPosition();
  delay(3000);
  wheel_pulse_count = 0; //reset encoder (just in case)
    //encoder
  
}

void loop() {
  Ethernetconnect();
  drivemotor();
}