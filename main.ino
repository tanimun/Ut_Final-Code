#include <SoftwareSerial.h> //Library for GPS and lora
#include <TinyGPS++.h> // GPS Data decoding library

static const int RXPin = 9, TXPin = 8; //GPS RX pin and TX pin
static const uint32_t GPSBaud = 9600; // GPS Baud rate

// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);

//Lora pins
#define M1_230 PD6
#define M0_230 PD5
#define RX_230 PD4
#define TX_230 PD3
#define vib A2

const uint32_t myMac = 3344; // Mac number of individual devices. Each device must have different mac number which will also be put into OT code
uint32_t hisMac = 0; // Temporary mac number of second UT whose data needs to be repeated. 

int myADDH = 11, myADDL = 22, mySOS = 0, mySOSOT = 0, myBat = 4; // Own Credentials
int hisADDH = 0, hisADDL = 0, hisSOS = 0, hisSOSOT = 0, hisBat = 0; // Credentials of UT2
int otADDH = 0, otADDL = 0, otChannel = 1; // OT credentials
//23.822764394070912, 90.36423941239507
//23.86120370398947, 90.39967368355967
float myLong = 90.36423941239507, myLati = 23.822764394070912; //Own GPS data
//float myLong=90.39967368355967, myLati=23.86120370398947; 294392678
float hisLong = 0, hisLati = 0; // GPS data of UT2

unsigned long myNst = 0; //Not used for now
unsigned long hisNst = 0; // Not used for now
const unsigned long aliveTimeout = 60000; //Timeout for address reseting (not used for now)
unsigned long aliveTime = 0;
bool ipFlag = false, dataFlag = false, comFlag = false, busyFlag = false, skipFlag = false, sosFlag = 0; //only sosFlag is used

// 192 00 07 00 00 00 98 00 23 3 Default set

// 207 207 194 00 04 18 52 00 97 wireless configuer code

// Main data type which will be passed through lora
typedef union
{
  struct
  {
    uint32_t unix;
    uint32_t mac;
    uint8_t sos;
    uint8_t sosOT;
    uint8_t bat;
    float lon;
    float lati;
    uint8_t addh;
    uint8_t addl;
    float nst;
    uint8_t channel;
  };
  uint8_t data[50];
} Message;
Message msg;


SoftwareSerial lora_230(TX_230, RX_230);

int button = 2, gpsPin = 10;

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(20); //Dont wait more than 20ms for single Serial data 
  pinMode(button, INPUT);
  pinMode(A0, INPUT);
  pinMode(vib, OUTPUT);
  digitalWrite(vib, HIGH);
  pinMode(gpsPin, OUTPUT);

  pinMode(M1_230, OUTPUT);
  pinMode(M0_230, OUTPUT);

  digitalWrite(M1_230, LOW);
  digitalWrite(M0_230, LOW);

  lora_230.begin(9600);
  pinMode(A1, OUTPUT);
  aliveTime = millis();
  
//Hard code of Mac address to address allocation
  
  if (myMac == 1122)
  {
    myADDH = 11;
    myADDL = 22;
  }
  else if (myMac == 2233)
  {
    myADDH = 22;
    myADDL = 33;
  }
  else if (myMac == 3344)
  {
    myADDH = 33;
    myADDL = 44;
  }
  else if (myMac == 4455)
  {
    myADDH = 44;
    myADDL = 55;
  }
  //configure Lora as Fixed point transmission, uniq address, channel and others. 
  setDataSetting();
  
  Serial.println("Device Started");
  Serial.println(String(myADDH) + "  " + String(myADDL));
  Serial.println("Battery is: " + String(analogRead(A0)));
}
bool buttonStatus = false, powerStatus = true; //
unsigned long pressTime = 0, sosPressTime = 2000, powerSwitchTime = 10000; //Timing control of button pressing for SOS and powerDown mode

void loop() 
{
  pressTime = millis(); //Recording current time
  
  while (digitalRead(button) == LOW ) //Intitiate button pressing sequence when a button is pressed
  {
    unsigned long nowTime = millis(); //Record the moment when the button is preseed
    
    if (nowTime - pressTime > sosPressTime && sosFlag == false) //If the button is Pressed tor 'sosPressTime', SOS will be activated.
    {
      Serial.println("SOS Activated");
      digitalWrite(vib, LOW);
      delay(500);
      digitalWrite(vib, HIGH);
      sosFlag = true;
    }
    if (nowTime - pressTime > powerSwitchTime) //If the button is pressed for 'powerSwitchTime' power will be off or on
    {

      Serial.println("Power: " + String(powerStatus));
      powerStatus = !powerStatus;
      pressTime = millis();
      digitalWrite(vib, LOW);
      delay(2000);
      digitalWrite(vib, HIGH);
      if (powerStatus == true)
      {
        digitalWrite(M1_230, LOW);
        digitalWrite(M0_230, LOW);
        digitalWrite(gpsPin, LOW);
      }
      else if (powerStatus == false)
      {
        digitalWrite(M1_230, HIGH);
        digitalWrite(M0_230, HIGH);
        digitalWrite(gpsPin, HIGH);
      }
    }
  }
  
  //if the device if on, the process will be continued
  
  if (powerStatus == true)
  {
    if (Serial.available()) //only for debug the repeater function. Not applicable in real use.
    {
      int a = Serial.parseInt();
      if (a > 0 && a < 9) packetSend(a);
      else if (a == 99)
      {
        skipFlag = true;
        aliveTime = millis();
        Serial.println("Ignoring request");
      }
      else if (a == 11)
      {
        skipFlag = false;
        aliveTime = millis();
        Serial.println("Accepting request");
      }
    }
    int b = parseData();
    if (b != 0)
    {
      //aliveTime=millis();
      Serial.println("Packet Type: " + String(b));
    }
    /*if(b==7 && ipFlag==false)
      {
      Serial.println("I am Here");
      packetSend(8);
      }
      else if(b==6 && msg.mac==myMac && ipFlag==false)
      {
      myADDH=msg.addh;
      myADDL=msg.addl;
      otChannel=msg.channel;
      Serial.println("Mac: " + String(msg.mac));
      Serial.println("ADDH: " + String(myADDH));
      Serial.println("ADDL: " + String(myADDL));
      Serial.println("Channel: " + String(otChannel));
      Serial.println("Address got");
      ipFlag=true;
      packetSend(8);
      delay(500);
      setDataSetting();
      }*/
    if (b == 4 && skipFlag == false)
    {
      Serial.println("Sent gps data as: " + String(myMac));

      msg.mac = myMac;
      msg.sos = sosFlag;
      sosFlag = false;
      msg.bat = analogRead(A0);
      Serial.println("Battery is: " + String(msg.bat));
      msg.lon = myLong;
      msg.lati = myLati;
      packetSend(1);
      delay(250);
      digitalWrite(vib, LOW);
      //Serial.println(msg.lon);
      //Serial.println(msg.lati);
      delay(250);
      digitalWrite(vib, HIGH);
      smartDelay(1000);
      //delay(1000);
    }
    else if (b == 2)
    {
      Serial.println("Sending data request as repeater to:  " + String(msg.mac));
      hisMac = msg.mac;
      if (msg.mac == 1122)
      {
        msg.addh = 11;
        msg.addl = 22;
      }
      else if (msg.mac == 2233)
      {
        msg.addh = 22;
        msg.addl = 33;
      }
      else if (msg.mac == 3344)
      {
        msg.addh = 33;
        msg.addl = 44;
      }
      else if (msg.mac == 4455)
      {
        msg.addh = 44;
        msg.addl = 55;
      }
      msg.mac = myMac;
      packetSend(5);
    }
    else if (b == 5)
    {
      if (msg.mac == 1122)
      {
        msg.addh = 11;
        msg.addl = 22;
      }
      else if (msg.mac == 2233)
      {
        msg.addh = 22;
        msg.addl = 33;
      }
      else if (msg.mac == 3344)
      {
        msg.addh = 33;
        msg.addl = 44;
      }
      else if (msg.mac == 4455)
      {
        msg.addh = 44;
        msg.addl = 55;
      }
      
      Serial.println("Sent gps data");
      msg.mac = myMac;
      msg.sos = sosFlag;
      sosFlag = false;
      msg.bat = analogRead(A0);
      msg.lon = myLong;
      msg.lati = myLati;
      packetSend(1);
      delay(250);
      digitalWrite(vib, LOW);
      delay(250);
      digitalWrite(vib, HIGH);
      smartDelay(1000);
    }
    else if (b == 1)
    {
      Serial.println("Received UT2 Data from: " + String(msg.mac));
      msg.addh = 0;
      msg.addl = 0;
      packetSend(1);
    }
    /*if(millis()-aliveTime>aliveTimeout && ipFlag==true && skipFlag==false)
      {
      aliveTime=millis();
      Serial.println("Going to default settings");
      delay(1000);
      setSearchSetting();
      delay(1000);
      ipFlag=false;
      }
      else if(b==2 && ipFlag==true)
      {
      packetSend(5);
      }
      else if(b==5 && ipFlag==true)
      {
      mySOSOT=msg.sosOT;
      Serial.println("Request from repeater received");
      packetSend(3);
      }
      else if(b==3 && ipFlag==true)
      {
      Serial.println("Got data to repeat. Sending to OT");
      packetSend(1);
      }*/
  }

}

/*char CRC8(const char *data,int length)
  {
   char crc = 0x00;
   char extract;
   char sum;
   for(int i=0;i<length;i++)
   {
      extract = *data;
      for (char tempI = 8; tempI; tempI--)
      {
         sum = (crc ^ extract) & 0x01;
         crc >>= 1;
         if (sum)
            crc ^= 0x8C;
         extract >>= 1;
      }
      data++;
   }
   return crc;
  }
*/

void packetSend(int packetType)
{
  if (1) //not in work, just for folding :D
  {
    //packetType 1 = Send General data UT1 ---> OT
    //               (preamble, mac, sos, battery, Longitude, Latitude, Timestamp)
    //packetType 2 = Request for being repeater OT ---> UT1
    //               (preamble, addH, addH, sosOT, Next_Sleep_Time);
    //packetType 3 = Send repeated data UT2 ---> UT1 ---> OT
    //               (preamble, mac, sos, battery, Longitude, Latitude, timestamp)
    //packetType 4 = Request general data OT ---> UT1
    //               (preamble, sosOT, Next_Sleep_Time);
    //packetType 5 = Request data for repeating UT1 ---> UT2
    //               (preamble, sosOT, Next_Sleep_Time, addh, addl);
    //packetType 6 = IP allocation to unknown UT
    //               (preamble, addh, addl, ch);
  }

  if (packetType == 1) //packetType 1 = Send General data UT1 ---> OT
    //               (preamble, mac, sos, battery, Longitude, Latitude, Timestamp)
  {


    uint8_t buffer[35];
    int preamble = packetType;
    int msgLen = 26;
    memset(buffer, 0, 25);
    buffer[0] = 0xF9;
    buffer[1] = packetType; //preamble
    buffer[2] = msgLen;

    Serial.println("General Data Sent to: " + String(msg.addh) + "   " + String(msg.addl));
    memcpy(&buffer[3], msg.data, msgLen);
    buffer[msgLen + 3] = CRC8(&buffer[1], msgLen + 2);
    Serial.println("Sending data as");
    Serial.println(msg.mac);
    Serial.println(msg.lon);
    Serial.println(msg.lati);
    lora_230.write((byte) msg.addh);
    lora_230.write((byte) msg.addl);
    lora_230.write((byte) otChannel);
    for (int j = 0; j < msgLen + 4; j++) lora_230.write(buffer[j]);
    memset(msg.data, 0, 30);
  }
  else if (packetType == 2) //packetType 2 = Request for being repeater OT ---> UT1
    //               (preamble, addH, addL, sosOT, Next_Sleep_Time, timeStamp);
  {


    uint8_t buffer[35];
    int msgLen = 26;
    memset(buffer, 0, 20);
    buffer[0] = 0xF9;
    buffer[1] = packetType;
    buffer[2] = msgLen;
    msg.unix = 1636881172;
    msg.sosOT = 1;
    msg.addh = 12;
    msg.addl = 34;
    msg.nst = 103279840;
    memcpy(&buffer[3], msg.data, msgLen);
    buffer[msgLen + 3] = CRC8(&buffer[1], msgLen + 2);
    for (int j = 0; j < msgLen + 4; j++) lora_230.write(buffer[j]);
    memset(msg.data, 0, 30);
  }
  else if (packetType == 3) //packetType 3 = Send repeated data UT2 ---> UT1 ---> OT
    //               (preamble, mac, sos, battery, Longitude, Latitude, timestamp)
  {
    uint8_t buffer[35];
    int msgLen = 26;
    memset(buffer, 0, 25);
    buffer[0] = 0xF9;
    buffer[1] = packetType;
    buffer[2] = msgLen;
    int addhBuf = msg.addh;
    int addlBuf = msg.addl;
    msg.sos = mySOS;
    msg.bat = myBat;
    msg.lon = myLong;
    msg.lati = myLati;
    Serial.println("Sending data request as repeater to:  " + String(msg.mac));
    Serial.println(addhBuf);
    Serial.println(addlBuf);
    //Serial.println(y);
    //Serial.println(channelBuf);
    lora_230.write((byte) addhBuf);
    lora_230.write((byte) addlBuf);
    lora_230.write((byte) otChannel);
    memcpy(&buffer[3], msg.data, msgLen);
    buffer[msgLen + 3] = CRC8(&buffer[1], msgLen + 2);
    for (int j = 0; j < msgLen + 4; j++) lora_230.write(buffer[j]);
    memset(msg.data, 0, 30);
  }
  else if (packetType == 4) //packetType 4 = Request general data OT ---> UT1
    //               (preamble, sosOT, Next_Sleep_Time, timeStamp);
  {


    uint8_t buffer[35];
    int msgLen = 26;
    memset(buffer, 0, 20);
    buffer[0] = 0xF9;
    buffer[1] = packetType;
    buffer[2] = msgLen;
    msg.unix = 1636881172;
    msg.sosOT = 1;
    msg.nst = 194466509;
    memcpy(&buffer[3], msg.data, msgLen);
    buffer[msgLen + 3] = CRC8(&buffer[1], msgLen + 2);
    for (int j = 0; j < msgLen + 4; j++) lora_230.write(buffer[j]);
    memset(msg.data, 0, 30);
  }
  else if (packetType == 5) //packetType 5 = Request data for repeating UT1 ---> UT2
    //               (preamble, sosOT, Next_Sleep_Time, addh, addl);
  {


    uint8_t buffer[35];
    int msgLen = 26;
    memset(buffer, 0, 20);
    buffer[0] = 0xF9;
    buffer[1] = packetType;
    buffer[2] = msgLen;

    Serial.println(msg.addh);
    Serial.println(msg.addl);
    //Serial.println(y);
    //Serial.println(channelBuf);
    lora_230.write((byte) msg.addh);
    lora_230.write((byte) msg.addl);
    lora_230.write((byte) otChannel);
    memcpy(&buffer[3], msg.data, msgLen);
    buffer[msgLen + 3] = CRC8(&buffer[1], msgLen + 2);
    for (int j = 0; j < msgLen + 4; j++) lora_230.write(buffer[j]);
    memset(msg.data, 0, 30);
  }
  else if (packetType == 6) //packetType 6 = IP allocation to unknown UT
    //               (preamble, timeStamp, nst, addh, addl, ch, );
  {


    uint8_t buffer[35];
    int msgLen = 26;
    memset(buffer, 0, 20);
    buffer[0] = 0xF9;
    buffer[1] = packetType;
    buffer[2] = msgLen;
    msg.mac = 1943926798;
    msg.unix = 1636881172;
    msg.nst = 194466509;
    msg.addh = 12;
    msg.addl = 22;
    msg.channel = 23;
    memcpy(&buffer[3], msg.data, msgLen);
    buffer[msgLen + 3] = CRC8(&buffer[1], msgLen + 2);
    for (int j = 0; j < msgLen + 4; j++) lora_230.write(buffer[j]);
    memset(msg.data, 0, 30);
  }
  else if (packetType == 7) //packetType 7 = IP allocation request from OT to unknown UT
    //               (preamble, timeStamp, nst, addh, addl, ch, );
  {


    uint8_t buffer[35];
    int msgLen = 26;
    memset(buffer, 0, 20);
    buffer[0] = 0xF9;
    buffer[1] = packetType;
    buffer[2] = msgLen;
    msg.unix = 1636881172;
    memcpy(&buffer[3], msg.data, msgLen);
    buffer[msgLen + 3] = CRC8(&buffer[1], msgLen + 2);
    for (int j = 0; j < msgLen + 4; j++) lora_230.write(buffer[j]);
    memset(msg.data, 0, 30);
  }
  else if (packetType == 8) //packetType 7 = IP allocation request from UT to OT
    //               (preamble, timeStamp, mac );
  {


    uint8_t buffer[35];
    int msgLen = 26;
    memset(buffer, 0, 20);
    buffer[0] = 0xF9;
    buffer[1] = packetType;
    buffer[2] = msgLen;
    msg.unix = 1636881172;
    msg.mac  = myMac;
    memcpy(&buffer[3], msg.data, msgLen);
    buffer[msgLen + 3] = CRC8(&buffer[1], msgLen + 2);
    for (int j = 0; j < msgLen + 4; j++) lora_230.write(buffer[j]);
    memset(msg.data, 0, 30);
  }
}

uint8_t buffer[30];
uint8_t index = 0;
uint8_t length = 0;
bool pack_start = false;

int parseData()
{
  int packType = 0;
  while (lora_230.available()) {
    uint8_t data = lora_230.read();
    Serial.print(data, HEX);
    if (data == 0xF9 && pack_start != true)
    {
      pack_start = true;
      //Serial.println("Packet start");
    }
    else if (pack_start == true)
    {
      //Serial.print("0x");
      //Serial.print(data, HEX);
      //Serial.print(" ");
      buffer[index] = data;
      index++;
      if (index == 1 + 1)
      {
        //Serial.print("Length: ");
        //Serial.println(data);
        length = data;
      }
      else if (index == length + 2 + 1)
      {
        uint8_t crc = data;
        //Serial.print("CRC: ");
        //Serial.println(crc, HEX);
        uint8_t ccrc = CRC8(buffer, length + 2);
        if (crc == ccrc)
        {
          Serial.println("New packet received!");
          if (buffer[0] == 1)
          {
            packType = buffer[0];
            //Serial.println("Packect 1 received");
            memcpy(msg.data, &buffer[2], length);
            //Serial.println(msg.unix);
            //Serial.println(msg.mac);
            //Serial.println(msg.bat);
            //Serial.println(msg.lon,6);
            //Serial.println(msg.lati,6);
            //Serial.println(msg.sos);
          }
          else if (buffer[0] == 2)
          {
            packType = buffer[0];
            //Serial.println("Packect 2 received");
            memcpy(msg.data, &buffer[2], length);
            //Serial.println(msg.unix);
            //Serial.println(msg.addh);
            //Serial.println(msg.addl);
            //Serial.println(msg.nst);
            //Serial.println(msg.sosOT);
          }
          else if (buffer[0] == 3)
          {
            packType = buffer[0];
            //Serial.println("Packect 3 received");
            memcpy(msg.data, &buffer[2], length);
            //Serial.println(msg.unix);
            //Serial.println(msg.mac);
            //Serial.println(msg.bat);
            //Serial.println(msg.lon,6);
            //Serial.println(msg.lati,6);
            //Serial.println(msg.sos);
          }
          else if (buffer[0] == 4)
          {
            packType = buffer[0];
            //Serial.println("Packect 4 received");
            memcpy(msg.data, &buffer[2], length);
            //Serial.println(msg.unix);
            //Serial.println(msg.nst);
            //Serial.println(msg.sosOT);
          }
          else if (buffer[0] == 5)
          {
            packType = buffer[0];
            //Serial.println("Packect 5 received");
            memcpy(msg.data, &buffer[2], length);
            //Serial.println(msg.unix);
            //Serial.println(msg.nst);
            //Serial.println(msg.sosOT);
            //Serial.println(msg.addh);
            //Serial.println(msg.addl);
          }
          else if (buffer[0] == 6)
          {
            packType = buffer[0];
            //Serial.println("Packect 6 received");
            memcpy(msg.data, &buffer[2], length);
            //Serial.println(msg.unix);
            //Serial.println(msg.nst);
            //Serial.println(msg.channel);
            //Serial.println(msg.addh);
            //Serial.println(msg.addl);
          }
          else if (buffer[0] == 7)
          {
            packType = buffer[0];
            //Serial.println("Packect 7 received");
            memcpy(msg.data, &buffer[2], length);
            //Serial.println(msg.unix);
          }
          else if (buffer[0] == 8)
          {
            packType = buffer[0];
            //Serial.println("Packect 8 received");
            memcpy(msg.data, &buffer[2], length);
            //Serial.println(msg.unix);
            //Serial.println(msg.mac);
          }
        }
        memset(buffer, 25, 0);
        index = 0;
        length = 0;
        pack_start = false;
      }
    }
  }
  return packType;
}

void setSearchSetting()
{
  // Serial.println("I am here");
  delay(500);
  digitalWrite(M1_230, HIGH);
  digitalWrite(M0_230, LOW);
  delay(100);
  lora_230.write((byte) 192);
  lora_230.write((byte) 00);
  lora_230.write((byte) 07);
  lora_230.write((byte) otADDH);
  lora_230.write((byte) otADDL);
  lora_230.write((byte) 00);
  lora_230.write((byte) 98);
  lora_230.write((byte) 00);
  lora_230.write((byte) 23);
  lora_230.write((byte) 3); // Transparent Transmission
  delay(100);
  //digitalWrite(M1_230, LOW);
  //digitalWrite(M0_230, LOW);
  //delay(500);
  /*unsigned long nowTime=millis();
    Serial.println("Hi");
    while(millis()-nowTime<5000)
    {
    if(lora_230.available()>0) Serial.println(lora_230.read());
    }
    nowTime=millis();
    digitalWrite(M1_230, LOW);
    digitalWrite(M0_230, LOW);
    nowTime=millis();
    Serial.println("Hlw");
    while(millis()-nowTime<5000)
    {
    //Serial.println("I am here");
    if(lora_230.available()>1) Serial.println(lora_230.read());
    }
    nowTime=millis();*/
  digitalWrite(M1_230, LOW);
  digitalWrite(M0_230, LOW);
  delay(500);
  //lora_230.write((byte) 111);
}

void setDataSetting()
{
  // Serial.println("I am here");
  // delay(500);
  delay(500);
  digitalWrite(M1_230, HIGH);
  digitalWrite(M0_230, LOW);
  delay(100);
  lora_230.write((byte) 192);
  lora_230.write((byte) 00);
  lora_230.write((byte) 07);
  lora_230.write((byte) myADDH);
  lora_230.write((byte) myADDL);
  lora_230.write((byte) 00);
  lora_230.write((byte) 98);
  lora_230.write((byte) 00);//0 for 30dbm, 01 for 27dbm
  lora_230.write((byte) otChannel);
  lora_230.write((byte) 67); //Fixed Transmission
  delay(100);
  digitalWrite(M1_230, LOW);
  digitalWrite(M0_230, LOW);
  delay(500);
  /*unsigned long nowTime=millis();
    Serial.println("Hi");
    while(millis()-nowTime<20)
    {
    if(lora_230.available()>0) Serial.println(lora_230.read());
    }
    nowTime=millis();
    digitalWrite(M1_230, LOW);
    digitalWrite(M0_230, LOW);
    nowTime=millis();
    Serial.println("Hlw");
    while(millis()-nowTime<50)
    {
    if(lora_230.available()>1) Serial.println(lora_230.read());
    }
    nowTime=millis();
    digitalWrite(M1_230, LOW);
    digitalWrite(M0_230, LOW);
    delay(500);*/
  //lora_230.write((byte) 111);
}

const unsigned char r_crctable[256] = { //reversed, 8-bit, poly=0x07
  0x00, 0x91, 0xE3, 0x72, 0x07, 0x96, 0xE4, 0x75,
  0x0E, 0x9F, 0xED, 0x7C, 0x09, 0x98, 0xEA, 0x7B,
  0x1C, 0x8D, 0xFF, 0x6E, 0x1B, 0x8A, 0xF8, 0x69,
  0x12, 0x83, 0xF1, 0x60, 0x15, 0x84, 0xF6, 0x67,
  0x38, 0xA9, 0xDB, 0x4A, 0x3F, 0xAE, 0xDC, 0x4D,
  0x36, 0xA7, 0xD5, 0x44, 0x31, 0xA0, 0xD2, 0x43,
  0x24, 0xB5, 0xC7, 0x56, 0x23, 0xB2, 0xC0, 0x51,
  0x2A, 0xBB, 0xC9, 0x58, 0x2D, 0xBC, 0xCE, 0x5F,
  0x70, 0xE1, 0x93, 0x02, 0x77, 0xE6, 0x94, 0x05,
  0x7E, 0xEF, 0x9D, 0x0C, 0x79, 0xE8, 0x9A, 0x0B,
  0x6C, 0xFD, 0x8F, 0x1E, 0x6B, 0xFA, 0x88, 0x19,
  0x62, 0xF3, 0x81, 0x10, 0x65, 0xF4, 0x86, 0x17,
  0x48, 0xD9, 0xAB, 0x3A, 0x4F, 0xDE, 0xAC, 0x3D,
  0x46, 0xD7, 0xA5, 0x34, 0x41, 0xD0, 0xA2, 0x33,
  0x54, 0xC5, 0xB7, 0x26, 0x53, 0xC2, 0xB0, 0x21,
  0x5A, 0xCB, 0xB9, 0x28, 0x5D, 0xCC, 0xBE, 0x2F,
  0xE0, 0x71, 0x03, 0x92, 0xE7, 0x76, 0x04, 0x95,
  0xEE, 0x7F, 0x0D, 0x9C, 0xE9, 0x78, 0x0A, 0x9B,
  0xFC, 0x6D, 0x1F, 0x8E, 0xFB, 0x6A, 0x18, 0x89,
  0xF2, 0x63, 0x11, 0x80, 0xF5, 0x64, 0x16, 0x87,
  0xD8, 0x49, 0x3B, 0xAA, 0xDF, 0x4E, 0x3C, 0xAD,
  0xD6, 0x47, 0x35, 0xA4, 0xD1, 0x40, 0x32, 0xA3,
  0xC4, 0x55, 0x27, 0xB6, 0xC3, 0x52, 0x20, 0xB1,
  0xCA, 0x5B, 0x29, 0xB8, 0xCD, 0x5C, 0x2E, 0xBF,
  0x90, 0x01, 0x73, 0xE2, 0x97, 0x06, 0x74, 0xE5,
  0x9E, 0x0F, 0x7D, 0xEC, 0x99, 0x08, 0x7A, 0xEB,
  0x8C, 0x1D, 0x6F, 0xFE, 0x8B, 0x1A, 0x68, 0xF9,
  0x82, 0x13, 0x61, 0xF0, 0x85, 0x14, 0x66, 0xF7,
  0xA8, 0x39, 0x4B, 0xDA, 0xAF, 0x3E, 0x4C, 0xDD,
  0xA6, 0x37, 0x45, 0xD4, 0xA1, 0x30, 0x42, 0xD3,
  0xB4, 0x25, 0x57, 0xC6, 0xB3, 0x22, 0x50, 0xC1,
  0xBA, 0x2B, 0x59, 0xC8, 0xBD, 0x2C, 0x5E, 0xCF
};

unsigned char CRC8(const unsigned char *input, int count) {
  unsigned char fcs = 0xFF;
  int i;
  for (i = 0; i < count; i++) {
    fcs = r_crctable[fcs ^ input[i]];
  }
  return (0xFF - fcs);
}

static void smartDelay(unsigned long ms)
{
  lora_230.end();
  Serial.println("Checking GPS");
  delay(100);
  ss.begin(9600);
  unsigned long start = millis();
  do
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
  if (gps.location.isValid())
  {
    myLong = gps.location.lng();
    myLati = gps.location.lat();
  }
  else
  {
    Serial.println("sending dummy data");
    myLong = 00.00;
    myLati = 00.00;
  }
  ss.end();
  delay(100);
  lora_230.begin(9600);
}
