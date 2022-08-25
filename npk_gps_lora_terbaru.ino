/* Define pin RS485 */
  #define npkRX 15//4 //RO
  #define npkTX 4 //DI
  #define RE    13//5
  #define DE    5//26
/* Define pin gps NEO-6M */
  #define gpsRx 26//13
  #define gpsTx 25//15
/* Define Serial Structure */
  #define serialBaud    115200
  #define npkBaud     4800
  #define gpsBaud     9600
  #include <lorawan.h>
  #include <TinyGPS.h> 

  TinyGPS gps;
  
  byte request[7][8] = {{0x01,0x03,0x00,0x00,0x00,0x01,0x84,0x0A}, // Hum             01  03  00  00  00  01  84  0A
                        {0x01,0x03,0x00,0x01,0x00,0x01,0xD5,0xCA}, // temp            01  03  00  01  00  01  D5  CA 
                        {0x01,0x03,0x00,0x02,0x00,0x01,0x25,0xCA}, // Ec              01  03  00  02  00  01  25  CA
                        {0x01,0x03,0x00,0x03,0x00,0x01,0x74,0x0A}, // pH              01  03  00  03  00  01  74  0A
                        {0x01,0x03,0x00,0x04,0x00,0x01,0xC5,0xCB}, // nitrogen        01  03  00  04  00  01  C5  CB
                        {0x01,0x03,0x00,0x05,0x00,0x01,0x94,0x0B}, // fosfor          01  03  00  05  00  01  94  0B 
                        {0x01,0x03,0x00,0x06,0x00,0x01,0x64,0x0B}};// potasium/kalium 01  03  00  06  00  01  64  0B

  float nilaiNPK[] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
  String nilaiStringNPK[8];
  String variable[] = {"hum","temp","Ec","pH","N","P","K"};
  float constant[]  = {0.1,0.1,1,0.1,1,1,1};

  const char *devAddr = "d2369f80";//a72b6baa,, b8888a57
  const char *nwkSKey = "4d5acfcf99b8314a0000000000000000";//ccb392823def4d6d:6f06abccd5a3cc52
  const char *appSKey = "0000000000000000032d8937486174f0";
  
  byte buffer_array[7];
/* Define Looping Variables */
  unsigned long       lastMillis = 0;
  unsigned long       sekarang   = 0;
  
const unsigned int requestSize= 8;
const unsigned long interval = 10000;    // 10 s interval to send message
const unsigned long intervalByteNPK = 750;
unsigned long previousMillis = 0;  // will store last time message sent
unsigned int counter = 0;     // message counter
String dataSend = "";
char latChar[10];
char lonChar[10];
char myStr[100];
byte outStr[255];
byte recvStatus = 0;
int port, channel, freq;
bool newmessage = false;
float flat, flon;
unsigned long age;
const sRFM_pins RFM_pins = {
  .CS = 2,
  .RST = 32,
  .DIO0 = 14,
  .DIO1 = 12,
};
/* Object and Library Setup */
  #include <SoftwareSerial.h>
  SoftwareSerial npk(npkRX, npkTX);
  SoftwareSerial gpsSS(gpsTx, gpsRx);

boolean test=0;
unsigned int peringatan=0;
void setup(){
  /* Setup Serial Communication */
    Serial.begin(115200);
    gpsSS.begin(gpsBaud);
    npk.begin(npkBaud);
  /* First Request Address */
    if (!lora.init()) {
      Serial.println("RFM95 not detected");
      delay(5000);
      //return;
    }
      // Set LoRaWAN Class change CLASS_A or CLASS_C
    lora.setDeviceClass(CLASS_C);

    // Set Data Rate
    lora.setDataRate(SF10BW125);

    // Set FramePort Tx
    lora.setFramePortTx(5);

    // set channel to random
    lora.setChannel(MULTI);

    // Put ABP Key and DevAddress here
    lora.setNwkSKey(nwkSKey);
    lora.setAppSKey(appSKey);
    lora.setDevAddr(devAddr);
    Serial.println("Setting Up ...");
    pinMode(RE, OUTPUT);
    pinMode(DE, OUTPUT);
    digitalWrite(DE,HIGH);
    digitalWrite(RE,HIGH);
    delay(interval);
    
}

void loop(){
  /* Request every n interval miliseconds */
    sekarang = millis();
    if (sekarang - lastMillis > interval) {
      Serial.println("");
      for(counter =0;counter<=7;counter++){
        digitalWrite(DE,HIGH);
        digitalWrite(RE,HIGH);
        delay(10);
        if(npk.write(request[counter],requestSize) == requestSize){
          digitalWrite(DE,LOW);
          digitalWrite(RE,LOW);
          if(npk.available()){
            Serial.print(variable[counter-1]);
            Serial.print(" : ");
            for (int i = 0; i < 7; i++) {
              buffer_array[i] = npk.read();
              Serial.print("0x");
              Serial.print(buffer_array[i],HEX);
              Serial.print(" ");
            }
            nilaiStringNPK[counter-1]=ByteToString(buffer_array);
            nilaiNPK[counter-1]=float(toNumber(buffer_array[3],buffer_array[4]))*constant[counter-1];
            Serial.print(" | ");
            Serial.println(nilaiNPK[counter-1]);
          }
        }
        delay(intervalByteNPK);
      }

      panggilGps();
      Serial.print("lat : ");
      Serial.println(flat,6);
      Serial.print("lon : ");
      Serial.println(flon,6);
    
      dataSend = FloatToHex(flat)+","+FloatToHex(flon)+","+nilaiStringNPK[0]
                  +nilaiStringNPK[1]+nilaiStringNPK[2]+nilaiStringNPK[3]
                  +nilaiStringNPK[4]+nilaiStringNPK[5]+nilaiStringNPK[6];
      
      dataSend.toCharArray(myStr,50);
      Serial.print("Sending: ");
      Serial.println(dataSend);
     
      lora.sendUplink(myStr, strlen(myStr), 0);
      
      port = lora.getFramePortTx();
      channel = lora.getChannel();
      freq = lora.getChannelFreq(channel);
      Serial.print(F("fport: "));    Serial.print(port);Serial.print(" ");
      Serial.print(F("Ch: "));    Serial.print(channel);Serial.print(" ");
      Serial.print(F("Freq: "));    Serial.print(freq);Serial.println(" ");
      test=0;
      lastMillis = millis();
      lora.update();
    }
    if(test==0){
      Serial.print("\nAkan mengirim dalam ");
      test=1;
    }
    if(test==1&&((millis()-peringatan)>=1000)){
      peringatan=millis();
      Serial.print(int((interval-(sekarang-lastMillis))/1000));
      Serial.print("... ");
    } 
    
}
int toNumber(byte larger, byte smaller){
  int left = larger<<8;
  int right = smaller;
  return left+right;
}

String FloatToHex(float data){
  byte hex[4];
  byte* f_byte = reinterpret_cast<byte*>(&data);
  memcpy(hex, f_byte, 4);
  char hexa[10];
  sprintf(hexa,"%02X%02X%02X%02X",hex[3],hex[2],hex[1],hex[0]);
  Serial.println(hexa);
  return (String)hexa;
}

String ByteToString(byte* data){
  char hexa[6];
  sprintf(hexa,"%02X%02X",data[3],data[4]);
  return (String)hexa;
}

void panggilGps(){
  bool newData = false;
  unsigned long chars;
  unsigned short sentences, failed;

  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    while (gpsSS.available())
    {
      char c = gpsSS.read();
      // Serial.write(c); // hilangkan koment jika mau melihat data yang dikirim dari modul GPS
      if (gps.encode(c)) // Parsing semua data
        newData = true;
    }
  }

  if (newData)
  {
    gps.f_get_position(&flat, &flon, &age);
    Serial.print("LAT=");
    Serial.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
    Serial.print(" LON=");
    Serial.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
    Serial.print(" SAT=");
    Serial.print(gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites());
    Serial.print(" PREC=");
    Serial.print(gps.hdop() == TinyGPS::GPS_INVALID_HDOP ? 0 : gps.hdop());
    newData=false;
  }
  
  gps.stats(&chars, &sentences, &failed);
  Serial.print(" CHARS=");
  Serial.print(chars);
  Serial.print(" SENTENCES=");
  Serial.print(sentences);
  Serial.print(" CSUM ERR=");
  Serial.println(failed);
  if (chars == 0)
    Serial.println("** Tidak ada Data Masuk, Periksa Wiring **");
}
