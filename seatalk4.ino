#include <Alt9SoftSerial.h>
#include <SoftwareSerial9.h>

#define SEATALK_RX 8
#define SEATALK_TX 9
Alt9SoftSerial SerialI(SEATALK_RX, NULL);
SoftwareSerial9 SerialO(NULL, SEATALK_TX);


bool printNaNST = false;
bool printKnownST = false;
bool printNmeaToUsb = true; // send nmea on usb
uint8_t windAnglePin = A0;
float stDepthOffset = 2.2; // from surface off the wather [meatars]


bool windBurst = false;
bool stMesKnown = false;
uint8_t st[32];
int ind, windAngle, windAngleLast;
char windAvg;
float windAngleAvg;
String usartMsg;
char cnt = 0;

float stDepth, stSog;
int stHeading, apCourse, stCog;
String stLat, stLon, stMLat, stMLon;
char stNS, stEW;

int printangle, inbyte ;
unsigned long currentMillis, oldMillis;

int apRudder = 0;
char apStatus = 0;
// 0 - standby 2 - auto 4 - vane 8 - track

/*
  //RMB, APB, BWR, BWC, XTE
  //85 X6 XX VU ZW ZZ YF 00 yf
  //$MXRMB,A,0.002,L,001,002,1625.544,N,08548.850,W,6.587,230.390,4.734,V*1D
  int apCTE = 0.0;
  bool apLeft = false; // true - left , false - right
  String apToWay = "";
  String apFromWay = "";
  long apDesWayLat = 0.0, apDesWayLon = 0.0;
  bool apDesWayN = false, apDesWayE = false;
  long apRanToDes = 0.0;
  int apBeaToDes = 0;
  long apDesCloVel = 0.0;
  bool apArrival = false;
*/



void SerialsBegin(void) {
  SerialI.begin(4800);
  SerialO.begin(4800);
}
void SerialsEnd(void) {
  SerialI.end();
  SerialO.end();
}

void setup() {
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);

  SerialsBegin();
  SerialO.stopListening();
  pinMode(SEATALK_RX, INPUT);

  Serial.begin(57600);
  while (!Serial);
  Serial.println("hello:)");

  stDepth = NULL;
  stSog = NULL;
  stCog = NULL;
  stHeading = NULL;
  windAngle = NULL;
  windAngleAvg = 0;
  windAvg = 0;
  windAngleLast = 0;
  stLat = String("");
  stLon = String("");
  usartMsg = String("");

  oldMillis = 0;
}

void blink(void) {
  digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(100);              // wait for a second
  digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW

}


bool checkStBusy() {
  //return true;
  int c;
  for (c = 0; c < 255; c++) {
    if ( digitalRead(SEATALK_RX) == 0)
      delay(5);
    //readUSART();
    else
      break;
  }
}

uint8_t get_8(int i) {
  return st[i];
}
uint8_t get_4(int i) {
  if (i % 2 == 0) return (st[i / 2] >> 4) & 0xf;
  else return (st[i / 2]) & 0x0f;
}
uint32_t get_int(int i1, int i2 = -1, int i3 = -1, int i4 = -1) {
  uint32_t x = st[i1];
  if (i2 >= 0) x |= st[i2] << 8;
  if (i3 >= 0) x |= st[i3] << 16;
  if (i4 >= 0) x |= st[i4] << 24;
  return x;
}


void printSt(int len, ...) {
  checkStBusy();
  va_list st;
  va_start(st, len);
  for (int i = 0; i < len; i++)
    SerialO.write9(va_arg(st, int));
  va_end(st);

  //printOut("printSt","printing to seatalk DONE");
}

int getCheckSum(String string) {
  int XOR = 0;
  for (int i = 0; i < string.length(); i++)
    XOR = XOR ^ string[i];

  return XOR;
}
bool printNmea(String msg) {
  if (!printNmeaToUsb)
    return false;
  msg += ",A";
  Serial.print("$");
  Serial.print(msg);
  Serial.print("*");
  Serial.println(String(getCheckSum(msg), HEX));
  return true;
}
void printOutN(String msg) {
  printNmea(String("ARUNO,") + msg);
}
bool printOut(String vName, String nVal) {
  if (!printKnownST)
    return false;
  Serial.println("#" + vName + ": " + nVal);
  return true;
}
//String pt = String();
void displayLoop() {
  printOut(F("----------------------"), F("----------------------"));
  if ( stCog && stSog && stHeading ) {
    printNmea("SDHSC," + String(stCog) + ",T," + String(stHeading) + ",M");
    printNmea("SDOSD," + String(stCog) + ",A," + String(stHeading) + ",M," + String(stSog, 1) + ",G,0,0,K");
  }
}
void printLL(void) {
  if ( stLat != "" ) {
    printOut("Lat", stLat);
    printOut("Lon", stLon);

    String ts = String(
                  "SDRMC,,A," +
                  String(stMLat) + "," + String(stNS) + "," +
                  String(stMLon) + "," + String(stEW) + "," +
                  (stSog ? String(stSog, 2) : "0") + "," +
                  (stCog ? String(stCog) : "0") + "," +
                  ",,"
                );
    printNmea(ts);
    ts = "";
  }
}

void printAP(void) {
  if ( apCourse ) {
    printOut("AP course", String(apCourse));
    printNmea("SDAPD," + String((int)(apCourse)) + ",M," + String((int)(apStatus)) + ",S");
  }

  printOut("AP rudder", String(apRudder));
  //printNmea("SDRSA,"+String((int)(apRudder))+",A,-"+String((int)(apRudder))+",A");
  printNmea("SDRSA," + String((int)(apRudder)) + ",A,");

  if ( apStatus == 0) printOut("AP mode", "standby");
  if ( apStatus == 2) printOut("AP mode", "auto");
  if ( apStatus == 6) printOut("AP mode", "vane");
  if ( apStatus == 8) printOut("AP mode", "track");

}
void printCog(void) {
  if ( stCog ) {
    printOut("COG", String(stCog));
    printNmea("SDHDG," + String(stCog) + ",0.0,W,0.0,E");
  }
}
void printDepth(void) {
  if ( stDepth != NULL ) {
    //printOut("Depth offset", String(stDepthOffset,1));
    if ( (int)(stDepth) < 10 ) {
      printOut("Depth trans", String(stDepth, 1));
      printOut("Depth", String((stDepth + stDepthOffset), 1));
      printNmea("SDDBT," + String(stDepth, 1));
      printNmea("SDDPT," + String((stDepth + stDepthOffset), 1));

    } else {
      printOut("Depth trans", String((int)(stDepth)));
      printOut("Depth", String((int)(stDepth + stDepthOffset)));
      printNmea("SDDBT," + String((int)(stDepth)));
      printNmea("SDDPT," + String((int)(stDepth + stDepthOffset)));


    }

    stDepth = NULL;
  }
}
void printSog(void) {
  if ( stSog != NULL )
    //printOut("Speed over ground", String(stSog,1));
    printOut("SOG", String(stSog, 1));
}
void printWindAngle(void) {
  if ( windAngle != NULL ) {
    printOut("Wind Angle", String(((float)windAngleLast / 2.0), 0));
    printNmea("SDMWV," + String((float)(windAngleLast / 2.0), 0) + ",R,0.0,K" );
  }
}
void printHeading(void) {
  if ( stHeading != NULL ) {
    printOut("Heading flux", String(stHeading));
    printNmea("SDHDM," + String(stHeading) + ",M");
  }
}


bool printLastFullCommandFound() {
  if ( !printNaNST ) return false;
  Serial.print("NaN.ST\t");
  for (int i = 0; i < ind; i++) {
    Serial.print(st[i], HEX);
    Serial.print(" ");
  }
  Serial.println("");

  return true;
}

void readSeaTalk() {
  if (SerialI.available ()) {
    inbyte = (SerialI.read());
    //Serial.println(inbyte,HEX);
    if (inbyte & 0x100) {
      if ( !stMesKnown ) {
        printLastFullCommandFound();
      }
      stMesKnown = false;
      ind = 0 ;
      //Serial.println(inbyte,HEX);
    }

    st[ind++] = inbyte ;
    
    if (ind == 5 && st[0] == 0x00) { // depth
      stMesKnown = true;
      parsDepth();
      printDepth();
    }

    if (ind == 3 && st[0] == 0x30) { // lamp
      stMesKnown = true;
      uint8_t a = get_8(2) & 0x0f;
      uint8_t l = 0;
      if (!a) l = 0;
      if (a == 4) l = 1;
      if (a == 8) l = 2;
      if (a = 0xC) l = 3;
      printOut("Lamp", String(l));
    }

    if (ind == 4 && st[0] == 0x52) { // sog
      stMesKnown = true;
      parsSog();
      printSog();
    }

    if (ind == 3 && st[0] == 0x53) { // cog
      stMesKnown = true;
      parsCog();
      printCog();
    }

    if (ind == 4 && st[0] == 0x56) { // RMC date from gps
      stMesKnown = true;
      uint8_t m = get_4(2);
      uint8_t d = get_8(2), y = get_8(3);
      printOut("Date (dd/mm/yy)", String(String(d) + "/" + String(m) + "/" + String(y)));
    }

    if (ind == 8 && st[0] == 0x58) { // lat lon
      stMesKnown = true;
      parsLL();
      printLL();
    }

    if (ind == 9 && st[0] == 0x84) { // compass heading autopilot coure
      stMesKnown = true;
      parseComHeaAutCou();
      printAP();
    }

    if (ind == 4 && st[0] == 0x9c) { // heading
      stMesKnown = true;
      parsHeading();
      printHeading();
    }
    
    //if( stMesKnown ) 
    //  Serial.println("stKnown");

    if ( ind > 30 )
      ind = 0;

  }

}

void parseComHeaAutCou() {
  apStatus = (st[4] & 0x0f);
  apCourse = ((st[2] % 0xf0) >> 6) * 90 + (st[3]) / 2;
  apRudder = st[6];
  if ( apRudder > 128 )
    apRudder = apRudder - 255;

  apRudder /= 2;
}

void parsHeading() {
  uint8_t u = st[1] >> 4;
  uint8_t vw = st[2];
  stHeading = (u & 0x3) * 90 + (vw & 0x3f) * 2 + (u & 0xc ? (u & 0xc == 0xc ? 2 : 1) : 0);
}

void parsDepth () {
  stDepth = (makeWord(st[4] , st[3])) * 0.03048;// - stDepthOffset;
}

void parsCog() {
  uint8_t u = st[1] >> 4;
  uint8_t vw = st[2];
  stCog = (u & 0x3) * 90 + (vw & 0x3f) * 2 + (u & 0xc) / 8;
}
void parsSog() {
  stSog = (makeWord(st[2] + st[3]));
  stSog *= 0.1;
}


String stToLL(char posNeg, int deg, float mm, float sec) {
  return String(
           "" + String(posNeg) + " " +
           String( deg ) +
           "." +
           String( ( mm * 256.00 + sec ) / 0.6 )
         );
}

void parsLL() {
  stNS = (st[1] & 0x10 ? 'S' : 'N');
  stEW = (st[1] & 0x20 ? 'E' : 'W');

  String mm = String( (st[3] * (float)256) + st[4], 0);
  if (mm.length() < 5)
    for (int i = 0; i < (mm.length() - 5); i++)
      mm = '0' + mm;
  stMLat = String(st[2]) + mm.substring(0, 2) + '.' + mm.substring(2, -1);

  mm = String( (st[6] * (float)256) + st[7], 0);
  if (mm.length() < 5)
    for (int i = 0; i < (mm.length() - 5); i++)
      mm = '0' + mm;
  stMLon = String(st[5]) + mm.substring(0, 2) + '.' + mm.substring(2, -1);

  stLat = stToLL( stNS, st[2], st[3], st[4] );
  stLon = stToLL( stEW, st[5], st[6], st[7] );
}
/*
  $LERMB,A,0.000,L,001,002,1827.718,N,07756.379,W,0.132,75.069,nan,V*77
  $LERMC,125952,A,1827.684,N,07756.514,W,nan,nan,160418,,,*3A
  $LEAPB,A,A,0.0,L,N,V,V,75.2,T,002,75.1,T,75.1,T*0E
  $LEXTE,A,A,0.000,L,N*40
*/
void actionPrintHelp(void) {
  printOutN("This is command list to execute on arduino board :)");
  printOutN("  a - autopilot on");
  printOutN("  w - wind mode");
  printOutN("  t - track mode");
  printOutN("  s - standby");
  printOutN("  d - bt display");
  printOutN("  + - +1 to autopilot heading");
  printOutN("  - - -1 to autopilot heading");
  printOutN("  q - test wind sensor potentiometr burst data adc");
  printOutN("  lx - backlight set to x [0,1,2,3]");
  printOutN("  $00UNO,x*AA - k command parse as str query to multiplexer");
  printOutN("  TODO $xxaaa,......A*AA - command started with \"$\" will be parse and push to seatalk");
  printOutN("  $STUNO,a,b,*AA - push to seatalk");

}
String Nget(int s) {
  int c = 0;
  int cc = 0;
  for (uint8_t i = 0, ic = usartMsg.length(); i < ic; i++) {
    if ( usartMsg[i] == ',' ) {
      if ( s == cc ) {
        Serial.println("found [" + String(usartMsg.substring(c, i)) + "]" + String(s));
        return String( usartMsg.substring(c, i) );
      }
      c = i + 1;
      cc++;
    }
  }

  return "";
}
void sendxte( uint16_t bearing, float distXtc, float distToStear){
  Serial.println("bearing "+String(bearing)+" xtc"+String(distXtc)+" dts"+String(distToStear));
  bool stLeft = false;
  //stLeft = true;
    
  uint16_t dist = (uint16_t)((distXtc) * 100.00);  
  uint16_t dts = (uint16_t)(distToStear*10.00);
  //uint16_t distToStear = (uint16_t)(bearing/1.46);
  
  
  uint8_t x6 = 0x06+((dist&0xf)<<4);
  uint8_t xx = (dist&0xff0)>>4;
  uint8_t u = (bearing/90);
  uint16_t v = (bearing-(u*90))*2;
  uint8_t vu = ((v&0x0f<<4))+u;//0x40;

  
  uint8_t zw = ((((uint16_t)dts)&0xf00)>>4) + ((v&0xf0)>>4);
  uint8_t zz = (((uint16_t)dts)&0xff);
  uint8_t yf = 0x57 + (stLeft?0x80:0x00);
  uint8_t oo = 0x00;
  uint8_t yff = 0xA8;
  
  printSt(9,0x185,x6,xx,vu,zw,zz,yf,oo,yff);
  
  //printSt(9,0x185,0x06,0x00,0xB1,0x51,0x01,0x57,0x00,0xA8);
}
void ParseLine(void) {
  //if(usartMsg.length() == 0 ) return 0;

  if (usartMsg.length() == 8 &&
      usartMsg[0] == '$' &&
      usartMsg[1] == '0' &&
      usartMsg[2] == '0' &&
      usartMsg[3] == 'U' &&
      usartMsg[4] == 'N' &&
      usartMsg[5] == 'O' ) {

    if ( usartMsg[8] == 0)
      usartMsg = String(usartMsg[7]);
  } else if (usartMsg[0] == '$' &&
             usartMsg[3] == 'R' &&
             usartMsg[4] == 'M' &&
             usartMsg[5] == 'B' ) {
    // from garmin gpsmap620 to cross error waypoint track
    // RMB
    sendxte(
      Nget(11).toInt(),
      Nget(2).toFloat(),
      Nget(10).toFloat()
      );



  } else if (usartMsg[0] == '$' &&
             usartMsg[1] == 'S' &&
             usartMsg[2] == 'T' &&
             usartMsg[3] == 'U' &&
             usartMsg[4] == 'N' &&
             usartMsg[5] == 'O' ) {
    usartMsg = (usartMsg).substring(7);
    printOutN("ST cmd !");
    printOutN("cmd[" + usartMsg + "]");
    int c = 0;
    int ts = 0;
    String ss = "";
    checkStBusy();
    for (uint8_t i = 0, ic = usartMsg.length(); i < ic; i++) {
      if ( usartMsg[i] == ',' ) {
        ss = usartMsg.substring(c, i);
        ts = (int)(strtol( &ss[0], NULL, 16));
        if ( c == 0 )
          ts += 0x100;

        SerialO.write9(ts);
        c = i + 1;
      }
    }
    printOutN("count" + String(c));
  }

  if (usartMsg.length() == 2) {
    if (usartMsg[0] == 'l' ) {
      uint8_t l = 0;
      if ( usartMsg[1] == '0' ) l = 0;
      else if ( usartMsg[1] == '1' ) l = 4;
      else if ( usartMsg[1] == '2' ) l = 8;
      else if ( usartMsg[1] == '3' ) l = 0x0c;
      printSt(3, 0x130, 0x00, l );
    }
  } else if (usartMsg.length() == 1) {
    if (usartMsg[0] == '?' ) actionPrintHelp();

    if (usartMsg[0] == 'q' ) windBurst = (windBurst ? false : true );

    if (usartMsg[0] == 'a' ) printSt( 4, 0x186, 0x21, 0x01, 0xfe);
    if (usartMsg[0] == 's' ) printSt( 4, 0x186, 0x21, 0x02, 0xfd);
    if (usartMsg[0] == 't' ) printSt( 4, 0x186, 0x21, 0x03, 0xfc);
    if (usartMsg[0] == 'd' ) printSt( 4, 0x186, 0x21, 0x04, 0xfb);
    if (usartMsg[0] == 'w' ) printSt( 4, 0x186, 0x21, 0x23, 0xdc);


    if (usartMsg[0] == '+' ) printSt( 4, 0x186, 0x11, 0x07, 0xf8);
    if (usartMsg[0] == '-' ) printSt( 4, 0x186, 0x11, 0x05, 0xfa);
  }

  //printOut("------------------------\n#parse line DONE","0");
  //printOutN("message\n#\t["+usartMsg+"]\n#\tlen["+usartMsg.length()+"]");
  //printNmea("00UNO,["+usartMsg+"] len["+usartMsg.length()+"]");
  Serial.println("00UNO,[" + usartMsg + "] len[" + usartMsg.length() + "]");
  usartMsg = String("");

}

bool readUSART() {
  while (Serial.available()) {
    inbyte = Serial.read();
    // blink();
    if ( (inbyte == '\n') || (inbyte == '\r') || usartMsg.length() >= 100) {
      if ( usartMsg.length() == 0 )
        return false;
      else
        return true;
    }
    usartMsg += String((char)(inbyte));
  }
  return false;
}


void instrumentWindIter(void) {
  windAngle = (analogRead(windAnglePin));
  //Serial.println(windAngle);
  /*
    potentiometer is not 360 it is ~255` so converting 1024 from adc to equal scale
    235` pot
    0.459
    125`
  */
  if (windAngle <= 512) {
    windAngle = (int)((float)windAngle * 0.449) + 125;
    windAngleAvg += 360 - windAngle;
  } else {
    windAngle = (int)((float)windAngle * 0.449) + 125;
    windAngleAvg += 720 - (windAngle - 360);
  }

  windAvg++;

}
void instrumentWindPrint(void) {
  windAngleLast = (int)((float)(windAngleAvg / (float)(windAvg)));
  windAngleAvg = 0;
  windAvg = 0;
  printSt( 4, 0x110, 0x01, (((windAngleLast) >> 8) & 0xff), ((windAngleLast) & 0xff) );
  printSt( 4, 0x111, 0x01, (((9000) >> 8) & 0xff), 0x01 );
}


void sendA2(){
  String pName = String("XYZEO");
  pName.toUpperCase();
  
  uint8_t x4 = 0x04;

checkStBusy();
SerialO.write9(0x182);
SerialO.write9(0x05); 
SerialO.write9(0xD0);
SerialO.write9(0x2F);
SerialO.write9(0xE5);
SerialO.write9(0x1A);
SerialO.write9(0x51);
SerialO.write9(0xAE); 

checkStBusy();
SerialO.write9(0x185);
SerialO.write9(0x06); 
SerialO.write9(0x00); 
SerialO.write9(0x00); 
SerialO.write9(0x00);
SerialO.write9(0x00);
SerialO.write9(0x10);
SerialO.write9(0x00);
SerialO.write9(0xEF);

checkStBusy();
SerialO.write9(0x19F); 
SerialO.write9(0xE4); 
SerialO.write9(0x09);
SerialO.write9(0x0C);
SerialO.write9(0x9C);
SerialO.write9(0x00);
SerialO.write9(0x9C);



  checkStBusy();
  SerialO.write9(0x1A2);
  SerialO.write9(0x04);
  SerialO.write9(0x01);
  SerialO.write9(0x00);
  SerialO.write9(0x45);
  SerialO.write9(0x4e);
  SerialO.write9(0x44);

}

byte i = 0;
void loop() {

  if ( windBurst ) {

    delay(10);
    instrumentWindIter();
    delay(10);
    instrumentWindPrint();
    delay(10);
    printWindAngle();
    delay(10);
    if ( readUSART() != NULL )
      ParseLine();
    delay(10);
  }

  currentMillis = millis();
  readSeaTalk ();

  if ( readUSART() != NULL ) {
    ParseLine();
  }
  if ( (oldMillis + 100) < currentMillis || oldMillis > currentMillis ) {
    //instrumentWindIter();
    oldMillis = currentMillis;
    i++;
  }

  if ( i > 6 ) {
    i = 0;
    instrumentWindIter();
    instrumentWindPrint();
    displayLoop();
    printWindAngle();

    //Serial.println("send xtc :P");
    //sendxte();
    //Serial.println("send A2");
    //sendA2();
  }

}
