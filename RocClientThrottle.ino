#define ver 36
#include "Secrets.h"

uint8_t Volts_Calibration;


// new stuff for buttons
#include <JC_Button.h>          // https://github.com/JChristensen/JC_Button
const unsigned long
LONG_PRESS(500);

bool     Fourway_SW_Menu_Action_Enabled;   // to assist with  the menu level change in four way switchto avoid immediately sending a function action
bool     Menu_Action_Enabled;   // to assist with  the menu level change to avoid immediately sending a function action
bool PowerON;                   // to assist with making the menu work with vry lng press for power on

Button ButtonLastLoco(buttonLastLoco);
Button ButtonNextLoco(buttonNextLoco);
Button ButtonBackward(buttonBackward);
Button ButtonForward(buttonForward);
Button ButtonStop(buttonStop);
Button ButtonF0(buttonF0);
Button ButtonF1(buttonF1);
Button ButtonF2(buttonF2);
Button ButtonF3(buttonF3);
Button ButtonF4(buttonF4);
Button ButtonF5(buttonF5);
Button ButtonF6(buttonF6);
Button ButtonF7(buttonF7);

float analog_FP;
int batt_calibration;

#include <SSD1306.h> // alias for `#include "SSD1306Wire.h"` //https://github.com/ThingPulse/esp8266-oled-ssd1306
// connect to display using pins D1, D2 for I2C on address 0x3c with esp32, 5,4 with esp32?
SSD1306  display(0x3C, OLED_SDA, OLED_SCL);
// #include <PubSubClient.h> is included in MQTT.cpp.. Very important to read the notes there...

#include "Terrier.h"

#include <ArduinoOTA.h>
#ifdef _Use_Wifi_Manager
#include <WiFiManager.h>
#else
String wifiSSID = SSID_RR;
String wifiPassword = PASS_RR;
#endif
int disconnected;
int BrokerAddr;
String NameOfThisThrottle = ThrottleNameDefault;
#ifdef ESP32
#include <WiFi.h>
#else
#include <ESP8266WiFi.h>
#endif
uint8_t wifiaddr;
uint8_t ip0;
uint8_t ip1;
uint8_t    subIPH;
uint8_t    subIPL;
IPAddress ipBroad;
IPAddress mosquitto;
int connects;
bool  WIFI_SETUP;

uint32_t LoopTimer;
uint32_t DisplayTimer;
extern int hrs;
extern int mins;
extern int secs;
extern int clock_divider;
extern bool Clock_Freeze;
uint32_t LastSynchTime;
uint32_t LastSecTime;
uint32_t TIME;
long StartupAt;
bool LcpropsSent;

#include "MQTT.h" //new place for mqtt stuff (defines maxlocos
//LOCO throttle settings from mqtt
extern int LocoNumbers; //set in parse to actual number of locos in lc list
extern String LOCO_id[MAXLOCOS + 1];
extern String SW_id[MAXLOCOS + 1];
extern int SW_bus[MAXLOCOS + 1];
extern int SW_addr[MAXLOCOS + 1];

#include "Menu.h" //place for menu and display stuff, uses loco_id

int y;
bool EncoderA = false;
bool EncoderB = false;
bool RightButtonState = false;         // variable for reading the pushbutton status
bool SelectButtonState = false;         // variable for reading the pushbutton status
bool UpButtonState = false;         // variable for reading the pushbutton status
bool DownButtonState = false;         // variable for reading the pushbutton status
bool LeftButtonState = false;         // variable for reading the pushbutton status
bool TopPB_State, MidPB_State, BottomPB_State; //for membrane
bool buttonpressed;
uint32_t ButtonPressTimer;
// display and selection stuff
int MenuLevel;
int switchindex;
int RightIndexPos;
int LeftIndexPos;
int locoindex;
int CurrentLocoIndex;
int fnindex;
int speedindex;
bool directionindex;
bool AllDataRead;

int functionPage = 0;

bool ButtonLastLocoState = false;
bool ButtonNextLocoState = false;
bool ButtonBackwardState = false;
bool ButtonForwardState = false;
bool ButtonStopState = false;



// Fader
int faderPos = 0;
int faderPosOld = 0;
int faderPosNew = 50;
bool setFader = true; // setFader to new value
bool getFader = false;
bool updateFader = false; // loco has changed, set fader to new position
bool blockFader = false;
bool stopLoco = false;
int serverSpeedOld = 0;
int serverSpeedNew = 0;
int faderSpeedNew = 0;
//bool faderDirection = true;
int maxFaderPos = 230;
int faderPosServer = 0;
#define CHANNEL_FADER_UP 0
#define CHANNEL_FADER_DOWN 1

// MedianFilter
#define MEDIAN_FILTER_SIZE 20
int medianFilterValues[MEDIAN_FILTER_SIZE];
int medianFilterPos = 0;
bool initMedianFilter = true;

// PWM for fader
#define PWM_FREQUENCY 1000
#define PWM_RESOUTION 8

const int MaxAttribSize = 35;
char FoundAttrib[MaxAttribSize];// FoundAttrib= will be the attributes found by the attrib and attribcolon  functions;
char DebugMessage[128];
#include "NVSettingInterface.h"  // location for te EEPROM write and interface stuff
// allows call to check buttonstate

// Todo
bool StopPressed(){
  return false;
}



// These OLED display functions allow calling from anywhere, including my libraries.
void Picture() {
  display.clear();
  display.drawXbm(1, 1, Terrier_Logo_width, Terrier_Logo_height, Terrier_Logo);
  //  display.display();
}


void OLED_5_line_display_p(String L1, String L2, String L3, String L4, String L5) {
  display.clear();
  Picture();
  display.setFont(ArialMT_Plain_10);
  display.drawString(64, 0, L1);
  display.drawString(64, 10, L2);
  display.drawString(64, 22, L3);
  display.drawString(64, 34, L4);
  display.drawString(64, 46, L5);
  display.display();
}
void OLED_5_line_display(String L1, String L2, String L3, String L4, String L5) {
  display.clear();
  // Picture();
  display.setFont(ArialMT_Plain_10);
  display.drawString(64, 0, L1);
  display.drawString(64, 10, L2);
  display.drawString(64, 22, L3);
  display.drawString(64, 34, L4);
  display.drawString(64, 46, L5);
  display.display();
}

void ConnectionPrint() {


  Serial.println(F("-----------------------------------------------------------"));
  Serial.print (F("---------- Connected to SSID:"));
  Serial.print(WiFi.SSID());
  Serial.print(F("  IP:"));
  Serial.println(WiFi.localIP());
}

void Banner() {
  String MSGText1; String MSGText2;
  display.init();
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.setFont(ArialMT_Plain_10);
  MSGText1 = "Looking for <"; MSGText1 += wifiSSID; MSGText1 += "> ";
  MSGText2 = "code <Ver:"; MSGText2 += ver; MSGText2 += "> ";
  OLED_5_line_display_p("", " ", MSGText1, MSGText2, "");
  Serial.println(F("-----------------------------------------------------------"));
  Serial.println(F("            This is Rocrail Client Throttle "));
  Serial.print  (F("            Named      \"")); Serial.print(NameOfThisThrottle); Serial.println("\" ");
  Serial.print(F("-------------------- limit "));
  Serial.print(MAXLOCOS);
  Serial.println(F( " locos -----------------------"));
  Serial.print(F(  "                    revision:"));
  Serial.println(ver);
  Serial.println(F("-----------------------------------------------------------"));
  delay(100);
}


void _SetupOTA(String StuffToAdd) {
  String Name;
  // ota stuff  Port defaults to 8266
  // ArduinoOTA.setPort(8266);
  // Hostname defaults to esp8266-[ChipID]


  Name = "RocMouse<";
  Name = Name + StuffToAdd;
  Name = Name + ">";
  Serial.printf("------ Setting OTA Hostname <%s>\n", Name.c_str());
  Serial.println(F("-----------------------------------------------------------"));
  ArduinoOTA.setHostname(Name.c_str());
  // No authentication by default
  //ArduinoOTA.setPassword((const char *)"123");

  ArduinoOTA.onStart([]() {
    Serial.println("Start");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();
  //---------------------end ota stuff -------

}

void ConnectToWifi(String WiSSID, String Password, int Broker) {
  //Void Status(){
  String MSGText; String MSGText1; String MSGText2;
  int counter;
  delay(100);
#ifdef _Use_Wifi_Manager
  WiFiManager wifiManager;  // this  stores ssid and password invisibly  !!
  //reset settings - for testing
  //wifiManager.resetSettings();
  wifiManager.autoConnect("ROCNODE ESP AP");  //ap name (with no password) to be used if last ssid password not found
#else

  WiFi.mode(WIFI_STA);  //Alternate "normal" connection to wifi
#ifdef ESP32
  // nothing needed here, the ESP32 doe not have the 'set output power' function
#else
  WiFi.setOutputPower(30);//  0 sets transmit power to 0dbm to lower power consumption, but reduces usable range.. try 30 for extra range
#endif
  MSGText1 = "Trying SSID<";  MSGText1 +=  WiSSID;  MSGText1 += "> ";
  OLED_5_line_display_p("", "", " ", MSGText1, "");
  MSGText1 = "- Trying to connect to SSID<";  MSGText1 +=  WiSSID;  MSGText1 += "> "; // longer message for serial interface...
  MSGText2 += " password <";  MSGText2 +=  "secret";  MSGText2 += "> ";
  counter = 0;
  Serial.println(F("-----------------------------------------------------------"));
  Serial.print (MSGText1); Serial.print(MSGText2); Serial.print(" BrokerAddr:"); Serial.println(Broker);
  WiFi.begin(WiSSID.c_str(), Password.c_str());
  delay(100);
  while (WiFi.status() != WL_CONNECTED) {
    delay(100);
    Serial.print(".");
    counter++;
    if (counter >= 30) {
      counter = 0;
      Serial.println("");
    }
  }

#endif // not using wifi manager

  //if you get here you should have connected to the WiFi
  //  Serial.println();Serial.println(F("--Connected--"));

  ipBroad = WiFi.localIP();
  subIPH = ipBroad[2];
  subIPL = ipBroad[3];
  wifiaddr = ipBroad[3];
  ConnectionPrint();  // display connection screen
  //   ++++++++++ MQTT setup stuff   +++++++++++++++++++++
  mosquitto[0] = ipBroad[0]; mosquitto[1] = ipBroad[1]; mosquitto[2] = ipBroad[2];
  mosquitto[3] = Broker; // mosquitto address
  Serial.print(F("--------Mosquitto will first try to connect to:"));
  Serial.println(mosquitto);
  MQTT_ReConnect();
  //if you get here you should have connected to the MQTT broker but MQTT_ReConnect() in MQTT includes code to search addr 3 to 75 if its not at the expected address
  Serial.println(F("----------------MQTT NOW setup ----------------"));
  WIFI_SETUP = true;
  _SetupOTA(NameOfThisThrottle); // now we  have set the ota update with nickname ThrottleName
  //Note In Arduino Ports: You will also see the IP address so you can select which throttle of many with the same name to update OTA
  MSGText = " IP:"; MSGText += ipBroad[0]; MSGText += ":"; MSGText += ipBroad[1]; MSGText += ":"; MSGText += ipBroad[2]; MSGText += ":"; MSGText += wifiaddr;
  OLED_5_line_display_p("", "", " ", "WiFi Connected", MSGText); delay(1000);
}




void PrintLocoSettings() {
  for (int loco = 1; loco <= LocoNumbers; loco++) {
    Serial.print(F("Attributes stored for <")); Serial.print(loco); Serial.print(F("> <")); Serial.print(LOCO_id[loco]);

    Serial.println("");
  }
}




void setup() {
  bool UsedDefaults;

  String msg; // local variable for any string message builds
  // init serial port

  Serial.begin(115200);
  delay(500); // allow time for serial to set up
  Serial.println("-----Serial Port Running------");
  // set the builtin LED pin to work as an output
  pinMode(On_Board_LED, OUTPUT);
  pinMode (ADC_IN, INPUT);
  pinMode (ADC_SPEED, INPUT);
  // Fader motor
  ledcSetup(CHANNEL_FADER_UP, PWM_FREQUENCY, PWM_RESOUTION);
  ledcSetup(CHANNEL_FADER_DOWN, PWM_FREQUENCY, PWM_RESOUTION);
  ledcAttachPin(FaderUp, CHANNEL_FADER_UP);
  ledcAttachPin(FaderDown, CHANNEL_FADER_DOWN);
  //pinMode (FaderUp, OUTPUT);
  //pinMode (FaderDown, OUTPUT);
  WIFI_SETUP = false;
  EEPROM.begin(EEPROM_Size); //Initialize EEPROM
  UsedDefaults = false;
  // TestFillEEPROM(72); // used for test only to check read eeprom etc functions
  if (strcmp(SSID_RR, "Router Name") == 0) {
    wifiSSID = read_String(ssidEEPROMLocation);
    wifiPassword = read_String(passwordEEPROMLocation);
    BrokerAddr = EEPROM.read(BrokerEEPROMLocation);
    NameOfThisThrottle = read_String(ThrottleNameEEPROMLocation);
  } else {
    Serial.println("Using Secrets.h settings");
    wifiSSID = SSID_RR;
    wifiPassword = PASS_RR;
    BrokerAddr = BrokerAddrDefault;
    NameOfThisThrottle = ThrottleNameDefault;
  }
  Volts_Calibration = EEPROM.read(CalEEPROMLocation);
  if (Volts_Calibration == 0) {
    Volts_Calibration = 91;
  }
  Serial.print(" Broker addr:"); Serial.println(BrokerAddr);
  Serial.print(" Battery Volts_Calibration Factor:"); Serial.println(Volts_Calibration);
  if ((wifiSSID == "") || (wifiSSID.length() >= 90)) {
    wifiSSID = SSID_RR;  // if empty, or if bigger than 90 use the secrets default
    UsedDefaults = true;
    Serial.println("Using Default SSID");
  }
  if ((wifiPassword == "") || (wifiPassword.length() >= 90)) {
    wifiPassword = PASS_RR;  // if empty, or if bigger than 90 use the secrets default
    UsedDefaults = true;
    Serial.println("Using Default Password");
  }
  if ((NameOfThisThrottle == "") || (NameOfThisThrottle.length() >= 90)) {
    NameOfThisThrottle = ThrottleNameDefault;  // if empty, or if bigger than 90 use the secrets default
    UsedDefaults = true;
    Serial.println("Using Default Throttlename");
  }
  if ((BrokerAddr == 0) || (BrokerAddr == 255)) {
    BrokerAddr = BrokerAddrDefault;  // zero and 255 are not  valid Ip for the broker, use default instead
    UsedDefaults = true;
    Serial.println("Using Default Broker address");
  }
  if (UsedDefaults) {
    WriteWiFiSettings();
  }
  Banner();

  CheckForSerialInput(); // do this before you set any other  pin definitions..

  // start the JC button functions
  ButtonLastLoco.begin();
  ButtonNextLoco.begin();
  ButtonBackward.begin();
  ButtonForward.begin();
  ButtonStop.begin();
  ButtonF0.begin();
  ButtonF1.begin();
  ButtonF2.begin();
  ButtonF3.begin();
  ButtonF4.begin();
  ButtonF5.begin();
  ButtonF6.begin();
  ButtonF7.begin();

  TopPB_State = false;
  MidPB_State = false;
  BottomPB_State = false;
  // init the display

  connects = 0;
  ConnectToWifi(wifiSSID, wifiPassword, BrokerAddr);
  //Status();  // where we try to log in to WiFi


  hrs = 0;
  mins = 0;
  secs = 0;
  clock_divider = 1;
  Clock_Freeze = false;
  buttonpressed = false;
  MenuLevel = 0;
  switchindex = 1;
  locoindex = 0;
  CurrentLocoIndex = -1; // set to somethging we cannot have to allow the consistent triggering of the get loco data function in menu
  Menu_Action_Enabled = false;
  Fourway_SW_Menu_Action_Enabled = false;
  PowerON = true; // we cannot set power on, only off, so we assume power is on by default
  fnindex = 0;
  RightIndexPos = 3;
  LeftIndexPos = 0;
  directionindex = true;
  LocoNumbers = 0;
  AllDataRead = false;
  for (int i = 0; i <= (MAXLOCOS) ; i++) {
    LOCO_id[i] = "~blank~";
  }

  StartupAt = millis();
  LastSynchTime = StartupAt;
  LastSecTime = StartupAt;
  clock_divider = 1;
  LcpropsSent = false;

  GetLocoList();
  DisplayTimer = millis();



}

long ReadADC(int avg) {
  long AnalogAvg; int j; long analog_value, BatteryVx100;
#ifdef ESP32
  analogReadResolution(10);
  analogSetAttenuation(ADC_6db); //6db range is 1024 counts to 2v approx
#endif
  j = 0; AnalogAvg = 0;
  for (int i = 0; i <= avg; i++) {
#ifdef ESP32
    analog_value = analogRead(ADC_IN);
#endif
#ifdef ESP8266
    analog_value = analogRead(ADC_IN);
#endif
    if ((analog_value >= 100) && (analog_value <= 1500)) {
      j = j + 1; AnalogAvg = AnalogAvg + analog_value;
    }
  }
  if (j >= 1) {
    analog_value = AnalogAvg / j;
  } else {
#ifdef ESP32
    analog_value = analogRead(ADC_IN);
#endif
#ifdef ESP8266
    analog_value = analogRead(ADC_IN);
#endif
  } // average or just last chance at reading!
  BatteryVx100 = analog_value; //
  BatteryVx100 = BatteryVx100 * Volts_Calibration; //~x100
  BatteryVx100 = BatteryVx100 / Cal_factor;
  return BatteryVx100;
}

// TODO
long ReadADCSpeed(int avg) {
  long analog_value;
  analogReadResolution(8);
  analogSetAttenuation(ADC_6db); //6db range is 1024 counts to 2v approx
  analog_value = analogRead(ADC_SPEED);
  return doMedianFilter(analog_value);
}

void MQTT_DO(void) {
  // MQTT stuff, & check MQTT is connected.. WiFi conection test is separate!

  if (!MQTT_Connected()) {
    MQTT_ReConnect();  // was if (!client.connected())
    delay(100);
  }
  MQTT_Loop(); // for client.loop(); //gets wifi messages etc..
}

void GetLocoList() {
  ParseIndex = 0;
  AllDataRead = false;
  MQTT_DO(); // needed to make the function work in startup, before mqtt is running in the main loop
  MQTTSend("rocrail/service/client", "<model cmd=\"lcprops\" />");
}

void GetLocoFunctions(int index) {
  char MsgTemp[127];
  sprintf ( MsgTemp, "<model cmd=\"lcprops\" val=\"%s\"/>", LOCO_id[index].c_str());
  //Serial.println(MsgTemp);  //https://wiki.rocrail.net/doku.php?id=cs-protocol-en
  for (int i = 0; i <= (N_Functions) ; i++) {
    String msg;
    msg = "F"; msg = msg + i;
    FunctionName[i] = msg;
    FunctionState[i] = false;
    FunctionStateKnown[i] = false;
    FunctionTimer[i] = 0;
  }
  MQTTSend("rocrail/service/client", MsgTemp);

}





void ClockUpdate() {
  if (!Clock_Freeze) { // Rocrail TIME is time since epoch start Seconds since Jan 01 1970. (UTC)
    if (((LoopTimer - LastSynchTime)*clock_divider) >= 1000) {
      secs = secs + 1;
      LastSynchTime = LoopTimer;
    }

    if (secs >= 59.5) {
      secs = 0; mins = mins + 1; LastSynchTime = LoopTimer; // Serial.print("%"); // do stuff here for tests every second..for debug
      if (mins >= 60) {
        hrs = hrs + 1;
        mins = 0;
      }
      if (hrs >= 25) {
        hrs = 1; //
      }
    }
  } else {                } //

}


boolean WiFiReturns() {  //https://github.com/esp8266/Arduino/issues/4161#issuecomment-378086456
  if (WiFi.localIP() == IPAddress(0, 0, 0, 0)) return 0;
  switch (WiFi.status()) {
    case WL_NO_SHIELD: return 0;
    case WL_IDLE_STATUS: return 0;
    case WL_NO_SSID_AVAIL: return 0;
    case WL_SCAN_COMPLETED: return 1;
    case WL_CONNECTED: return 1;
    case WL_CONNECT_FAILED: return 0;
    case WL_CONNECTION_LOST: return 0;
    case WL_DISCONNECTED: return 0;
    default: return 0;
  }
}

int doMedianFilter(int input) {
  int i, j, m;
  int medianValue;
  int medianFilterSort[MEDIAN_FILTER_SIZE];

  if (initMedianFilter) {
    for (i = 0; i < MEDIAN_FILTER_SIZE; i++)
      medianFilterValues[i] = input;
    initMedianFilter = false;
  }

  medianFilterValues[medianFilterPos] = input;
  // copy
  for (i = 0; i < MEDIAN_FILTER_SIZE; i++)
    medianFilterSort[i] = medianFilterValues[i];
  // sort
  for (i = 0; i < MEDIAN_FILTER_SIZE; i++)
    for (j = 0; j < MEDIAN_FILTER_SIZE - i - 1; j++)
      if (medianFilterSort[j] > medianFilterSort[j + 1]) {
        m = medianFilterSort[j];
        medianFilterSort[j] = medianFilterSort[j + 1];
        medianFilterSort[j + 1] = m;
      }

  medianValue = medianFilterSort[(int) (MEDIAN_FILTER_SIZE / 2)];
  /*
    Serial.print("sort (pos=");
    Serial.print(medianFilterPos);
    Serial.print("): ");
    for (i=0;i<MEDIAN_FILTER_SIZE-1;i++) {
      Serial.print(medianFilterSort[i]);
      Serial.print(" ");
    }
    Serial.println(medianFilterSort[MEDIAN_FILTER_SIZE-1]);
  */
  medianFilterPos++;
  if (medianFilterPos >= MEDIAN_FILTER_SIZE)
    medianFilterPos = 0;

  return medianValue;
}

void faderUp(int duty) {
  //digitalWrite(FaderDown, LOW);
  ledcWrite(CHANNEL_FADER_DOWN, 0);
  //digitalWrite(FaderUp, HIGH);
  ledcWrite(CHANNEL_FADER_UP, duty);
}

void faderDown(int duty) {
  //digitalWrite(FaderUp, LOW);
  ledcWrite(CHANNEL_FADER_UP, 0);
  //digitalWrite(FaderDown, HIGH);
  ledcWrite(CHANNEL_FADER_DOWN, duty);
}

void faderStop() {
  ledcWrite(CHANNEL_FADER_UP, 0);
  ledcWrite(CHANNEL_FADER_DOWN, 0);
  //digitalWrite(FaderDown, LOW);
  //digitalWrite(FaderUp, LOW);
}

void setFunction(int button) {
  if (button == 0) {
    Do_Function(locoindex, 0); // State is toggled there
    DoButtonFunction(0, FunctionState[0]);    
  } else {
    int function = button + functionPage * 7;
    if (function <= N_Functions) {
      //FunctionState[function]=!FunctionState[function];
      Do_Function(locoindex, function); // State is toggled there
      DoButtonFunction(function, FunctionState[function]);
    }
  }
}

void loop() {
  int duty;
  
  // turn on the LED
  digitalWrite(On_Board_LED, LOW);
  if (!WiFiReturns()) {
    Serial.println("Lost Connection Trying Reconnect");
    WiFi.reconnect(); delay(200); digitalWrite(On_Board_LED, HIGH); delay(200); //Connection OFF - conexión OFF
  } else { //conexión ON - Connection ON


    //while (WiFi.status() != WL_CONNECTED) {Serial.println("Lost Connection Trying Reconnect");wifi_connect();delay(500); }
    LoopTimer = millis(); // idea is to use LoopTimer instead of millis to ensure synchronous behaviour in loop
    ClockUpdate();

    recvWithEndMarker();
    showNewData();

    // start the JC button functions
    ButtonLastLoco.read();
    ButtonNextLoco.read();
    ButtonBackward.read();
    ButtonForward.read();
    ButtonStop.read();
    ButtonF0.read();
    ButtonF1.read();
    ButtonF2.read();
    ButtonF3.read();
    ButtonF4.read();
    ButtonF5.read();
    ButtonF6.read();
    ButtonF7.read();


    display.clear();
    MQTT_DO();

    DoDisplay(MenuLevel);


    // turn off the LED
    digitalWrite(On_Board_LED, HIGH);
    delay(1);// essentially this is the main loop delay

    serverSpeedNew = abs(speedindex);

    if ((serverSpeedNew != serverSpeedOld || updateFader) && !blockFader && !stopLoco) { // speed has been changed by server
      setFader = true;
      updateFader = false;
      Serial.print("setFader to ");
      Serial.println(serverSpeedNew);
      serverSpeedOld = serverSpeedNew;
      faderPosServer = ( (float) serverSpeedNew / 100 ) * maxFaderPos;
    } else if (serverSpeedNew == faderSpeedNew)
      blockFader=false;

    if (stopLoco) {
      setFader = true;
      faderPosServer = 0;   
      SetLoco(locoindex, 0); 
      stopLoco = false;    
    }
      
    //setFader=false;

    // get position from fader
    faderPosNew = ReadADCSpeed(0);
    if (faderPosNew > maxFaderPos) {
      faderPosNew = maxFaderPos;
    }
    faderSpeedNew = ( (float) faderPosNew / maxFaderPos ) * 100;
    //Serial.println(faderSpeedNew);

    // has fader been changed?
    if ((faderPosNew > faderPosOld + 2 || faderPosNew < faderPosOld - 2) && !setFader) {
      getFader = true;
      Serial.print("setLoco to ");
      Serial.println("faderSpeedNew");
      faderPosOld = faderPosNew;
    }

    if (setFader) {
      duty = 160 + abs(abs(faderPosNew)-abs(faderPosServer));
      if (duty > 255) duty = 255;
      if (faderPosNew < faderPosServer - 2) {
        faderUp(duty);
      } else if (faderPosNew > faderPosServer + 2) {
        faderDown(duty);
      } else {
        faderStop();
        setFader = false;
        faderPosOld = faderPosNew;
        Serial.print("faderPosNew=");
        Serial.println(faderPosNew);
        //test
        faderPosOld = faderPosNew; // Verhindern, dass der Wert wieder an den Server geschrieben wird.
      }
    }
    else if (getFader) {
      if (directionindex)
        SetLoco(locoindex, faderSpeedNew);
      else
        SetLoco(locoindex, faderSpeedNew * -1);
      Serial.print("faderSpeedNew=");
      Serial.println(faderSpeedNew);
      getFader = false;
      blockFader= true;
    }



    // neue Navigation
    if ( ButtonForward.wasPressed() ) {
      if (!directionindex)
        stopLoco = true;
      directionindex = true;
      //DoButtonForward();
    }
    if ( ButtonBackward.wasPressed() ) {
      if (directionindex)
        stopLoco = true;
      directionindex = false;
      //DoButtonBackward();
    }
    if ( ButtonStop.pressedFor(LONG_PRESS) && !ButtonStopState) { // power off
      if (PowerON) {
        MQTTSend("rocrail/service/client","<sys cmd=\"stop\"/>");
        PowerON=false;
        //DoButtonStop(true);
      }
      else {
        MQTTSend("rocrail/service/client","<sys cmd=\"go\"/>");
        PowerON=true;
      }
      ButtonStopState = true;
    }
    if ( ButtonStop.wasPressed() ) { // stop loco
      if (ButtonStopState)
        ButtonStopState = false;
      else {
        // Todo set fader to zero
        //DoButtonStop(false);
        stopLoco = true;
      }
    }
    if ( ButtonNextLoco.pressedFor(LONG_PRESS) && !ButtonNextLocoState) {
      functionPage++;
      if (functionPage > ceil(N_Functions / 8))
        functionPage = 0;
      DoButtonFunctionPage(functionPage);
      ButtonNextLocoState = true;
    }
    if ( ButtonNextLoco.wasPressed() ) {
      if (ButtonNextLocoState)
        ButtonNextLocoState = false;
      else {
        DoButtonNextLoco();
        updateFader = true;
      }
    }
    if ( ButtonLastLoco.pressedFor(LONG_PRESS) && !ButtonLastLocoState) {
      functionPage--;
      if (functionPage < 0)
        functionPage = ceil(N_Functions / 8) - 1;
      DoButtonFunctionPage(functionPage);
      ButtonLastLocoState = true;
    } 
    if ( ButtonLastLoco.wasPressed() ) {
      if (ButtonLastLocoState)
        ButtonLastLocoState = false;
      else {
        DoButtonLastLoco();
        updateFader = true;
      }
    }
    if ( ButtonF0.wasPressed() ) {
      setFunction(0);
    }
    if ( ButtonF1.wasPressed() ) {
      setFunction(1);
    }
    if ( ButtonF2.wasPressed() ) {
      setFunction(2);
    }
    if ( ButtonF3.wasPressed() ) {
      setFunction(3);
    }
    if ( ButtonF4.wasPressed() ) {
      setFunction(4);
    }
    if ( ButtonF5.wasPressed() ) {
      setFunction(5);
    }
    if ( ButtonF6.wasPressed() ) {
      setFunction(6);
    }
    if ( ButtonF7.wasPressed() ) {
      setFunction(7);
    }


    ArduinoOTA.handle();

    // turn off the LED
    digitalWrite(On_Board_LED, HIGH);

  }// only do if connected to wifi
}
