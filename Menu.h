#include "Arduino.h"
// Throttle settings extern variables for display and selection stuff
extern int MenuLevel;
extern int locoindex;
extern int fnindex;
extern int speedindex;
extern bool directionindex;
extern int LocoNumbers;
extern int functionPage;
//extern int y;
extern uint32_t ButtonPressTimer;
//extern uint32_t LcPropsEnabled;
extern byte ParseIndex;
extern bool AllDataRead;
extern void GetLocoList();
extern void GetLocoFunctions(int index);

extern String FunctionName[(N_Functions + 1)];
extern bool FunctionState[(N_Functions + 1)];
extern bool FunctionStateKnown[(N_Functions + 1)];
extern int FunctionTimer[(N_Functions + 1)];
extern int Loco_V_max;
extern bool UpdatedRotaryMovementUsed;
bool FunctionActive;
extern long ReadADC(int avg);
long analog_value;

#define DISPLAY_WIDTH 128
#define DISPLAY_HEIGHT 64

/*
void DrawBattery(long Voltsx100) {
  int H_shift, V_shift;
  H_shift = 0;
  V_shift = 4;

  display.drawRect(H_shift, V_shift + 0, 17, 6);
  display.fillRect(H_shift + 17, V_shift + 2, 2,  2);
  if (Voltsx100 >= 320) {
    display.fillRect(H_shift + 1, V_shift + 1, 5, 4);
  }
  if (Voltsx100 >= 343) {
    display.fillRect(H_shift + 4, V_shift + 1, 5, 4);
  }
  if (Voltsx100 >= 366) {
    display.fillRect(H_shift + 7, V_shift + 1, 5, 4);
  }
  if (Voltsx100 >= 390) {
    display.fillRect(H_shift + 10, V_shift + 1, 5, 4);
  }

}*/
/*
int getQuality() { //https://stackoverflow.com/questions/15797920/how-to-convert-wifi-signal-strength-from-quality-percent-to-rssi-dbm
  if (WiFi.status() != WL_CONNECTED)
    return -1;
  int dBm = WiFi.RSSI();   // dBm to Quality: Very roughly!!!
  if (dBm <= -100)
    return 0;
  if (dBm >= -50)
    return 100;
  return 2 * (dBm + 100);
}


void SignalStrengthBar( int32_t rssi) { //https://stackoverflow.com/questions/15797920/how-to-convert-wifi-signal-strength-from-quality-percent-to-rssi-dbm
  int PosX, PosY;

  // rssi -90 is just about dropout..
  // rssi -40 is a great signal
  PosX = 114; // position left right  max = 128
  PosY = 0; // top left position up / down max 64

  if (rssi >= -50) {
    display.drawLine(PosX + 10, PosY, PosX + 10, PosY + 10);
  }
  if (rssi >= -55) {
    display.drawLine(PosX + 8, PosY + 0, PosX + 8, PosY + 10);
  }
  if (rssi >= -65) {
    display.drawLine(PosX + 6, PosY + 2, PosX + 6, PosY + 10);
  }
  if (rssi >= -70) {
    display.drawLine(PosX + 4, PosY + 4, PosX + 4, PosY + 10);
  }
  if (rssi >= -80) {
    display.drawLine(PosX + 2, PosY + 6, PosX + 2, PosY + 10);
  }
  display.drawLine(PosX, PosY + 8, PosX, PosY + 10);
}
*/
void drawFunctionBar() {
  int PosX, PosY, Height, Width, i, function;
  PosX = 0; // position left right  max = 128
  PosY = 52; // top left position up / down max 64
  Height = DISPLAY_HEIGHT - PosY;
  Width = DISPLAY_WIDTH / 8;

  // F0/Light is always shown
  if (FunctionState[0])
      display.drawRect(PosX, PosY, Width, Height);
  display.setFont(ArialMT_Plain_10);
  display.drawString(PosX + 9, PosY, "L");
  

  for (i = 1; i < 8; i++) {
    function = i + 7 * functionPage;
    if (FunctionState[function] && function <= N_Functions)
      display.drawRect(PosX + i * (Width), PosY, Width, Height);

    display.setFont(ArialMT_Plain_10);
    if (function >= 10 && function <= N_Functions)
      display.drawString(PosX + (i * (Width)) + 9, PosY, String(function));
    else if (function < N_Functions)
      display.drawString(PosX + (i * (Width)) + 9, PosY, String(function));
  }
}

void drawSpeed(int speed) {
  int H_shift, V_shift;
  H_shift = 65;
  V_shift = 0;
  
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.setFont(ArialMT_Plain_10);
  display.drawString(H_shift, V_shift, String(abs(speed)) + "%");
}

void drawPowerOff() {
  int H_shift, V_shift;
  H_shift = 65;
  V_shift = 0;

  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.setFont(ArialMT_Plain_10);
  display.drawString(H_shift, V_shift, "Power off");
}

void drawArrows() {
  int PosX, PosY, Height, Width;
  PosX = 0; // position left right  max = 128
  PosY = 0; // top left position up / down max 64
  Height = 8;
  Width = 10;

  if (directionindex) {
    PosX = DISPLAY_WIDTH - Width -1;
    display.drawLine(PosX, PosY, PosX + Width, PosY + Height / 2);
    display.drawLine(PosX, PosY + Height, PosX + Width, PosY + Height / 2);
    display.drawVerticalLine(PosX, PosY, Height);
  }
  else {
    display.drawLine(PosX, PosY + Height / 2, PosX + Width, PosY);
    display.drawLine(PosX, PosY + Height / 2, PosX + Width, PosY + Height);
    display.drawVerticalLine(PosX + Width, PosY, Height);
  }
}

void drawImageDemo() {
  // see http://blog.squix.org/2015/05/esp8266-nodemcu-how-to-create-xbm.html
  // on how to create xbm files
  // Go to: http://www.online-utility.org/image_converter.jsp
  //Select MONO as Output format AND press “Select format” (Who has invented THAT UI???)
  //Upload your image. I was succesful with a monochrome PNG file, whereas the XBM only resulted in an error.
  //Upload the created MONO file to your ESP8266 and use it with a code like this:

  // display.drawXbm(20,20, WiFi_Logo_width, WiFi_Logo_height, WiFi_Logo_bits);
  display.drawXbm(0, 0, Terrier_Logo_width, Terrier_Logo_height, Terrier_Logo);

}

extern long ThrottlePosition;
extern bool Encoder_Timeout;

bool LastDir;
int LastLoco;
void SetLoco(int locoindex, int speedindex) {
  bool Dir;
  char MsgTemp[200];

  int SpeedSelected;
  SpeedSelected = speedindex;

  Dir = LastDir;
  LastLoco = locoindex;
  if (speedindex >= 1) {
    Dir = true;
  }
  if (speedindex <= -1) {
    Dir = false; // this set of code tries to ensure that when speed = 0 it uses the last direction set.
  }

  if (Dir) {
    LastDir = true;
    sprintf(MsgTemp, "<lc id=\"%s\"  V=\"%d\" dir=\"true\"  throttleid=\"%s\" />", LOCO_id[locoindex].c_str(), SpeedSelected, NameOfThisThrottle.c_str());
  }
  else {
    LastDir = false;
    sprintf(MsgTemp, "<lc id=\"%s\"  V=\"%d\" dir=\"false\"  throttleid=\"%s\" />", LOCO_id[locoindex].c_str(), abs(SpeedSelected), NameOfThisThrottle.c_str());
  }

  //Serial.print(LOCO_id[locoindex]);
  //Serial.print(" <");Serial.print(MsgTemp);Serial.println(">"); // to help with debug
  MQTTSend("rocrail/service/client", MsgTemp);
}


void Send_function_command(int locoindex, int fnindex, bool state) {
  char MsgTemp[200];

  if (state) {
    sprintf(MsgTemp, "<fn fnchanged=\"%d\" fnchangedstate=\"false\" id=\"%s\" f%d=\"false\" /> ", fnindex, LOCO_id[locoindex].c_str(), fnindex);
  }
  else      {
    sprintf(MsgTemp, "<fn fnchanged=\"%d\" fnchangedstate=\"true\" id=\"%s\" f%d=\"true\" /> ", fnindex, LOCO_id[locoindex].c_str(), fnindex);
  }
  // command is of this general form.. <fn fnchanged="9" fnchangedstate="true" id="Test Board"  f9="true"

  // Serial.print(LOCO_id[locoindex]);Serial.println(" fn set<");Serial.print(MsgTemp);Serial.println(">");
  MQTTSend("rocrail/service/client", MsgTemp);
}

void Do_Function(int locoindex, int fnindex) {
  // initially only f0 is toggle but now checks if timer is not zero
  FunctionState[fnindex] = !FunctionState[fnindex];
  Send_function_command(locoindex, fnindex, !FunctionState[fnindex]);
}

extern void Picture();
extern bool PowerON;
extern uint8_t Volts_Calibration;

void DoDisplay(int MenuLevel) {
  String SpeedIndexString = String(speedindex);
  String FnIndexString = String(fnindex);
  String SpeedSelected;
  String MSGText;
  char MsgTextTime[32];

  extern void BatteryDisplay(int avg);

  display.clear();
//  display.setFont(ArialMT_Plain_10);
  
/*
  // Wifi
  SignalStrengthBar(WiFi.RSSI());
*/

/*
  //Digital Time display
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  if ((hrs == 0) && (mins == 0)) { //not Synchronised yet..
    sprintf(MsgTextTime, "--");
  }
  else {
    sprintf(MsgTextTime, "%02d:%02d:%02d", hrs, mins, secs);
  }
  display.drawString(0, 1, MsgTextTime); // adds time display
*/


/*
  // Battery
  DrawBattery(ReadADC(10));
*/

  // Loco
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  MSGText = "";

  // only display this if we actually have a loco list!
  if (LocoNumbers <= 0) {
    display.setFont(ArialMT_Plain_16);
    display.drawString(64, 10, "No Loco");
    display.drawString(64, 20, "List yet");
  }
  else {
    // functions
    drawFunctionBar();

    // direction
    drawArrows();

    // speed or if power is off
    if (!PowerON) {
      drawPowerOff();
    }
    else
      drawSpeed(speedindex);


      
    
    display.setFont(ArialMT_Plain_16);
    if ( display.getStringWidth(LOCO_id[locoindex]) > DISPLAY_WIDTH) {
      display.setTextAlignment(TEXT_ALIGN_CENTER_BOTH);
      display.drawStringMaxWidth(64, 23, DISPLAY_WIDTH, LOCO_id[locoindex]);
    }
    else {
      display.setFont(ArialMT_Plain_16);
      display.setTextAlignment(TEXT_ALIGN_CENTER_BOTH);
      display.drawString(64, 31, LOCO_id[locoindex]);
    }
    
  }
  display.display();
}


// NEU TODO alle weg!
void DoButtonNextLoco() {
  locoindex = locoindex + 1;
  if (locoindex >= LocoNumbers)
    locoindex = 0;
}

void DoButtonLastLoco() {
  locoindex = locoindex - 1;
  if (locoindex <= -1)
    locoindex = LocoNumbers - 1;
}
/*
void DoButtonForward() {
  Serial.println(">");
}

void DoButtonBackward() {
  Serial.println("<");
}
*/
/*
void DoButtonStop(bool powerOff) {
  if (powerOff) {
    Serial.println("power off");
  }
  else {
    Serial.println("stop");
  }
}
*/
void DoButtonFunction(int function, bool functionOn) {
  Serial.print("function F");
  Serial.print(function);
  Serial.print("=");
  Serial.println(functionOn);
}

void DoButtonFunctionPage(int page) {
  Serial.print("function page=");
  Serial.println(page);
}









byte SW_all[9];
