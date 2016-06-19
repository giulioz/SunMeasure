/*
 * =======================================
 * HELIOMAX
 * Pyrheliometer firmware for Arduino
 * All rights reserved
 * 14/06/2016 Giulio Zausa
 * =======================================
 */

//#define GRAPHICS

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Servo.h>
#ifdef GRAPHICS
  #include <SPI.h>
  #include <GD.h>
  #include "assets.h"
#endif
#include <PetitFS.h>
#include <EEPROM.h>

bool outShift[8]; // Leds and button matrix
byte currentMode = 0;
byte settings, delta, record; // Settings in EEPROM
byte second, minute, hour, month, weekDay, monthDay, year; // Time
int buttons;

void setup()
{
  BootGD();
  
  // Read settings in EEPROM
  settings = EEPROM.read(0);
  delta = EEPROM.read(1);
  record = EEPROM.read(2);

  // Set pins for Shift Registers
  DDRD |= B10001100;
  DDRB = B00000110;
  PORTD |= B00000100;
  PORTB |= B00000010;

  // Initialize Shift Out
  for (int i = 0; i < 8; i++) {
    outShift[i] = 1;
    WriteOutShift();
    outShift[i] = 0;
    delay(50);
  }

  // Init SD
  ReadButtons();
  if (bitRead(buttons, 10)) SensorDebug();
  SetSDPin();
  FATFS fs;
  if (pf_mount(&fs) > 0 || pf_open("REC.SUN") > 0) {
    if (!bitRead(buttons, 5)) {
      SetGDPin();
      #ifdef GRAPHICS
        GD.putstr(1, 11, "SD Error.");
      #endif
      while (true) {
        outShift[5] = true;
        WriteOutShift();
        delay(200);
        outShift[5] = false;
        WriteOutShift();
        delay(200);
      }
    }
  }
  outShift[4] = 1;
  WriteOutShift();

  // Main Cycle
  while (true) {
    switch (currentMode) {
      case 0:
        SetSDPin();
        SensorReading(&fs);
        break;
      case 1:
        CheckSensorsRemove();
        LCDForScreen();
        TV(&fs);
        break;
      case 2:
        CheckSensorsRemove();
        SetSDPin();
        PCConnection(&fs);
        break;
      case 3:
        SetSDPin();
        Util(&fs);
        break;
    }
  }
}

void SensorDebug()
{
  Serial.begin(9600);
  while (true)
  {
    Serial.print(analogRead(2));
    Serial.print(" ");
    Serial.print(analogRead(3));
    Serial.print(" ");
    Serial.print(analogRead(0));
    Serial.print(" ");
    Serial.println(analogRead(1));
  }
}

void SetSDPin()
{
  pinMode(8, OUTPUT);
  digitalWrite(8, LOW);
  pinMode(9, OUTPUT);
  digitalWrite(9, HIGH);
}

void SetGDPin()
{
  pinMode(9, OUTPUT);
  digitalWrite(9, LOW);
  pinMode(8, OUTPUT);
  digitalWrite(8, HIGH);
}

void CheckSensorsRemove()
{
  byte addr[8];
  OneWire ds(9);
  if (ds.search(addr)) {
    LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);
    Wire.begin();
    lcd.begin(16,2);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("Remove sensors"));
    lcd.setCursor(0, 1);
    lcd.print(F("And press REC"));
    while (true) {
      ReadButtons();
      if (bitRead(buttons, 4)) {
        CheckSensorsRemove();
        break;
      }
    }
  }
}

void BootGD()
{
  #ifdef GRAPHICS
  SetGDPin();
  GD.begin();
  GD.ascii();

  Wire.begin();
  GetTime();
  PutByte(1, 1, monthDay, false);
  GD.putstr(3, 1, "/");
  PutByte(4, 1, month, false);
  GD.putstr(6, 1, "/");
  PutByte(7, 1, year, false);
  PutByte(11, 1, hour, false);
  GD.putstr(13, 1, ":");
  PutByte(14, 1, minute, false);
  GD.putstr(16, 1, ":");
  PutByte(17, 1, second, false);
  GD.putstr(1, 2, "Booting...");
  GD.putstr(1, 4, "Heliomax MARK. II");
  GD.putstr(1, 5, "Ver 2.7, Giulio Zausa");
  GD.putstr(1, 7, "Press upper buttons to change mode.");
  GD.putstr(1, 8, "Press SEQ and FILE DATA to view table and graphs.");
  #endif
}

void LCDForScreen()
{
  LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);
  Wire.begin();
  lcd.begin(16,2);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("Graph View (VGA)"));
  lcd.setCursor(0, 1);
  lcd.print(F("+/- Mode"));
}

void loop()
{
  LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);
  Wire.begin();
  lcd.begin(16,2);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("Error!"));
}

// Sensor reading and record mode
void SensorReading(FATFS *fs)
{
  // Load LCD and interface
  LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);
  Wire.begin();
  lcd.begin(16,2);
  lcd.createChar(0, new byte[8]{ 0b01110, 0b01010, 0b01110, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000 }); // Degree char
  lcd.clear();
  bool recPress, stopPress;

  // Load Sensor
  OneWire oneWire(9);
  DallasTemperature sensors(&oneWire);
  sensors.begin();
  sensors.setWaitForConversion(false); // async mode
  sensors.requestTemperatures();
  
  unsigned long lastTempRequest = millis(), lastRecord = 0;
  int delayInMillis = 750 / (1 << (12 - 12));
  unsigned int rDelta = delta * 1000;
  float temperature = 0;
  bool recording = false;

  // Load motors
  float pos = 60;
  Servo servo;
  Servo motor;
  servo.attach(5);
  servo.write(pos);
  
  while (true) {
    ReadButtons();
    if (millis() - lastTempRequest >= delayInMillis) { // Update temperature
      temperature = sensors.getTempCByIndex(0);
      sensors.requestTemperatures(); 
      lastTempRequest = millis(); 
    }
    // Print to LCD
    lcd.setCursor(0, 0);
    lcd.print(F("T: "));
    lcd.print(temperature);
    lcd.write((uint8_t)0);    
    lcd.print(F("C  "));
    
    lcd.setCursor(0, 1);
    lcd.write((char)223);
    lcd.print("= ");
    lcd.print(pos - 60);
    lcd.write((uint8_t)0);
    lcd.print(F("   "));

    // Record
    if (recording) {
      if (millis() - lastRecord >= rDelta) {
        UINT nr;
        byte buff[4];
        *((float *)buff) = pos - 60;
        pf_write(buff, sizeof(buff), &nr);
        UINT bnr;
        *((float *)buff) = temperature;
        pf_write(buff, sizeof(buff), &bnr);
        outShift[4] = false;
        WriteOutShift();
        delay(50);
        outShift[4] = true;
        WriteOutShift();
        delay(50);
        lastRecord = millis();
      }
    }

    // Start Record
    if (bitRead(buttons, 4)) {
      if (!recPress && !recording) {
        GetTime();
        pf_lseek(record * 2073608);
        UINT nr;
        byte buff[7];
        buff[0] = monthDay;
        buff[1] = month;
        buff[2] = year;
        buff[3] = hour;
        buff[4] = minute;
        buff[5] = second;
        buff[6] = delta;
        pf_write(buff, sizeof(buff), &nr);
        outShift[5] = 1;
        WriteOutShift();
        lastRecord = millis();

        recording = true;
        recPress = true;
      }
    }
    else recPress = false;

    // Stop Record
    if (bitRead(buttons, 6)) {
      if (!stopPress && recording) {
        UINT bnr;
        byte end[3] = { 'e', 'n', 'd' };
        pf_write(end, sizeof(end), &bnr);
        pf_write(0, 0, &bnr);
        outShift[5] = 0;
        WriteOutShift();

        recording = false;
        stopPress = true;
      }
    }
    else stopPress = false;
    
    // Move
    if (bitRead(settings, 1)) { // Auto movement
      if (bitRead(buttons, 3) || bitRead(buttons, 2) || bitRead(buttons, 1) || bitRead(buttons, 0)) { // Pause auto movement
        if (bitRead(buttons, 3)) pos += pos < 165 ? 1 : 0;
        else if (bitRead(buttons, 2)) pos -= pos > 5 ? 1 : 0;
        
        if (bitRead(buttons, 1)) { motor.attach(6); motor.write(95); }        // >>
        else if (bitRead(buttons, 0)) { motor.attach(6); motor.write(100); }  // <<
        else motor.detach();
      }
      else {
        int left = analogRead(3);
        int right = analogRead(2);
        int bottom = analogRead(0);
        int top = analogRead(1);
        
        pos += pos < 166 ? (pos > 5 ? (top - bottom) / 200 : 0.5) : -0.5;
        
        // 96 MEDIO
        //float angleTop = (pos - 60) / 180.0;
        float deltal = (left - right);// * abs(angleTop);
        if (deltal > 0.01)  { motor.attach(6); motor.write(98.5); }
        else if (deltal < -0.01)  { motor.attach(6); motor.write(96); }
        //else motor.detach();
      }
    }
    else { // Manual movement
      if (bitRead(buttons, 3)) pos += pos < 165 ? 1 : 0;
      else if (bitRead(buttons, 2)) pos -= pos > 5 ? 1 : 0;
      if (bitRead(buttons, 1)) { motor.attach(6); motor.write(95); }        // >>
        else if (bitRead(buttons, 0)) { motor.attach(6); motor.write(100); }  // <<
        else motor.detach();
    }
    servo.write(pos);
    if (ChangeMode(buttons)) { // Check for change mode buttons
      motor.detach();
      servo.detach();
      return;
    }
  }
}

void PCConnection(FATFS *fs)
{
  LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);
  lcd.begin(16,2);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("USB Dump Mode"));
  lcd.setCursor(0, 1);
  lcd.print(F("Connecting..."));
  Serial.begin(250000);
  
  while (Serial.available() <= 0) {
    ReadButtons();
    if (ChangeMode(buttons)) {
      Serial.end();
      return;
    }
  }
  while (Serial.available() > 0) {
    lcd.setCursor(0, 1);
    lcd.print(F("Transfering..."));
    uint8_t buf[1];
    pf_lseek(record * 2073608);
    bool a1 = false, a2 = false, a3 = false;
    while (!a3) {
      UINT nr;
      pf_read(buf, sizeof(buf), &nr);
      if (!a1 && !a2 && !a3 && buf[0] == 'e') a1 = true;
      if (a1 && !a2 && !a3 && buf[0] == 'n') a2 = true;
      if (a1 && a2 && !a3 && buf[0] == 'd') a3 = true;
      if (nr == 0) break;
      Serial.println(String(buf[0], 16));
    }
    Serial.print("end");
    lcd.setCursor(0, 1);
    lcd.print(F("Completed.    "));
    while (!ChangeMode(buttons)) {
      ReadButtons();
    }
    Serial.end();
    return;
  }
}

void Util(FATFS *fs)
{
  LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);
  lcd.begin(16,2);
  lcd.createChar(0, new byte[8]{ 0b01110, 0b01010, 0b01110, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000 }); // Degree char
  lcd.clear();
  byte state = 0; // 0:Record 1:Delta 2:Backlight 3:AutoMove 4:Date 5:Time 6:Second 7:Minute 8:Hour 9:WeekDay 10:MonthDay 11:Month 12:Year 13:SaveTime 14:About
  bool movePressed1, movePressed2, gPress1, gPress2;

  while (true) {
    ReadButtons();
    if (bitRead(buttons, 0)) {
      if (!movePressed1) {
        if (state != 0) {
          state--;
          lcd.clear();
        }
        movePressed1 = true;
      }
    }
    else movePressed1 = false;
    if (bitRead(buttons, 1)) {
      if (!movePressed2) {
        if (state != 14) {
          state++;
          lcd.clear();
        }
        movePressed2 = true;
      }
    }
    else movePressed2 = false;
    
    lcd.setCursor(0, 0);
    switch (state) {
      case 0:
        lcd.print(F("</> Record:"));
        lcd.setCursor(0, 1);
        lcd.print(F("+/- "));
        lcd.print(record);
        lcd.print(F("   "));
        if (bitRead(buttons, 2)) {
          if (!gPress1) {
            if (record != 0) record--;
            gPress1 = true;
          }
        }
        else gPress1 = false;
        if (bitRead(buttons, 3)) {
          if (!gPress2) {
            if (record != 2) record++;
            gPress2 = true;
          }
        }
        else gPress2 = false;
        break;
      case 1:
        lcd.print(F("</> Delta T:"));
        lcd.setCursor(0, 1);
        lcd.print(F("+/- "));
        lcd.print(delta);
        lcd.print(F(" sec  "));
        if (bitRead(buttons, 2)) {
          if (!gPress1) {
            if (delta != 1) delta--;
            gPress1 = true;
          }
        }
        else gPress1 = false;
        if (bitRead(buttons, 3)) {
          if (!gPress2) {
            if (delta != 255) delta++;
            gPress2 = true;
          }
        }
        else gPress2 = false;
        break;
      case 2:
        lcd.print(F("</> Backlight:"));
        lcd.setCursor(0, 1);
        lcd.print(F("+/- On/Off "));
        if (bitRead(buttons, 3)) lcd.backlight();
        else if (bitRead(buttons, 2)) lcd.noBacklight();
        break;
      case 3:
        lcd.print(F("</> Auto Move:"));
        lcd.setCursor(0, 1);
        lcd.print(bitRead(settings, 1) ? "+/- On " : "+/- Off");
        if (bitRead(buttons, 2)) bitWrite(settings, 1, 0);
        else if (bitRead(buttons, 3)) bitWrite(settings, 1, 1);
        break;
      case 4:
        GetTime();
        lcd.print(F("</> Date:"));
        lcd.setCursor(0, 1);
        lcd.print(monthDay);
        lcd.print("/");
        lcd.print(month);
        lcd.print("/");
        lcd.print(year);
        break;
      case 5:
        GetTime();
        lcd.print(F("</> Time:"));
        lcd.setCursor(0, 1);
        lcd.print(hour);
        lcd.print(":");
        lcd.print(minute);
        lcd.print(":");
        lcd.print(second);
        break;
      case 6:
        lcd.print(F("</> Second:"));
        lcd.setCursor(0, 1);
        lcd.print(F("+/- "));
        lcd.print(second);
        lcd.print(F("   "));
        if (bitRead(buttons, 2)) {
          if (!gPress1) {
            if (second != 0) second--;
            gPress1 = true;
          }
        }
        else gPress1 = false;
        if (bitRead(buttons, 3)) {
          if (!gPress2) {
            if (second != 59) second++;
            gPress2 = true;
          }
        }
        else gPress2 = false;
        break;
      case 7:
        lcd.print(F("</> Minute:"));
        lcd.setCursor(0, 1);
        lcd.print(F("+/- "));
        lcd.print(minute);
        lcd.print(F("   "));
        if (bitRead(buttons, 2)) {
          if (!gPress1) {
            if (minute != 0) minute--;
            gPress1 = true;
          }
        }
        else gPress1 = false;
        if (bitRead(buttons, 3)) {
          if (!gPress2) {
            if (minute != 59) minute++;
            gPress2 = true;
          }
        }
        else gPress2 = false;
        break;
      case 8:
        lcd.print(F("</> Hour:"));
        lcd.setCursor(0, 1);
        lcd.print(F("+/- "));
        lcd.print(hour);
        lcd.print(F("   "));
        if (bitRead(buttons, 2)) {
          if (!gPress1) {
            if (hour != 0) hour--;
            gPress1 = true;
          }
        }
        else gPress1 = false;
        if (bitRead(buttons, 3)) {
          if (!gPress2) {
            if (hour != 23) hour++;
            gPress2 = true;
          }
        }
        else gPress2 = false;
        break;
      case 9:
        lcd.print(F("</> Week Day:"));
        lcd.setCursor(0, 1);
        lcd.print(F("+/- "));
        lcd.print(weekDay);
        lcd.print(F("   "));
        if (bitRead(buttons, 2)) {
          if (!gPress1) {
            if (weekDay != 0) weekDay--;
            gPress1 = true;
          }
        }
        else gPress1 = false;
        if (bitRead(buttons, 3)) {
          if (!gPress2) {
            if (weekDay != 7) weekDay++;
            gPress2 = true;
          }
        }
        else gPress2 = false;
        break;
      case 10:
        lcd.print(F("</> Month Day:"));
        lcd.setCursor(0, 1);
        lcd.print(F("+/- "));
        lcd.print(monthDay);
        lcd.print(F("   "));
        if (bitRead(buttons, 0)) {
          if (!gPress1) {
            if (monthDay != 2) monthDay--;
            gPress1 = true;
          }
        }
        else gPress1 = false;
        if (bitRead(buttons, 3)) {
          if (!gPress2) {
            if (monthDay != 32) monthDay++;
            gPress2 = true;
          }
        }
        else gPress2 = false;
        break;
      case 11:
        lcd.print(F("</> Month:"));
        lcd.setCursor(0, 1);
        lcd.print(F("+/- "));
        lcd.print(month);
        lcd.print(F("   "));
        if (bitRead(buttons, 2)) {
          if (!gPress1) {
            if (month != 0) month--;
            gPress1 = true;
          }
        }
        else gPress1 = false;
        if (bitRead(buttons, 3)) {
          if (!gPress2) {
            if (month != 12) month++;
            gPress2 = true;
          }
        }
        else gPress2 = false;
        break;
      case 12:
        lcd.print(F("</> Year:"));
        lcd.setCursor(0, 1);
        lcd.print(F("+/- 20"));
        lcd.print(year);
        lcd.print(F("   "));
        if (bitRead(buttons, 2)) {
          if (!gPress1) {
            if (year != 0) year--;
            gPress1 = true;
          }
        }
        else gPress1 = false;
        if (bitRead(buttons, 3)) {
          if (!gPress2) {
            year++;
            gPress2 = true;
          }
        }
        else gPress2 = false;
        break;
      case 13:
        lcd.print(F("</> SaveTime"));
        lcd.setCursor(0, 1);
        lcd.print(F("Save: +"));
        if (bitRead(buttons, 1)) SetTime();
        break;
      case 14:
        lcd.print(F("Heliomax ROM 2.7"));
        lcd.setCursor(0, 1);
        lcd.print(F("14/06/16 G.Zausa"));
        break;
    }
    
    if (ChangeMode(buttons)) {
      EEPROM.write(0, settings);
      EEPROM.write(1, delta);
      EEPROM.write(2, record);
      return;
    }
  }
}

void GetTime()
{
  Wire.beginTransmission(0x68);
  byte zero = 0x00;
  Wire.write(zero);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 7);

  second = bcdToDec(Wire.read());
  minute = bcdToDec(Wire.read());
  hour = bcdToDec(Wire.read() & 0b111111); //24 hour time
  weekDay = bcdToDec(Wire.read()); //0-6 -> sunday - Saturday
  monthDay = bcdToDec(Wire.read());
  month = bcdToDec(Wire.read());
  year = bcdToDec(Wire.read());
}

void SetTime()
{
  Wire.beginTransmission(0x68);
  byte zero = 0x00;
  Wire.write(zero);
  Wire.write(decToBcd(second));
  Wire.write(decToBcd(minute));
  Wire.write(decToBcd(hour));
  Wire.write(decToBcd(weekDay));
  Wire.write(decToBcd(monthDay));
  Wire.write(decToBcd(month));
  Wire.write(decToBcd(year));
  Wire.write(zero);
  Wire.endTransmission();
}

bool ChangeMode(int buttons)
{
  if (bitRead(buttons, 7)) { ResetModeLed(); outShift[4] = 1; WriteOutShift(); currentMode = 0; return true; }
  else if (bitRead(buttons, 8)) { ResetModeLed(); outShift[3] = 1; WriteOutShift(); currentMode = 1; return true; }
  else if (bitRead(buttons, 9)) { ResetModeLed(); outShift[2] = 1; WriteOutShift(); currentMode = 2; return true; }
  else if (bitRead(buttons, 10)) { ResetModeLed(); outShift[1] = 1; WriteOutShift(); currentMode = 3; return true; }
  return false;
}

void ResetModeLed()
{
  outShift[2] = 0;
  outShift[3] = 0;
  outShift[4] = 0;
  outShift[1] = 0;
}

int ReadButtons()
{
  byte data;
  outShift[7] = 0;
  outShift[6] = 1;
  WriteOutShift();
  digitalWrite(2, LOW);
  digitalWrite(2, HIGH);
  data = shiftIn(4, 3, LSBFIRST);
  //limit = !bitRead(data, 5);
  bitWrite(buttons, 1, bitRead(data, 0)); // >>
  bitWrite(buttons, 2, bitRead(data, 1)); // -
  bitWrite(buttons, 3, bitRead(data, 2)); // +
  bitWrite(buttons, 4, bitRead(data, 3)); // rec
  bitWrite(buttons, 5, bitRead(data, 4)); // pause
  bitWrite(buttons, 0, bitRead(data, 6)); // <<
  
  outShift[7] = 1;
  outShift[6] = 0;
  WriteOutShift();
  digitalWrite(2, LOW);
  digitalWrite(2, HIGH);
  data = shiftIn(4, 3, LSBFIRST);
  bitWrite(buttons, 10, bitRead(data, 0)); // util
  bitWrite(buttons, 9, bitRead(data, 1)); // job
  bitWrite(buttons, 8, bitRead(data, 2)); // seq
  bitWrite(buttons, 7, bitRead(data, 4)); // mdr
  bitWrite(buttons, 6, bitRead(data, 6)); // start / stop

  return buttons;
}

void WriteOutShift()
{
  for (int i = 0; i < 8; i++) {
    digitalWrite(10, LOW);
    digitalWrite(7, outShift[i]);
    digitalWrite(10, HIGH);
  }
}

byte bcdToDec(byte val)
{
  return ((val / 16 * 10) + (val % 16));
}

byte decToBcd(byte val)
{
  return ((val / 10 * 16) + (val % 10));
}

#ifdef GRAPHICS

// replicate a 2-bit color across the whole byte.
byte replicate(byte color)
{
  return (color << 6) | (color << 4) | (color << 2) | color;
}

int BLACK;
#define WHITE RGB(255,255,255)

class PlotterClass
{
public:
  void begin();
  void line(byte x0, byte y0, byte x1, byte y1);
  void show();
  void end();
private:
  byte flip;
  byte plotting;
  void erase();
  void waitready();
};

PlotterClass Plotter;

void PlotterClass::waitready()
{
  while (GD.rd(COMM+7))
    ;
}

void PlotterClass::erase()
{
  byte color = flip ? 1 : 2;

  plotting = 0;
  GD.wr(J1_RESET, 1);
  GD.wr(COMM+7, 1);
  GD.wr(COMM+8, replicate(color ^ 3));
  GD.microcode(eraser_code, sizeof(eraser_code));
}

void PlotterClass::begin()
{
  // Draw 256 sprites left to right, top to bottom, all in 4-color
  // palette mode.  By doing them in column-wise order, the address
  // calculation in setpixel is made simpler.
  // First 64 use bits 0-1, next 64 use bits 2-4, etc.
  // This gives a 256 x 256 4-color bitmap.

  unsigned int i;
  for (i = 0; i < 256; i++) {
    int x =     72 + 16 * ((i >> 4) & 15);
    int y =     30 + 16 * (i & 15);// + 22;
    int image = i & 63;     /* image 0-63 */
    int pal =   3 - (i >> 6);   /* palettes bits in columns 3,2,1,0 */
    GD.sprite(i, x, y, image, 0x8 | (pal << 1), 0);
  }

  flip = 0;
  plotting = 0;
  erase();
  show();
}

void PlotterClass::show()
{
  waitready();
  if (flip == 1) {
    GD.wr16(PALETTE4A, BLACK);
    GD.wr16(PALETTE4A + 2, WHITE);
    GD.wr16(PALETTE4A + 4, BLACK);
    GD.wr16(PALETTE4A + 6, WHITE);
  } else {
    GD.wr16(PALETTE4A, BLACK);
    GD.wr16(PALETTE4A + 2, BLACK);
    GD.wr16(PALETTE4A + 4, WHITE);
    GD.wr16(PALETTE4A + 6, WHITE);
  }
  flip ^= 1;
  erase();
}

void PlotterClass::line(byte x0, byte y0, byte x1, byte y1)
{
  byte swap;
#define SWAP(a, b) (swap = (a), (a) = (b), (b) = swap)

  byte steep = abs(y1 - y0) > abs(x1 - x0);
  if (steep) {
    SWAP(x0, y0);
    SWAP(x1, y1);
  }
  if (x0 > x1) {
    SWAP(x0, x1);
    SWAP(y0, y1);
  }
  int deltax = x1 - x0;
  int deltay = abs(y1 - y0);
  int error = deltax / 2;
  char ystep;
  if (y0 < y1)
    ystep = 1;
  else
    ystep = -1;
  byte x;
  byte y = y0;

  waitready();
  if (!plotting) {
    GD.microcode(wireframe_code, sizeof(wireframe_code));
    plotting = 1;
    byte color = flip ? 1 : 2;
    GD.wr(COMM+8, color << 6);
  }
  GD.__wstart(COMM+0);
  SPI.transfer(x0);
  SPI.transfer(y0);
  SPI.transfer(x1);
  SPI.transfer(y1);
  SPI.transfer(steep);
  SPI.transfer(deltax);
  SPI.transfer(deltay);
  SPI.transfer(ystep);
  GD.__end();
}

unsigned int countSDVals(FATFS *fs)
{
  SetSDPin();
  unsigned int c = 0;
  uint8_t buf[1];
  pf_lseek(record * 2073608 + 7);
  bool a1 = false, a2 = false, a3 = false;
  while (!a3) {
    UINT anr;
    pf_read(buf, sizeof(buf), &anr);
    if (!a1 && !a2 && !a3 && buf[0] == 'e') a1 = true;
    if (a1 && !a2 && !a3 && buf[0] == 'n') a2 = true;
    if (a1 && a2 && !a3 && buf[0] == 'd') a3 = true;
    if (anr == 0) break;
    c++;
  }
  c--;
  SetGDPin();
  return c;
}

void GAngleRead(byte *avals, unsigned int cvals)
{
  SetSDPin();
  byte lastI = 0;
  pf_lseek(7 + record * 2073608);
  int d = cvals / 240;
  for (unsigned int i = 0; i < cvals; i += d) {
    pf_lseek(7 + record * 2073608 + i * 8);
    byte index = map(i, 0, cvals - 1, 0, 240);
    UINT anr, bnr;
    uint8_t buf[4];
    pf_read(buf, sizeof(buf), &anr);
    float aval = *(float *)&buf;
    avals[index] = (byte)((aval - 105.0) * 2.1818);
    if (index - lastI > 1) {
      for (byte j = 0; j < index - lastI; j++) {
        avals[lastI + j] = lerp((float)j / (float)(index - lastI), avals[lastI], avals[index]);
      }
    }
    pf_read(buf, sizeof(buf), &bnr);
    lastI = index;
  }
  SetGDPin();
}

void GTempRead(byte *tvals, unsigned int cvals)
{
  SetSDPin();
  byte lastI = 0;
  pf_lseek(7 + record * 2073608);
  int d = cvals / 240;
  for (unsigned int i = 0; i < cvals; i += d) {
    pf_lseek(7 + record * 2073608 + i * 8);
    byte index = map(i, 0, cvals - 1, 0, 240);
    UINT anr, bnr;
    uint8_t buf[4];
    pf_read(buf, sizeof(buf), &anr);
    pf_read(buf, sizeof(buf), &bnr);
    float tval = *(float *)&buf;
    tvals[index] = (byte)(tval * 3.4285);
    if (index - lastI > 1) {
      for (byte j = 0; j < index - lastI; j++) {
        tvals[lastI + j] = lerp((float)j / (float)(index - lastI), tvals[lastI], tvals[index]);
      }
    }
    lastI = index;
  }
  SetGDPin();
}

byte lerp(float t, byte a, byte b)
{
  return (byte)((1 - t) * a + t * b);
}

void ReDrawGraph(byte mode, unsigned int cvals)
{
  byte avals[240], tvals[240];
  switch(mode) {
    case 1:
      GD.putstr(21, 2, "Plot C/T");
      GTempRead(tvals, cvals);
      for (byte i = 0; i < 239; i++) {
        Plotter.line(16 + i, -tvals[i], 17 + i, -tvals[i + 1]);
      }
      Plotter.line(16, 240, 250, 240); // X Axis
      Plotter.line(250, 240, 245, 235);
      Plotter.line(250, 240, 245, 245);
      Plotter.line(16, 3, 16, 240); // Y Axis
      Plotter.line(16, 3, 11, 8);
      Plotter.line(16, 3, 21, 8);    
      Plotter.show();
      break;
    case 2:
      GD.putstr(21, 2, "Plot A/T");
      GAngleRead(avals, cvals);
      for (byte i = 0; i < 239; i++) {
        Plotter.line(16 + i, avals[i], 17 + i, avals[i + 1]);
      }
      Plotter.line(16, 240, 250, 240); // X Axis
      Plotter.line(250, 240, 245, 235);
      Plotter.line(250, 240, 245, 245);
      Plotter.line(16, 3, 16, 240); // Y Axis
      Plotter.line(16, 3, 11, 8);
      Plotter.line(16, 3, 21, 8);    
      Plotter.show();
      break;
    case 3:
      GD.putstr(21, 2, "Plot C/A");
      GTempRead(tvals, cvals);
      GAngleRead(avals, cvals);
      for (byte i = 0; i < 239; i++) {
        Plotter.line(16 + avals[i], -tvals[i], 17 + avals[i + 1], -tvals[i + 1]);
      }
      Plotter.line(16, 240, 250, 240); // X Axis
      Plotter.line(250, 240, 245, 235);
      Plotter.line(250, 240, 245, 245);
      Plotter.line(16, 3, 16, 240); // Y Axis
      Plotter.line(16, 3, 11, 8);
      Plotter.line(16, 3, 21, 8);    
      Plotter.show();
      break;
  }
}

void ReDrawTable(unsigned int cvals, unsigned int tableOffset)
{
  BLACK = TRANSPARENT;
  GD.putstr(21, 2, "  Table  ");
  GD.putstr(5, 4, "time");
  GD.putstr(12, 4, "angle");
  GD.putstr(19, 4, "temp");
  Plotter.show();
  
  uint8_t buf[4];
  for (byte i = 0; i < 30; i++) {
    SetSDPin();
    pf_lseek(7 + record * 2073608 + 8 * (i + tableOffset));
    UINT anr, bnr;
    pf_read(buf, sizeof(buf), &anr);
    float aval = *(float *)&buf;
    pf_read(buf, sizeof(buf), &bnr);
    float tval = *(float *)&buf;
    SetGDPin();
    PutInt(5, i + 6, i + tableOffset);
    PutByte(13, i + 6, (byte)aval, false);
    PutFloat(19, i + 6, tval);
  }
}

void ReDraw(byte mode, unsigned int cvals, unsigned int tableOffset)
{
  switch(mode) {
    case 1:
      ReDrawGraph(mode, cvals);
      break;
    case 2:
      ReDrawGraph(mode, cvals);
      break;
    case 3:
      ReDrawGraph(mode, cvals);
      break;
    case 0:
      ReDrawTable(cvals, tableOffset);
      break;
  }
}

void PutByte(int x, int y, byte b, bool first)
{
  GD.__wstart((y << 6) + x);
  if (first) SPI.transfer('0' + b / 100);
  SPI.transfer('0' + b % 100 / 10);
  SPI.transfer('0' + b % 10);
  GD.__end();
}

void PutFloat(int x, int y, float b)
{
  GD.__wstart((y << 6) + x);
  SPI.transfer('0' + (byte)b / 10);
  SPI.transfer('0' + (byte)b % 10);
  SPI.transfer('.');
  SPI.transfer('0' + (byte)(b * 10) % 10);
  SPI.transfer('0' + (byte)(b * 100) % 10);
  GD.__end();
}

void PutInt(int x, int y, unsigned int b)
{
  GD.__wstart((y << 6) + x);
  SPI.transfer('0' + b / 10000);
  SPI.transfer('0' + b % 1000 / 100);
  SPI.transfer('0' + b % 100 / 10);
  SPI.transfer('0' + b % 10);
  GD.__end();
}

#define OFFSET 7
void PrintHeader(FATFS *fs)
{
  GD.wr16(BG_COLOR, RGB(0, 0, 64));
  pf_lseek(record * 2073608);
  UINT anr;
  uint8_t buf[7];
  pf_read(buf, sizeof(buf), &anr);

  GD.putstr(OFFSET, 1, "Record: 0");
  GD.__wstart((1 << 6) + OFFSET + 9);
  SPI.transfer('0' + record);
  GD.__end();
  GD.putstr(OFFSET + 11, 1, "dt:");
  PutByte(OFFSET + 15, 1, buf[6], true);
  PutByte(OFFSET + 19, 1, buf[0], false);
  GD.putstr(OFFSET + 21, 1, "/");
  PutByte(OFFSET + 22, 1, buf[1], false);
  GD.putstr(OFFSET + 24, 1, "/");
  PutByte(OFFSET + 25, 1, buf[2], false);
  PutByte(OFFSET + 28, 1, buf[3], false);
  GD.putstr(OFFSET + 30, 1, ":");
  PutByte(OFFSET + 31, 1, buf[4], false);
  GD.putstr(OFFSET + 33, 1, ":");
  PutByte(OFFSET + 34, 1, buf[5], false);
}
#endif

void TV(FATFS *fs)
{
  #ifdef GRAPHICS
  pinMode(8, OUTPUT);
  digitalWrite(8, HIGH);
  GD.begin();
  GD.ascii();
  byte plotMode = 0; // 0:C/t 1:A/t 2:C/A 3:Table
  bool gPress1, gPress2, gPress3, gPress4;
  PrintHeader(fs);
  BLACK = TRANSPARENT;
  Plotter.begin();
  unsigned int tOff = 0;
  unsigned int cVals = countSDVals(fs);
  ReDraw(plotMode, cVals, tOff);

  while (true) {
    ReadButtons();
    if (bitRead(buttons, 2)) {
      if (!gPress1) {
        if (plotMode != 0) { plotMode--; ReDraw(plotMode, cVals, tOff); }
        gPress1 = true;
      }
    }
    else gPress1 = false;
    if (bitRead(buttons, 3)) {
      if (!gPress2) {
        if (plotMode == 0) {
          GD.fill(0, 0, 0xFFF);
          GD.ascii();
          BLACK = RGB(0,0,0);
          GD.wr16(BG_COLOR, RGB(0, 0, 64));
          PrintHeader(fs);
          Plotter.begin();
        }
        if (plotMode != 3) { plotMode++; ReDraw(plotMode, cVals, tOff); }
        gPress2 = true;
      }
    }
    else gPress2 = false;
    
    if (bitRead(buttons, 0)) {
      if (!gPress3) {
        if (plotMode == 0 && tOff != 0) tOff--;
        gPress3 = true;
        PrintHeader(fs);
        ReDraw(plotMode, cVals, tOff);
      }
    }
    else gPress3 = false;
    if (bitRead(buttons, 1)) {
      if (!gPress4) {
        if (plotMode == 0 && tOff < cVals - 1) tOff++;
        gPress4 = true;
        PrintHeader(fs);
        ReDraw(plotMode, cVals, tOff);
      }
    }
    else gPress4 = false;
    
    if (ChangeMode(buttons)) {
      return;
    }
  }
  #else
  while (true) {
    ReadButtons();
    if (ChangeMode(buttons)) {
      return;
    }
  }
  #endif
}
