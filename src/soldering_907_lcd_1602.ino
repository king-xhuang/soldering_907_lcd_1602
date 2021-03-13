/*
 * LCD 1602 version
 * Soldering IRON controller for hakko 907 soldering IRON using interrupts from the Timer1 to check the temperature
 * The IRON heater is managed by pin D10 with FastPWM function using Timer1
 * Timer1 runs with prescale 1 through 0 to 255 and back, switching the D10 pin each time
 * The PWM frequency on the pin D10 is 31250 Hz
 * Timer1 generates also the overflow interrupts at 31250 Hz.
 * Overflow interrupts are using to check the IRON temperature
 * First, the IRON is powered off and the controller waits for 32 timer interrupts (about 1 ms)
 * then the current IRON temperature is checked and the controller waits for check_period Timer1 interrupts
 * to restart the all procedure over again
 * 
 * This is a modified version of project from  https://github.com/sfrwmaker/soldering_907_lcd/tree/master/lcd_1602
 * The original code is in ../original
 */
 
#include <LiquidCrystal.h>
#include <EEPROM.h>

#include <Wire.h>
#include <Adafruit_INA219.h>



Adafruit_INA219 ina219;
// pin implicitly used by I2C libiary
const byte colck_pin = A5;   // wired to INA219 SCL
const byte data_pin = A4;    // wired to INA219 SDA

// The LCD 1602 parallel interface
const byte LCD_RS_PIN     = 13;
const byte LCD_E_PIN      = 12;
const byte LCD_DB4_PIN    = 5;
const byte LCD_DB5_PIN    = 6;
const byte LCD_DB6_PIN    = 7;
const byte LCD_DB7_PIN    = 8;

// Rotary encoder interface
const byte R_MAIN_PIN = 2;                      // Rotary Encoder main pin (right)
const byte R_SECD_PIN = 4;                      // Rotary Encoder second pin (left)
const byte R_BUTN_PIN = 3;                      // Rotary Encoder push button pin

const byte probePIN  = A0;                      // NOT used // Thermometer pin from soldering iron  
const byte paddlePIN = 9;                       // paddle pin
const byte heaterPIN = 10;                      // The soldering iron heater pin
const byte buzzerPIN = 11;                      // The simple buzzer to make a noise


const uint16_t temp_minC = 180;                 // Minimum temperature in degrees of celsius
const uint16_t temp_maxC = 400;                 // Maximum temperature in degrees of celsius
const uint16_t temp_minF = (temp_minC *9 + 32*5 + 2)/5;
const uint16_t temp_maxF = (temp_maxC *9 + 32*5 + 2)/5;
const uint16_t temp_tip[3] = {200, 300, 400};

//
volatile bool paddleDown = false;
volatile bool fullPower = false;
uint32_t paddleDown_start_ms = 0;
uint32_t paddleDown_time_ms = 0;

byte  warm_timeout = 2;                  // from heating temp switch to warming temp timeout in minutes [1 - 3]
byte  off_timeout = 10;                   // from WARMING temp switch to OFF timeout in minutes [5-30]
// The variables for Timer1 operations
volatile uint16_t  tmr1_count;                  // The count to calculate the temperature check period
volatile bool      iron_off;                    // Whether the IRON is switched off to check the temperature
const uint32_t     check_period = 100;          // The IRON temperature check period, ms
//
float sensorMvf = - 1.0;
 

//globle functions
byte adcType = 1; // INA219 + K type thermal
int16_t normalize(uint16_t t){ // normalize the sensor reading into 10 bits range
    if (adcType == 1){
        return t >> 2; // devided by 4  
    }
    else return t;//TODO
}

void initADC() { // initialize ADC
  Serial.println("initialize INA219");  
  // initialize serial communication at 9600 bits per second: 
 
  ina219.begin();  // Initialize first board (default address 0x40)
  //ina219.setCalibration_NO_BUS_320mv ();  // use Vin pins to measure voltage without shunt resistor
  ina219.setCalibration_NO_BUS_40mv(); 
}   

int16_t readSensorRaw(){ // read sensor through INA219 adc converter, LSB = 10uV
  return ina219.getShuntVoltage_raw();
}

float convertSensorRaw2MV(int16_t sensorRaw){   
  return  sensorRaw * 0.01;
}
 

void dpint(String s, int16_t t){
  Serial.print(s);Serial.println(t);
}
void dpuint(String s, uint16_t t){
  Serial.print(s);Serial.println(t);
}
void dplong(String s, long t){
  Serial.print(s);Serial.println(t);
}
void dpB(String s, boolean b){
  Serial.print(s);Serial.println(b);
}

void checkPaddle(){
  bool l  = digitalRead(paddlePIN);
  if ( l == LOW ){
    if (paddleDown == false){
      paddleDown_start_ms = millis();
      
    } 
    paddleDown = true;
  } else {
    paddleDown = false;
    paddleDown_start_ms = 0;
  }
}

// uint16_t  intmap(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max)
// {
//   dpuint("x - in_min ", x - in_min);
//   dpuint("out_max - out_min ", out_max - out_min);
//   dpuint("in_max - in_min ", in_max - in_min);
//   dpuint("out_min ", out_min);

  
//   return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
// }
//------------------------------------------ Configuration data ------------------------------------------------
/* Config record in the EEPROM has the following format:
  uint32_t ID                           each time increment by 1
  struct cfg                            config data, 8 bytes
  byte CRC                              the checksum
*/
struct cfg {
  uint32_t calibration;                         // The temperature calibration data for soldering IRON. (3 reference points: 200, 300, 400 Centegrees)
  uint16_t temp;                                // The preset temperature of the IRON in the internal units
  byte     off_timeout;                         // The Automatic switch-off timeout in minutes [0 - 30]
  bool     celsius;                             // Temperature units: true - Celsius, false - Farenheit
  
};

class CONFIG {
  public:
    CONFIG() {
      can_write     = false;
      buffRecords   = 0;
      rAddr = wAddr = 0;
      eLength       = 0;
      nextRecID     = 0;
      byte rs = sizeof(struct cfg) + 5;               // The total config record size
      // Select appropriate record size; The record size should be power of 2, i.e. 8, 16, 32, 64, ... bytes
      for (record_size = 8; record_size < rs; record_size <<= 1);
    }
    void init();
    bool load(void);
    void getConfig(struct cfg &Cfg);                  // Copy config structure from this class
    void updateConfig(struct cfg &Cfg);               // Copy updated config into this class
    bool save(void);                                  // Save current config copy to the EEPROM
    bool saveConfig(struct cfg &Cfg);                 // write updated config into the EEPROM
    void clearAll(void);                              // Clean all EEPROM

  protected:
    struct   cfg Config;

  private:
    bool     readRecord(uint16_t addr, uint32_t &recID);
    bool     can_write;                               // The flag indicates that data can be saved
    byte     buffRecords;                             // Number of the records in the outpt buffer
    uint16_t rAddr;                                   // Address of thecorrect record in EEPROM to be read
    uint16_t wAddr;                                   // Address in the EEPROM to start write new record
    uint16_t eLength;                                 // Length of the EEPROM, depends on arduino model
    uint32_t nextRecID;                               // next record ID
    byte     record_size;                             // The size of one record in bytes
};


 // Read the records until the last one, point wAddr (write address) after the last record
void CONFIG::init(void) {
  eLength = EEPROM.length();
  uint32_t recID;
  uint32_t minRecID = 0xffffffff;
  uint16_t minRecAddr = 0;
  uint32_t maxRecID = 0;
  uint16_t maxRecAddr = 0;
  byte records = 0;

  nextRecID = 0;

  // read all the records in the EEPROM find min and max record ID
  for (uint16_t addr = 0; addr < eLength; addr += record_size) {
    if (readRecord(addr, recID)) {
      ++records;
      if (minRecID > recID) {
        minRecID = recID;
        minRecAddr = addr;
      }
      if (maxRecID < recID) {
        maxRecID = recID;
        maxRecAddr = addr;
      }
    } else {
      break;
    }
  }

  if (records == 0) {
    wAddr = rAddr = 0;
    can_write = true;
    return;
  }

  rAddr = maxRecAddr;
  if (records < (eLength / record_size)) {            // The EEPROM is not full
    wAddr = rAddr + record_size;
    if (wAddr > eLength) wAddr = 0;
  } else {
    wAddr = minRecAddr;
  }
  can_write = true;
}

void CONFIG::getConfig(struct cfg &Cfg) {
  memcpy(&Cfg, &Config, sizeof(struct cfg));
}

void CONFIG::updateConfig(struct cfg &Cfg) {
  memcpy(&Config, &Cfg, sizeof(struct cfg));
}

bool CONFIG::saveConfig(struct cfg &Cfg) {
  updateConfig(Cfg);
  return save();                                      // Save new data into the EEPROM
}

bool CONFIG::save(void) {
  if (!can_write) return can_write;
  if (nextRecID == 0) nextRecID = 1;

  uint16_t startWrite = wAddr;
  uint32_t nxt = nextRecID;
  byte summ = 0;
  for (byte i = 0; i < 4; ++i) {
    EEPROM.write(startWrite++, nxt & 0xff);
    summ <<=2; summ += nxt;
    nxt >>= 8;
  }
  byte* p = (byte *)&Config;
  for (byte i = 0; i < sizeof(struct cfg); ++i) {
    summ <<= 2; summ += p[i];
    EEPROM.write(startWrite++, p[i]);
  }
  summ ++;                                            // To avoid empty records
  EEPROM.write(wAddr+record_size-1, summ);

  rAddr = wAddr;
  wAddr += record_size;
  if (wAddr > EEPROM.length()) wAddr = 0;
  nextRecID ++;                                       // Get ready to write next record
  return true;
}

bool CONFIG::load(void) {
  bool is_valid = readRecord(rAddr, nextRecID);
  nextRecID ++;
  dpB("CONFIG::load success ", is_valid);
  return is_valid;
}

bool CONFIG::readRecord(uint16_t addr, uint32_t &recID) {
  byte Buff[record_size];

  for (byte i = 0; i < record_size; ++i) 
    Buff[i] = EEPROM.read(addr+i);
  
  byte summ = 0;
  for (byte i = 0; i < sizeof(struct cfg) + 4; ++i) {

    summ <<= 2; summ += Buff[i];
  }
  summ ++;                                            // To avoid empty fields
  if (summ == Buff[record_size-1]) {                  // Checksumm is correct
    uint32_t ts = 0;
    for (char i = 3; i >= 0; --i) {
      ts <<= 8;
      ts |= Buff[byte(i)];
    }
    recID = ts;
    memcpy(&Config, &Buff[4], sizeof(struct cfg));
    return true;
  }
  return false;
}

void CONFIG::clearAll(void) {
  for (int i = 0; i < eLength; ++i)
    EEPROM.write(i, 0);
  init();
  load();
}

//------------------------------------------ class IRON CONFIG -------------------------------------------------
class IRON_CFG : public CONFIG {
  public:
    IRON_CFG()                                  { current_tip = 0; t_tip[0] = t_tip[1] = t_tip[2] = 0; is_calibrated = false; }
    void     init(void);
    bool     isCelsius(void)                    { return true; } //Config.celsius; }
    bool     isCold(uint16_t temp);             // Whether the IRON is temperature is low
    uint16_t tempPresetHuman(void);             // The preset Temperature in the human readable units
    uint16_t tempPreset(void)                   { return  def_set; };//TODOConfig.temp; }
    uint16_t human2temp(uint16_t temp);         // Translate the human readable temperature into internal value
    uint16_t tempHuman(uint16_t temp);          // Thanslate temperature from internal units to the human readable value (Celsius or Farenheit)
    byte     selectTip(byte index);             // Select new tip, return selected tip index
    byte     getOffTimeout(void)                { return Config.off_timeout; }
    bool     getTempUnits(void)                 { return true; } //Config.celsius; }
    bool     savePresetTempHuman(uint16_t temp);// Save preset temperature in the human readable units
    bool     savePresetTemp(uint16_t temp);     // Save preset temperature in the internal units (convert it to the human readable units)
    void     saveConfig(byte off, bool cels);   // Save global configuration parameters
    void     getCalibrationData(uint16_t& t_min, uint16_t& t_mid, uint16_t& t_max);
    void     saveCalibrationData(uint16_t t_min, uint16_t t_mid, uint16_t t_max);
    void     setDefaults(bool Write = false);   // Set default parameter values if failed to load data from EEPROM
  private:
     
    byte     current_tip;                       // The current tip index
    bool     is_calibrated;                     // whether the tip has calibrated data
    // t_tip[] - array of internal sensor readings of the current tip at reference temperatures,
    // defined in temp_tip[] global array
    uint16_t t_tip[3];
    // const uint16_t def_tip[3] = {587, 751, 916};// Default values of internal sensor readings at reference temperatures
    // const uint16_t def_set = 653;               // Default preset temperature in internal units
    // const uint16_t ambient_temp  = 300;         // Ambient temperatire in the internal units
    // const uint16_t ambient_tempC = 25;          // Ambient temperature in Celsius
    
    const uint16_t def_tip[3] = {normalize(1138), normalize(1800), normalize(2461)};// TODO change the default for a IAN219 adc and K type thermal
    // Default values of internal sensor readings at 
    // reference temperatures for K type thermal tempretures in C: 280, 437, 593 -- this thermal temp NOT tip's
    const uint16_t def_set = normalize(1800);               // Default preset temperature in internal units
    const uint16_t ambient_temp  = normalize(100);         // Ambient temperatire in the internal units
    const uint16_t ambient_tempC = 25;          // Ambient temperature in Celsius
};

void IRON_CFG::init(void) {
  CONFIG::init();
  if (!CONFIG::load()) setDefaults();           // If failed to load the data from EEPROM, initialize the config data with default values
  uint32_t   cd = Config.calibration;
  t_tip[0] = cd & 0x3FF; cd >>= 10;             // 10 bits per calibration parameter, because the ADC readings are 10 bits
  t_tip[1] = cd & 0x3FF; cd >>= 10;
  t_tip[2] = cd & 0x3FF;
  dpint("IRON_CFG::init t_tip[0]=", t_tip[0]);
  dpint("IRON_CFG::init t_tip[1]=", t_tip[1]);
  dpint("IRON_CFG::init t_tip[2]=", t_tip[2]);
  
  // Check the tip calibration is correct
  if ((t_tip[0] >= t_tip[1]) || (t_tip[1] >= t_tip[2])) {
    setDefaults();
    for (byte i = 0; i < 3; ++i)
      t_tip[i] = def_tip[i];
    
    dpB("IRON_CFG::init use def ", true);
    dpint("IRON_CFG::init t_tip[0]=", t_tip[0]);
    dpint("IRON_CFG::init t_tip[1]=", t_tip[1]);
    dpint("IRON_CFG::init t_tip[2]=", t_tip[2]); 
  
  }
}

bool IRON_CFG::isCold(uint16_t temp) {
  return (temp < t_tip[0]) && (map(temp, ambient_temp, t_tip[0], ambient_tempC, temp_tip[0]) < 32);
}

uint16_t IRON_CFG::tempPresetHuman(void) {
  return tempHuman(tempPreset());
}

uint16_t IRON_CFG::human2temp(uint16_t t) {     // Translate the human readable temperature into internal value
  uint16_t temp = t;
  // if (!Config.celsius)
  //   temp = map(temp, temp_minF, temp_maxF, temp_minC, temp_maxC);
  if (t < temp_minC) t = temp_minC;
  if (t > temp_maxC) t = temp_maxC;
  if (t >= temp_tip[1])
    temp = map(t+1, temp_tip[1], temp_tip[2], t_tip[1], t_tip[2]);
  else
    temp = map(t+1, temp_tip[0], temp_tip[1], t_tip[0], t_tip[1]);
 
  for (byte i = 0; i < 10; ++i) {
    uint16_t tH = tempHuman(temp);
    if (tH <= t) break;
    --temp;
  }
  return temp;
}

// Thanslate temperature from internal units to the human readable value (Celsius or Farenheit)
uint16_t IRON_CFG::tempHuman(uint16_t temp) {
  uint16_t tempH = 0;
   
  if (temp < ambient_temp) {
    tempH = ambient_tempC;
  } else if (temp < t_tip[0]) {
    tempH = map(temp, ambient_temp, t_tip[0], ambient_tempC, temp_tip[0]);
  } else if (temp >= t_tip[1]) {
    tempH = map(temp, t_tip[1], t_tip[2], temp_tip[1], temp_tip[2]);
  } else {
    tempH = map(temp, t_tip[0], t_tip[1], temp_tip[0], temp_tip[1]);
  }
  dpB("Config.celsius ", Config.celsius);
  // if (!Config.celsius)
  //   tempH = map(tempH, temp_minC, temp_maxC, temp_minF, temp_maxF);
  return tempH;
}

bool IRON_CFG::savePresetTempHuman(uint16_t temp) {
  Config.temp = human2temp(temp);
  return CONFIG::save();
}

bool IRON_CFG::savePresetTemp(uint16_t temp) {
  Config.temp = temp;
  return CONFIG::save();
}

void IRON_CFG::saveConfig(byte off, bool cels) {
  if (off > 30) off = 0;
  Config.off_timeout = off;
  Config.celsius = cels;
  CONFIG::save();                               // Save new data into the EEPROM
}

void IRON_CFG::getCalibrationData(uint16_t& t_min, uint16_t& t_mid, uint16_t& t_max) {
  t_min = t_tip[0];
  t_mid = t_tip[1];
  t_max = t_tip[2];
}

void IRON_CFG::saveCalibrationData(uint16_t t_min, uint16_t t_mid, uint16_t t_max) {
  uint32_t cd = t_max & 0x3FF; cd <<= 10;       // Pack tip calibration data in one 32-bit word: 10-bits per value
  cd |= t_mid & 0x3FF; cd <<= 10;
  cd |= t_min;

  Config.calibration = cd;
  t_tip[0] = t_min;
  t_tip[2] = t_mid;
  t_tip[2] = t_max;
} 

void IRON_CFG::setDefaults(bool Write) {
  dpB("IRON_CFG::setDefaults can write = ", Write);
  uint32_t c = def_tip[2] & 0x3FF; c <<= 10;
  c |= def_tip[1] & 0x3FF;         c <<= 10;
  c |= def_tip[0] & 0x3FF;
  Config.calibration = c;
  Config.temp        = def_set;
  Config.off_timeout = 0;                       // Default autometic switch-off timeout (disabled)
  Config.celsius     = true;                    // Default use celsius
  if (Write) {
    CONFIG::clearAll();
    CONFIG::save();
  }
}

 

//------------------------------------------ class BUZZER ------------------------------------------------------
class BUZZER {
  public:
    BUZZER(byte buzzerP)  { buzzer_pin = buzzerP; }
    void init(void);
    void shortBeep(void)  { tone(buzzerPIN, 3520, 160); }
    void lowBeep(void)    { tone(buzzerPIN,  880, 160); }
    void doubleBeep(void) { tone(buzzerPIN, 3520, 160); delay(300); tone(buzzerPIN, 3520, 160); }
    void failedBeep(void) { tone(buzzerPIN, 3520, 160); delay(170);
                            tone(buzzerPIN,  880, 250); delay(260);
                            tone(buzzerPIN, 3520, 160);
                          }
  private:
    byte buzzer_pin;
};

void BUZZER::init(void) {
  pinMode(buzzer_pin, OUTPUT);
  noTone(buzzer_pin);
}

//------------------------------------------ class BUTTON ------------------------------------------------------
class BUTTON {
  public:
    BUTTON(byte ButtonPIN, unsigned int timeout_ms = 3000) {
      pt = tickTime = 0;
      buttonPIN = ButtonPIN;
      overPress = timeout_ms;
    }
    void init(void) { pinMode(buttonPIN, INPUT_PULLUP); }
    void setTimeout(uint16_t timeout_ms = 3000) { overPress = timeout_ms; }
    byte intButtonStatus(void) { byte m = mode; mode = 0; return m; }
    void changeINTR(void);
    byte buttonCheck(void);
    bool buttonTick(void);
  private:
    volatile byte     mode;                     // The button mode: 0 - not pressed, 1 - pressed, 2 - long pressed
    uint16_t          overPress;                // Maxumum time in ms the button can be pressed
    volatile uint32_t pt;                       // Time in ms when the button was pressed (press time)
    uint32_t          tickTime;                 // The time in ms when the button Tick was set
    byte              buttonPIN;                // The pin number connected to the button
    const uint16_t    tickTimeout = 200;        // Period of button tick, while tha button is pressed 
    const uint16_t    shortPress = 900;         // If the button was pressed less that this timeout, we assume the short button press
};

void BUTTON::changeINTR(void) {                 // Interrupt function, called when the button status changed
  
  bool keyUp = digitalRead(buttonPIN);
  unsigned long now_t = millis();
  if (!keyUp) {                                 // The button has been pressed
    if ((pt == 0) || (now_t - pt > overPress)) pt = now_t; 
  } else {
    if (pt > 0) {
      if ((now_t - pt) < shortPress) mode = 1;  // short press
        else mode = 2;                          // long press
      pt = 0;
    }
  }
   
  
}

byte BUTTON::buttonCheck(void) {                // Check the button state, called each time in the main loop

  mode = 0;
  bool keyUp = digitalRead(buttonPIN);          // Read the current state of the button
  uint32_t now_t = millis();
  if (!keyUp) {                                 // The button is pressed
    if ((pt == 0) || (now_t - pt > overPress)) pt = now_t;
  } else {
    if (pt == 0) return 0;
    if ((now_t - pt) > shortPress)              // Long press
      mode = 2;
    else
      mode = 1;
    pt = 0;
  } 
  return mode;
}

bool BUTTON::buttonTick(void) {                 // When the button pressed for a while, generate periodical ticks

  bool keyUp = digitalRead(buttonPIN);          // Read the current state of the button
  uint32_t now_t = millis();
  if (!keyUp && (now_t - pt > shortPress)) {    // The button have been pressed for a while
    if (now_t - tickTime > tickTimeout) {
       tickTime = now_t;
       return (pt != 0);
    }
  } else {
    if (pt == 0) return false;
    tickTime = 0;
  } 
  return false;
}

//------------------------------------------ class ENCODER ------------------------------------------------------
class ENCODER {
  public:
    ENCODER(byte aPIN, byte bPIN, int16_t initPos = 0) {
      pt = 0; mPIN = aPIN; sPIN = bPIN; pos = initPos;
      min_pos = -32767; max_pos = 32766; channelB = false; increment = 1;
      changed = 0;
      is_looped = false;
    }
    void init(void) {
      pinMode(mPIN, INPUT_PULLUP);
      pinMode(sPIN, INPUT_PULLUP);
    }
    void    set_increment(byte inc)             { increment = inc; }
    byte    get_increment(void)                 { return increment; }
    int16_t read(void)                          { return pos; }
    void    reset(int16_t initPos, int16_t low, int16_t upp, byte inc = 1, byte fast_inc = 0, bool looped = false);
    bool    write(int16_t initPos);
    void    changeINTR(void);
  private:
    int32_t           min_pos, max_pos;
    volatile uint32_t pt;                       // Time in ms when the encoder was rotaded
    volatile uint32_t changed;                  // Time in ms when the value was changed
    volatile bool     channelB;
    volatile int16_t  pos;                      // Encoder current position
    byte              mPIN, sPIN;               // The pin numbers connected to the main channel and to the socondary channel
    bool              is_looped;                // Whether the encoder is looped
    byte              increment;                // The value to add or substract for each encoder tick
    byte              fast_increment;           // The value to change encoder when in runs quickly
    const uint16_t    fast_timeout = 300;       // Time in ms to change encodeq quickly
    const uint16_t    overPress = 1000;
};

bool ENCODER::write(int16_t initPos) {
  if ((initPos >= min_pos) && (initPos <= max_pos)) {
    pos = initPos;
    return true;
  }
  return false;
}

void ENCODER::reset(int16_t initPos, int16_t low, int16_t upp, byte inc, byte fast_inc, bool looped) {
  min_pos = low; max_pos = upp;
  if (!write(initPos)) initPos = min_pos;
  increment = fast_increment = inc;
  if (fast_inc > increment) fast_increment = fast_inc;
  is_looped = looped;
}

void ENCODER::changeINTR(void) {                // Interrupt function, called when the channel A of encoder changed
  
  bool rUp = digitalRead(mPIN);
  unsigned long now_t = millis();
  if (!rUp) {                                   // The channel A has been "pressed"
    if ((pt == 0) || (now_t - pt > overPress)) {
      pt = now_t;
      channelB = digitalRead(sPIN);
    }
  } else {
    if (pt > 0) {
      byte inc = increment;
      if ((now_t - pt) < overPress) {
        if ((now_t - changed) < fast_timeout) inc = fast_increment;
        changed = now_t;
        if (channelB) pos -= inc; else pos += inc;
        if (pos > max_pos) { 
          if (is_looped)
            pos = min_pos;
          else 
            pos = max_pos;
        }
        if (pos < min_pos) {
          if (is_looped)
            pos = max_pos;
          else
            pos = min_pos;
        }
      }
      pt = 0; 
    }
  }
}

//------------------------------------------ class lcd DSPLay for soldering iron -----------------------------
class DSPL : protected LiquidCrystal {
  public:
    DSPL(byte RS, byte E, byte DB4, byte DB5, byte DB6, byte DB7) : LiquidCrystal(RS, E, DB4, DB5, DB6, DB7) { }
    void init(void);
    void clear(void) { LiquidCrystal::clear(); }
    void tSet(uint16_t t, bool celsuis);        // Show the temperature set
    void tCurr(uint16_t t);                     // Show The current temperature
    void pSet(byte p);                          // Show the power set
    void timeToOff(byte sec);                   // Show the time to automatic off the iron
    void msgNoIron(void);                       // Show 'No iron' message
    void msgReady(void);                        // Show 'Ready' message
    void msgWorking(void);                      // Show 'Working' message
    void msgOn(void);                           // Show 'On' message
    void msgOff(void);                          // Show 'Off' message
    void msgCold(void);                         // Show 'Cold' message
    void msgFail(void);                         // Show 'Fail' message
    void msgTune(void);                         // Show 'Tune' message
    void msgCelsius(void);                      // Show 'Cels.' message
    void msgFarneheit(void);                    // Show 'Faren.' message
    void msgDefault();                          // Show 'default' message (load default configuratuin)
    void msgCancel(void);                       // Show 'cancel' message
    void msgApply(void);                        // Show 'save message'
    void setupMode(byte mode, byte p = 0);      // Show the configureation mode [0 - 2]
    void percent(byte Power);                   // Show the percentage
    void overShoot(int os);                     // show overshoot left time in seconds
  private:
    bool full_second_line;                      // Wether the second line is full with the message
    const byte degree[8] = {
      0b00110,
      0b01001,
      0b01001,
      0b00110,
      0b00000,
      0b00000,
      0b00000,
      0b00000};
};

void DSPL::init(void) {
  LiquidCrystal::begin(16, 2);
  LiquidCrystal::clear();
  LiquidCrystal::createChar(1, degree);
  full_second_line = false;
}

void DSPL::tSet(uint16_t t, bool celsius) {
  char buff[6];
  char units = 'C';
  if (!celsius) units = 'F';
  LiquidCrystal::setCursor(0, 0);
  sprintf(buff, "%3d%c%c", t, char(1), units);
  LiquidCrystal::print(buff);
}

void DSPL::overShoot(int o) {  
  char buff[3];
  char units = 'F';
   
  LiquidCrystal::setCursor(6, 0);
  sprintf(buff, "%c%2d", units, o);
  LiquidCrystal::print(buff);
}

void DSPL::tCurr(uint16_t t) {
  char buff[5];
  LiquidCrystal::setCursor(0, 1);
  if (t < 1000) {
    sprintf(buff, "%3d%c", t, char(1));
  } else {
    LiquidCrystal::print(F("xxxx"));
    return;
  }
  LiquidCrystal::print(buff);
  LiquidCrystal::setCursor(7, 1);  // show sensor voltage
  LiquidCrystal::print(sensorMvf, 2);

  // if (full_second_line) {
  //   LiquidCrystal::print(F("            "));
  //   full_second_line = false;
  // }
}

void DSPL::pSet(byte p) {
  char buff[6];
  sprintf(buff, "P:%3d", p);
  LiquidCrystal::setCursor(0, 0);
  LiquidCrystal::print(buff);
}

void DSPL::timeToOff(byte sec) {
  char buff[12];
  sprintf(buff, "       %3ds", sec);
  LiquidCrystal::setCursor(5, 0);
  LiquidCrystal::print(buff);
}

void DSPL::msgNoIron(void) {
  LiquidCrystal::setCursor(0, 1);
  LiquidCrystal::print(F("    no iron     "));
  full_second_line = true;
}

void DSPL::msgReady(void) {
  LiquidCrystal::setCursor(5, 0);
  LiquidCrystal::print(F("      ready"));
}

void DSPL::msgWorking(void) {
  LiquidCrystal::setCursor(5, 0);
  LiquidCrystal::print(F("    working"));
}

void DSPL::msgOn(void) {
  LiquidCrystal::setCursor(12, 0);
  LiquidCrystal::print(F("  ON"));
}

void DSPL::msgOff(void) {
  LiquidCrystal::setCursor(5, 0);
  LiquidCrystal::print(F("        OFF"));
}

void DSPL::msgCold(void) {
  LiquidCrystal::setCursor(0, 1);
  LiquidCrystal::print(F("      cold      "));
  full_second_line = true;
}

void DSPL::msgFail(void) {
  LiquidCrystal::setCursor(0, 1);
  LiquidCrystal::print(F("     Failed     "));
}

void DSPL::msgTune(void) {
  LiquidCrystal::setCursor(0, 0);
  LiquidCrystal::print(F("Tune"));
}

void DSPL::msgCelsius(void) {
  LiquidCrystal::setCursor(0, 1);
  LiquidCrystal::print(F(" Celsius        "));
}

void DSPL::msgFarneheit(void) {
  LiquidCrystal::setCursor(0, 1);
  LiquidCrystal::print(F(" Farenheit      "));
}

void DSPL::msgDefault() {
  LiquidCrystal::setCursor(0, 1);
  LiquidCrystal::print(F(" default         "));
}

void DSPL::msgCancel(void) {
  LiquidCrystal::setCursor(0, 1);
  LiquidCrystal::print(F(" cancel         "));
}

void DSPL::msgApply(void) {
  LiquidCrystal::setCursor(0, 1);
  LiquidCrystal::print(F(" save           "));
}

void DSPL::setupMode(byte mode, byte p) {
  char buff[5];
  LiquidCrystal::clear();
  LiquidCrystal::print(F("setup"));
  LiquidCrystal::setCursor(1,1);
  switch (mode) {
    case 0:
    LiquidCrystal::print(F("off:"));
    if (p > 0) {
      sprintf(buff, "%2dm", p);
      LiquidCrystal::print(buff);
    } else {
      LiquidCrystal::print(" NO");
    }
    break;
    case 1:
      LiquidCrystal::print(F("units "));
      LiquidCrystal::setCursor(7, 1);
      LiquidCrystal::print(char(1));
      if (p)
        LiquidCrystal::print("C");
      else
        LiquidCrystal::print("F");
      break;
    case 2:
      LiquidCrystal::print(F("calibrate"));
      break;
    case 3:
      LiquidCrystal::print(F("tune"));
      break;
  }
}

void DSPL::percent(byte Power) {
  char buff[13];
  sprintf(buff, "       %3d%c", Power, '%');
  LiquidCrystal::setCursor(4, 1);
  //LiquidCrystal::print(buff);//TODO
}

//------------------------------------------ class HISTORY ----------------------------------------------------
#define H_LENGTH 16
class HISTORY {
  public:
    HISTORY(void)                               { len = 0; }
    void     init(void)                         { len = 0; }
    bool     isFull(void)                       { return len == H_LENGTH; }
    uint16_t last(void);
    uint16_t top(void)                          { return queue[0]; }
    void     put(uint16_t item);                // Put new entry to the history
    uint16_t average(void);                     // calcilate the average value
    float    dispersion(void);                  // calculate the math dispersion
    float    gradient(void);                    // calculate the gradient of the history values
  private:
    volatile uint16_t queue[H_LENGTH];
    volatile byte len;                          // The number of elements in the queue
    volatile byte index;                        // The current element position, use ring buffer
};

void HISTORY::put(uint16_t item) {
  if (len < H_LENGTH) {
    queue[len++] = item;
  } else {
    queue[index ] = item;
    if (++index >= H_LENGTH) index = 0;         // Use ring buffer
  }
}

uint16_t HISTORY::last(void) {
  byte i = H_LENGTH - 1;
  if (index)
    i = index - 1;
  return queue[i];
}

uint16_t HISTORY::average(void) {
  uint32_t sum = 0;
  if (len == 0) return 0;
  if (len == 1) return queue[0];
  for (byte i = 0; i < len; ++i) sum += queue[i];
  sum += len >> 1;                              // round the average
  sum /= len;
  return uint16_t(sum);
}

float HISTORY::dispersion(void) {
  if (len < 3) return 1000;
  uint32_t sum = 0;
  uint32_t avg = average();
  for (byte i = 0; i < len; ++i) {
    long q = queue[i];
    q -= avg;
    q *= q;
    sum += q;
  }
  sum += len << 1;
  float d = (float)sum / (float)len;
  return d;
}

// approfimating the history with the line (y = ax+b) using method of minimum square. Gradient is parameter a
float HISTORY::gradient(void) {
  if (len < 2) return 0;
  long sx, sx_sq, sxy, sy;
  sx = sx_sq = sxy = sy = 0;
  for (byte i = 1; i <= len; ++i) {
    sx    += i;
    sx_sq += i*i;
    sxy   += i*queue[i-1];
    sy    += queue[i-1];
  }
  long numerator   = len * sxy - sx * sy;
  long denominator = len * sx_sq - sx * sx;
  float a = (float)numerator / (float)denominator;
  return a;
}

//------------------------------------------ class PID algoritm to keep the temperature -----------------------
/*  The PID algoritm 
 *  Un = Kp*(Xs - Xn) + Ki*summ{j=0; j<=n}(Xs - Xj) + Kd(Xn - Xn-1),
 *  Where Xs - is the setup temperature, Xn - the temperature on n-iteration step
 *  In this program the interactive formulae is used:
 *    Un = Un-1 + Kp*(Xn-1 - Xn) + Ki*(Xs - Xn) + Kd*(Xn-2 + Xn - 2*Xn-1)
 *  With the first step:
 *  U0 = Kp*(Xs - X0) + Ki*(Xs - X0); Xn-1 = Xn;
 */
class PID {
  public:
    PID(void) {
      Kp = 768;
      Ki =  40;
      Kd = 260;
    }
    void resetPID(int temp = -1);               // reset PID algoritm history parameters
    // Calculate the power to be applied
    int reqPower(int temp_set, int temp_curr);
    int changePID(byte p, int k);
  private:
    void  debugPID(int t_set, int t_curr, long kp, long ki, long kd, long delta_p);
    int   temp_h0, temp_h1;                     // previously measured temperature
    int   temp_diff_iterate;                    // The temperature difference to start iterate process
    bool  pid_iterate;                          // Whether the iterative process is used
    long  i_summ;                               // Ki summary multiplied by denominator
    long  power;                                // The power iterative multiplied by denominator
    long  Kp, Ki, Kd;                           // The PID algorithm coefficients multiplied by denominator
    const byte denominator_p = 9;               // The common coefficeient denominator power of 2 (9 means divide by 512)
};

void PID::resetPID(int temp) {
  temp_h0 = 0;
  power  = 0;
  i_summ = 0;
  pid_iterate = false;
  if ((temp > 0) && (temp < 1000))
    temp_h1 = temp;
  else
    temp_h1 = 0;
}

int PID::changePID(byte p, int k) {
  switch(p) {
    case 1:
      if (k >= 0) Kp = k;
      return Kp;
    case 2:
      if (k >= 0) Ki = k;
      return Ki;
    case 3:
      if (k >= 0) Kd = k;
      return Kd;
    default:
      break;
  }
  return 0;
}

int PID::reqPower(int temp_set, int temp_curr) {
  if (temp_h0 == 0) {
    // When the temperature is near the preset one, reset the PID and prepare iterative formulae                        
    if ((temp_set - temp_curr) < 10) {
      if (!pid_iterate) {
        pid_iterate = true;
        power = 0;
        i_summ = 0;
      }
    }
    i_summ += temp_set - temp_curr;             // first, use the direct formulae, not the iterate process
    power = Kp*(temp_set - temp_curr) + Ki*i_summ;
    // If the temperature is near, prepare the PID iteration process
  } else {
    long kp = Kp * (temp_h1 - temp_curr);
    long ki = Ki * (temp_set - temp_curr);
    long kd = Kd * (temp_h0 + temp_curr - 2*temp_h1);
    long delta_p = kp + ki + kd;
    power += delta_p;                           // power keeped multiplied by denominator!
  }
  if (pid_iterate) temp_h0 = temp_h1;
  temp_h1 = temp_curr;
  long pwr = power + (1 << (denominator_p-1));  // prepare the power to delete by denominator, roud the result
  pwr >>= denominator_p;                        // delete by the denominator
  return int(pwr);
}

//------------------------- class FastPWM operations using Timer1 on pin D10 at 31250 Hz ----------------------
class FastPWM {
  public:
    FastPWM()                                   { }
    void init(void);
    //void duty(byte d)                           { OCR1B = d; }
    void duty(byte d)                             { //analogWrite(heaterPIN, d); 
     
    }
    void on()            { digitalWrite(heaterPIN, HIGH);}  //
    void off()           { digitalWrite(heaterPIN, LOW);}
};

void FastPWM::init(void) {
  pinMode(heaterPIN, OUTPUT);                          // Use D10 pin for heationg the IRON
  digitalWrite(heaterPIN, LOW);                        // Switch-off the power
  tmr1_count = 0;
  //iron_off = false; 
  // noInterrupts();
  // TCNT1   = 0;
  // TCCR1B  = _BV(WGM13);                         // Set mode as phase and frequency correct pwm, stop the timer
  // TCCR1A  = 0;
  // ICR1    = 256;
  // TCCR1B  = _BV(WGM13) | _BV(CS10);             // Top value = ICR1, prescale = 1; 31250 Hz
  // TCCR1A |= _BV(COM1B1);                        // XOR D10 on OC1B, detached from D09
  // OCR1B   = 0;                                  // Switch-off the signal on pin D10;
  // TIMSK1  = _BV(TOIE1);                         // Enable overflow interrupts @31250 Hz
  // interrupts();
}

//------------------------------------------ class soldering iron ---------------------------------------------
class IRON : protected PID {
  public:
    IRON(byte heater_pin, byte sensor_pin) {
      hPIN = heater_pin;
      sPIN = sensor_pin;
      on = false;
      fix_power = false;
      no_iron = false;//TODO
    }
    void     init(void);
    void     switchPower(bool On);
    bool     isOn(void)                         { return on || fix_power; }
    bool     noIron(void)                       { return no_iron; }
    uint16_t getTemp(void)                      { return temp_set; }
    uint16_t getCurrTemp(void)                  { return nlSensorReading; }  // TODO { return h_temp.last(); }
    uint16_t tempAverage(void)                  { return nlSensorReading; } // TODO { return h_temp.average(); }
    uint16_t tempDispersion(void)               { return h_temp.dispersion(); }
    uint16_t powerDispersion(void)              { return h_power.dispersion(); }
    byte     getMaxFixedPower(void)             { return max_fixed_power; }
    int      changePID(byte p, int k)           { return PID::changePID(p, k); }
    int      overShootLeft()                    { return overshootLeftS; }
    void     setTemp(uint16_t t);               // Set the temperature to be keeped
    byte     getAvgPower(void);                 // Average applied power
    byte     appliedPower(void);                // Power applied to the solder [0-100%]
    byte     hotPercent(void);                  // How hot is the iron (used in the idle state)
	  void     checkIron(void);                   // Check the IRON, stop it in case of emergency
    void     keepTemp(void);                    // Keep the IRON temperature, called by Timer1 interrupt
    bool     fixPower(byte Power);              // Set the specified power to the the soldering iron
    void     setNextCheckTempTimeMS();          // set next time in millis for checking iron tempreture. 
    uint16_t convertSensorMv2TempC(float mvf);
    void     setWork(boolean work);                // set iron to work/sleep (true/false0) state
    boolean  isWorkState()                      { return workState; }        // iron is working(true)/sleep(false);
    void     setOverShootTime(int16_t start, int16_t end);
    void     setMaxTemp(uint16_t max);   
    uint16_t getMaxTemp()                       { return max_temp; }

  private:
    FastPWM  fastPWM;                           // Power the IRON using fast PWM through D10 pin using Timer1
    uint32_t check_ironMS;                      // Milliseconds when to check the IRON is connected
    byte     hPIN, sPIN;                        // The heater PIN and the sensor PIN
    int      power;                             // The soldering station power
    byte     actual_power;                      // The power supplied to the iron
    bool     fix_power;                         // Whether the soldering iron is set the fix power
    uint16_t temp_set;                          // The temperature that should be keeped
    bool     iron_checked;                      // Whether the iron works
    uint16_t temp_start;                        // The temperature when the solder was switched on
    uint32_t elapsed_time;                      // The time elipsed from the start (ms)
    uint16_t temp_min;                          // The minimum temperature (180 centegrees)
    volatile bool on;                           // Whether the soldering iron is on
    volatile bool no_iron;                      // Whether the iron is connected
    volatile bool chill;                        // Whether the IRON should be cooled (preset temp is lower than current)
    uint16_t temp_max;                          // The maximum temperature (400 centegrees)
    HISTORY  h_power;
    HISTORY  h_temp;
    
    const uint16_t temp_no_iron    = 980;       // Sensor reading when the iron disconnected
    const byte     max_power       = 180;       // maximum power to the iron (220)
    const byte     max_fixed_power = 120;       // Maximum power in fiexed power mode
    const uint16_t check_time      = 10000;     // Time in ms to check Whether the solder is heating
    const uint16_t heat_expected   = 10;        // The iron should change the temperature at check_time
	  const uint32_t check_iron_ms   = 1000;      // The period in ms to check Whether the IRON is conected
    const uint32_t check_temp_ms   = 100;       // time frame for check iron tempreture
    const uint16_t Overshoot_temp  = 700; //28mv 673C 
    uint32_t next_check_temp_time_ms   = 0;       //next time in millis for checking iron tempreture. 
                                                  //  should be set to mills() + check_temp_ms when one tempreture check is completed
    boolean workState = false;                   // iron is in working state when value is TRUE, otherwise in sleeping state
    boolean warming = false;  
    uint16_t max_temp = 500;                    //  iron temp is down to warming temp (about 200C) when keek preset temp for a specified the time (1/2/3 minutes)
    int16_t sensorReadingRaw = 0;
    int16_t nlSensorReading = 0;
    int overshootMS = 0;                  // when the different of target and start temp are big 
    // ( over 100c) it will take a long time for the tip to get the setting temp after the thermal couple 
    // reach the target temp. So we need to keep the iron on after thermal couple temp reading reach the target.
    // the over shoot time is calculated based on the diff. 
    uint32_t overshoot_expire_time_ms   = 0;
    boolean reachTarget = false;     
    int overshootLeftS = 0; // overshoot time (S) left
};
void IRON::setMaxTemp(uint16_t max){
  max_temp = max;
}
void IRON::setTemp(uint16_t t) {
  //if (on) resetPID();
  temp_set = t;
  uint16_t ta = h_temp.average();
  chill = (ta > t + 5);                         // The IRON must be cooled
}

void IRON::setNextCheckTempTimeMS(){    
  next_check_temp_time_ms =  millis() + check_temp_ms;
}
byte IRON::getAvgPower(void) {
  int p = h_power.average();
  return p & 0xff;  
}

byte IRON::appliedPower(void) {
  byte p = getAvgPower(); 
  return map(p, 0, max_power, 0, 100);  
}
void IRON::setWork(boolean work)  { workState = work; } 
void IRON::setOverShootTime(int16_t startC, int16_t endC){

  int16_t diff = endC - startC;
  if (diff < 30) overshootMS = 0;
  else overshootMS = diff*1000/30;
  dpint("IRON::setOverShootTime ms=", overshootMS);
  reachTarget = false;
}
void IRON::init(void) {
   
  //TODO setTemp();
            
  fix_power = false;
  power = 0;
  actual_power = 0;
  elapsed_time = 0;
  
  iron_checked = false;
  // resetPID();
  // h_power.init();
  // h_temp.init();
  check_ironMS = 0;
}

void IRON::switchPower(bool On) {
  on = On;
  if (!on) {
    //fastPWM.duty(0);
    fastPWM.off();
    fix_power = false;
    return;
  }
  fastPWM.on();//TODO test
  setNextCheckTempTimeMS();
  //resetPID(analogRead(sPIN));
  //h_power.init();
}

void IRON::checkIron(void) {
  
  if (millis() < check_ironMS) return;
  check_ironMS = millis() + check_iron_ms;
   
  //if (diff )

  // // Check Whether the iron can be heated
  // if (!iron_checked) {
  //   elapsed_time += check_iron_ms;
  //   if (elapsed_time >= check_time) {
  //     uint16_t temp = h_temp.average();
  //     if ((abs(temp_set - temp) < 100) || ((temp - temp_start) > heat_expected)) {
  //       iron_checked = true;
  //     } else {
  //       switchPower(false);                     // Prevent the iron damage
  //       elapsed_time = 0;
  //       temp_start = analogRead(sPIN);
  //       iron_checked = false;
  //     }
  //   }
  // }
  
  // if (!on && !fix_power) {                      // If the soldering IRON is set to be switched off
  //   fastPWM.duty(0);   
  //   //fastPWM.off();                       // Surely power off the IRON
  // }
  // if (on && no_iron) {
  //   switchPower(false);
  // }
}

void IRON::keepTemp(void) {
  if (!on) fastPWM.off();
  
  if (millis() >= next_check_temp_time_ms){
    setNextCheckTempTimeMS();
    sensorReadingRaw =  readSensorRaw();
    nlSensorReading = normalize(sensorReadingRaw);
    sensorMvf = convertSensorRaw2MV(sensorReadingRaw);
    dpint("keepTemp getTemp()= ",  getTemp() );
    dpint("keepTemp getMaxTemp()= ", getMaxTemp() );
    if (on){
      no_iron = false;//TODO
      uint16_t targetTemp = getTemp();
      if (paddleDown) targetTemp = Overshoot_temp; //getMaxTemp();
      dpint("targetTemp = ", targetTemp);
      if( (uint16_t)nlSensorReading > targetTemp ){//TODO fine tuning heating algorithm          
            fastPWM.off();   
      }else{
        fastPWM.on();
      }
    } 
  }
   
}

bool IRON::fixPower(byte Power) {
  if (Power == 0) {                             // To switch off the IRON, set the Power to 0
    fix_power = false;
    actual_power = 0;
    fastPWM.duty(0);
    return true;
  }

  if (Power > max_fixed_power) {
    actual_power = 0;
    return false;
  }

  if (!fix_power) {
    fix_power = true;
    power = Power;
    actual_power = power & 0xff;
  } else {
    if (power != Power) {
      power = Power;
      actual_power = power & 0xff;
    }
  }
  fastPWM.duty(actual_power);
  return true;
}

//------------------------------------------ class SCREEN ------------------------------------------------------
typedef enum s_mode { S_STANDBY, S_WORK, S_POWER, S_CONFIG, S_ERROR} sMode;
class SCREEN {
  public:
    SCREEN* next;                               // Pointer to the next screen
    SCREEN* nextL;                              // Pointer to the next Level screen, usually, setup
    SCREEN* main;                               // Pointer to the main screen
    SCREEN() {
      next = nextL = main = 0;
      update_screen  = 0;
      scr_timeout    = 0;
      time_to_return = 0;
    }
    virtual void    init(void)                  { }
    virtual void    show(void)                  { }
    virtual SCREEN* menu(void)                  {if (this->next != 0) return this->next; else return this; }
    virtual SCREEN* menu_long(void)             { if (this->nextL != 0) return this->nextL; else return this; }
    virtual void    rotaryValue(int16_t value)  { }
    virtual SCREEN* returnToMain(void);         // Return to the main screen in the menu tree
    bool            isSetup(void)               { return (scr_timeout != 0); }
    void            forceRedraw(void)           { update_screen = 0; }
    void            resetTimeout(void);         // Reset automatic return timeout
    void            setSCRtimeout(uint16_t t)   { scr_timeout = t; resetTimeout(); } 
    bool            wasRecentlyReset(void);     // Whether the return timeout was reset in the last 15 seconds
  protected:
    sMode   smode;                              // The screen mode
	  uint32_t update_screen;                     // Time in ms when the sreen should be updated
    uint16_t scr_timeout;                       // Timeout is sec. to return to the main screen, canceling all changes
    uint32_t time_to_return;                    // Time in ms to return to main screen
};

SCREEN* SCREEN::returnToMain(void) {
  if (main && (scr_timeout != 0) && (millis() >= time_to_return)) {
    scr_timeout = 0;
    return main;
  }
  return this;
}

void SCREEN::resetTimeout(void) {
  if (scr_timeout > 0)
    time_to_return = millis() + (uint32_t)scr_timeout*1000;
}

bool SCREEN::wasRecentlyReset(void) {
  uint32_t to = (time_to_return - millis()) / 1000;
  return((scr_timeout - to) < 15);
}

//---------------------------------------- class mainSCREEN [the soldering iron is OFF] ------------------------
class mainSCREEN : public SCREEN {
  public:
    mainSCREEN(IRON* Iron, DSPL* DSP, ENCODER* ENC, BUZZER* Buzz, IRON_CFG* Cfg) {
      update_screen = 0;
      pIron = Iron;
      pD    = DSP;
      pEnc  = ENC;
      pBz   = Buzz;
      pCfg  = Cfg;
      is_celsius = true;
	    smode = S_STANDBY;
    }
    virtual void init(void);
    virtual void show(void);
    virtual void rotaryValue(int16_t value);
  private:
    IRON*     pIron;                            // Pointer to the iron instance
    DSPL*     pD;                               // Pointer to the DSPLay instance
    ENCODER*  pEnc;                             // Pointer to the rotary encoder instance
    BUZZER*   pBz;                              // Pointer to the simple buzzer instance
    IRON_CFG* pCfg;                             // Pointer to the configuration instance
    bool      used;                             // Whether the iron was used (was hot)
    bool      cool_notified;                    // Whether there was cold notification played
    bool      is_celsius;                       // The temperature units (Celsius or Farenheit)
	  const uint16_t period = 1000;               // The period to update the screen
};

void mainSCREEN::init(void) {
  pIron->switchPower(false);
  uint16_t temp_set = pIron->getTemp();
  Serial.print("mainSCREEN init getTemp=");
  Serial.println(temp_set);
  is_celsius = pCfg->isCelsius();
  uint16_t tempH = pCfg->tempHuman(temp_set);
  if (is_celsius)
    pEnc->reset(tempH, temp_minC, temp_maxC, 1, 5);
  else
    pEnc->reset(tempH, temp_minF, temp_maxF, 1, 5);
  pD->clear();
  uint16_t temp = pIron->getCurrTemp();
  used = !pCfg->isCold(temp);
  cool_notified = !used;
  if (used) {                                   // the iron was used, we should save new data in EEPROM
    pCfg->savePresetTemp(temp_set);
  }
  forceRedraw();
}

void mainSCREEN::rotaryValue(int16_t value) {
  update_screen = millis() + period;
  uint16_t temp = pCfg->human2temp(value);
  pIron->setTemp(temp);
  pD->tSet(value, is_celsius);
}

void mainSCREEN::show(void) {
  if (millis() < update_screen) return;
  update_screen = millis() + period;

  uint16_t temp_set = pIron->getTemp();
  Serial.print("mainSCREEN show getTemp=");
  Serial.println(temp_set );
  temp_set = pCfg->tempHuman(temp_set);
  Serial.print(" tempHuman = ");
  Serial.println(temp_set);
  pD->tSet(temp_set, is_celsius);
  pD->msgOff();
  
  if (pIron->noIron()) {                        // No iron connected
    pD->msgNoIron();
    return;
  }

  uint16_t temp  = pIron->tempAverage();
  uint16_t tempH = pCfg->tempHuman(temp);
  if (pCfg->isCold(temp)) {
    if (used)
      pD->msgCold();
    else
      pD->tCurr(tempH);
    if (!cool_notified) {
      pBz->lowBeep();
      cool_notified = true;
    }
  } else {
    pD->tCurr(tempH);
  }
}

//---------------------------------------- class workSCREEN [the soldering iron is ON] -------------------------
class workSCREEN : public SCREEN {
  public:
    workSCREEN(IRON* Iron, DSPL* DSP, ENCODER* Enc, BUZZER* Buzz, IRON_CFG* Cfg) {
      update_screen = 0;
      pIron = Iron;
      pD    = DSP;
      pBz   = Buzz;
      pEnc  = Enc;
      pCfg  = Cfg;
      ready = false;
	  smode = S_WORK;
    }
    virtual void init(void);
    virtual void show(void);
    virtual void rotaryValue(int16_t value);
    virtual SCREEN* returnToMain(void);
  private:
    IRON*     pIron;                            // Pointer to the iron instance
    DSPL*     pD;                               // Pointer to the DSPLay instance
    BUZZER*   pBz;                              // Pointer to the simple Buzzer instance
    ENCODER*  pEnc;                             // Pointer to the rotary encoder instance
    IRON_CFG* pCfg;                             // Pointer to the configuration instance
    bool      ready;                            // Whether the iron is ready
    uint32_t  auto_off_notified;                // The time (in ms) when the automatic power-off was notified
    HISTORY  idle_power;                        // The power supplied to the iron when it is not used
	  const uint16_t period = 1000;               // The period to update the screen (ms)
};

void workSCREEN::init(void) {
  uint16_t temp_set = pIron->getTemp();
  bool is_celsius = pCfg->isCelsius();
  uint16_t tempH = pCfg->tempHuman(temp_set);
  pIron->setMaxTemp(pCfg->human2temp(400));//TODO 
 // int16_t nlSensorReading =  normalize(readSensorRaw());
 // uint16_t tempSet = pIron->getTemp(); 
  // if (tempSet > nlSensorReading)
  // {
  //   uint16_t tempSetC = pCfg->tempHuman(tempSet);
  //   uint16_t nlSensorReadingC =  pCfg->tempHuman(nlSensorReading);
  //   pIron->setOverShootTime(nlSensorReadingC, tempSetC);
    
  // }
  


  if (is_celsius)
    pEnc->reset(tempH, temp_minC, temp_maxC, 1, 5);
  else
    pEnc->reset(tempH, temp_minF, temp_maxF, 1, 5);
  pIron->switchPower(true);
  ready = false;
  pD->clear();
  pD->tSet(tempH, is_celsius);
  pD->msgOn();
  uint16_t to = pCfg->getOffTimeout() * 60;
  this->setSCRtimeout(to);
  idle_power.init();
  auto_off_notified = 0;
  forceRedraw();
}

void workSCREEN::rotaryValue(int16_t value) {
  ready = false;
  pD->msgOn();
  update_screen = millis() + period;
  uint16_t temp = pCfg->human2temp(value);
  pIron->setTemp(temp);
  pD->tSet(value, pCfg->isCelsius());
  idle_power.init();
  SCREEN::resetTimeout();
}

void workSCREEN::show(void) {
  if (millis() < update_screen) return;
  update_screen = millis() + period;

  int temp      = pIron->tempAverage();//TODO
  int temp_set  = pIron->getTemp();
  int tempH     = pCfg->tempHuman(temp); 
  
  pD->tCurr(tempH);
  if(paddleDown){// SHOW paddle down time in second on first line after pre-set temp
    int paddleDownSec = (millis() - paddleDown_start_ms) / 1000;
    pD->overShoot(paddleDownSec);
  } 
  byte p = pIron->appliedPower();
  pD->percent(p);

  uint16_t td = pIron->tempDispersion();
  uint16_t pd = pIron->powerDispersion();
  int ip      = idle_power.average();
  int ap      = pIron->getAvgPower();
  if ((temp <= temp_set) && (temp_set - temp <= 3) && (td <= 3) && (pd <= 4)) {
    idle_power.put(ap);
  }
  if (ap - ip >= 2) {                           // The IRON was used
    SCREEN::resetTimeout();
    if (ready) {
      idle_power.init();
      idle_power.put(ip+1);
      auto_off_notified = 0;
    }
  }

  if ((abs(temp_set - temp) < 3) && (pIron->tempDispersion() <= 3))  {
    if (!ready) {
      idle_power.put(ap);
      pBz->shortBeep();
      ready = true;
      pD->msgReady();
      update_screen = millis() + (period << 2);
      return;
    }
  }
  
  uint32_t to = (time_to_return - millis()) / 1000;
  if (ready) {
    if (scr_timeout > 0 && (to < 100)) {
      pD->timeToOff(to);
      if (!auto_off_notified) {
        pBz->shortBeep();
        auto_off_notified = millis();
      }
    } else if (SCREEN::wasRecentlyReset()) {
      pD->msgWorking();
    } else {
      pD->msgReady();
    }
  } else {
    pD->msgOn();
  }

}

SCREEN* workSCREEN::returnToMain(void) {
  if (main && (scr_timeout != 0) && (millis() >= time_to_return)) {
    scr_timeout = 0;
    pBz->doubleBeep();
    return main;
  }
  return this;
}

//---------------------------------------- class errorSCREEN [the soldering iron error detected] ---------------
class errorSCREEN : public SCREEN {
  public:
    errorSCREEN(DSPL* DSP, BUZZER* BZ) {
      pD  = DSP;
      pBz = BZ;
	    smode = S_ERROR;
    }
    virtual void init(void)                     { pD->clear(); pD->msgFail(); pBz->failedBeep(); }
  private:
    DSPL*   pD;                                 // Pointer to the display instance
    BUZZER* pBz;                                // Pointer to the buzzer instance
};

//---------------------------------------- class powerSCREEN [fixed power to the iron] -------------------------
class powerSCREEN : public SCREEN {
  public:
    powerSCREEN(IRON* Iron, DSPL* DSP, ENCODER* Enc, IRON_CFG* CFG) {
      pIron = Iron;
      pD    = DSP;
      pEnc  = Enc;
      pCfg  = CFG;
      on    = false;
	    smode = S_POWER;
    }
    virtual void init(void);
    virtual void show(void);
    virtual void rotaryValue(int16_t value);
    virtual SCREEN* menu(void);
    virtual SCREEN* menu_long(void);
  private:
    IRON*     pIron;                            // Pointer to the iron instance
    DSPL*     pD;                               // Pointer to the DSPLay instance
    ENCODER*  pEnc;                             // Pointer to the rotary encoder instance
    IRON_CFG* pCfg;                             // Pointer to the IRON Config
    bool      on;                               // Whether the power of soldering iron is on
	  const uint16_t period = 500;                // The period in ms to update the screen
};

void powerSCREEN::init(void) {
  byte p = pIron->getAvgPower();
  byte max_power = pIron->getMaxFixedPower();
  pEnc->reset(p, 0, max_power, 1);
  on = true;                                    // Do start heating immediately
  pIron->switchPower(false);
  pIron->fixPower(p);
  pD->clear();
  pD->pSet(p);
  forceRedraw();
}

void powerSCREEN::show(void) {
  if (millis() < update_screen) return;
  uint16_t temp = pIron->tempAverage();
  temp = pCfg->tempHuman(temp);
  pD->tCurr(temp);
  update_screen = millis() + period;
}

void powerSCREEN::rotaryValue(int16_t value) {
  pD->pSet(value);
  if (on)
    pIron->fixPower(value);
  update_screen = millis() + (period << 1);
}

SCREEN* powerSCREEN::menu(void) {
  on = !on;
  if (on) {
    uint16_t pos = pEnc->read();
    on = pIron->fixPower(pos);
	  pD->clear();
    pD->pSet(pos);
	  update_screen = 0;
  } else {
    pIron->fixPower(0);
	  pD->clear();
	  pD->pSet(0);
	  pD->msgOff();
  }
  return this;
}

SCREEN* powerSCREEN::menu_long(void) {
  pIron->fixPower(0);
  if (nextL) {
    pIron->switchPower(true);
    return nextL;
  }
  return this;
}

//---------------------------------------- class configSCREEN [configuration menu] -----------------------------
class configSCREEN : public SCREEN {
  public:
    configSCREEN(IRON* Iron, DSPL* DSP, ENCODER* Enc, IRON_CFG* Cfg) {
      pIron = Iron;
      pD    = DSP;
      pEnc  = Enc;
      pCfg  = Cfg;
      smode = S_CONFIG;
    }
    virtual void init(void);
    virtual void show(void);
    virtual void rotaryValue(int16_t value);
    virtual SCREEN* menu(void);
    virtual SCREEN* menu_long(void);
    SCREEN* calib;                              // Pointer to the calibration SCREEN
  private:
    IRON*     pIron;                            // Pointer to the IRON instance
    DSPL*     pD;                               // Pointer to the DSPLay instance
    ENCODER*  pEnc;                             // Pointer to the rotary encoder instance
    IRON_CFG* pCfg;                             // Pointer to the config instance
    byte mode;                                  // Which parameter to change
    bool tune;                                  // Whether the parameter is modifiying
    bool changed;                               // Whether some configuration parameter has been changed
    bool cels;                                  // Current celsius/farenheit;
    byte off_timeout;                           // Automatic switch-off timeout in minutes
    const uint16_t period = 10000;              // The period in ms to update the screen
};

void configSCREEN::init(void) {
  mode = 0;
  pEnc->reset(mode, 0, 6, 1, 0, true);          // 0 - off-timeout, 1 - C/F, 2 - tip calibrate, 3 - tune, 4 - save, 5 - cancel, 6 - defaults
  tune        = false;
  changed     = false;
  cels        = pCfg->getTempUnits();
  off_timeout = pCfg->getOffTimeout();
  pD->clear();
  pD->setupMode(0);
  this->setSCRtimeout(30);
}

void configSCREEN::show(void) {
  if (millis() < update_screen) return;
  update_screen = millis() + period;
  switch (mode) {
    case 0:
      pD->setupMode(mode, off_timeout);
      break;
    case 1:
      if (tune) {
        if (cels)
          pD->msgCelsius();
        else
          pD->msgFarneheit();
      } else {
        pD->setupMode(mode, cels);
      }
      break;
    case 2:
    case 3:
      pD->setupMode(mode, cels);
      break;
    case 4:
      pD->msgApply();
      break;
    case 5:
      pD->msgCancel();
      break;
    case 6:
      pD->msgDefault();
    default:
      break;
  }
}

void configSCREEN::rotaryValue(int16_t value) {
  if (tune) {                                   // tune the temperature units
    changed = true;
    switch (mode) {
      case 0:                                   // tuning the switch-off timeout
        if (value > 0) value += 2;              // The minimum timeout is 3 minutes
        off_timeout = value;
        break;
      case 1:                                   // tunung the temperature units
        cels = value;
        break;
      default:
        break;
    }
  } else {
    mode = value;
  }
  forceRedraw();
}

SCREEN* configSCREEN::menu(void) {
  if (tune) {
    tune = false;
    pEnc->reset(mode, 0, 6, 1, 0, true);        // The value has been tuned, return to the menu list mode
  } else {
    int v = off_timeout;
    switch (mode) {
      case 0:                                   // automatic switch-off timeout
        if (v > 0) v -= 2;
        pEnc->reset(v, 0, 28, 1, 0, false);
        break;
      case 1:                                   // Celsius / Farenheit
        pEnc->reset(cels, 0, 1, 1, 0, true);
        break;
      case 2:
        if (calib) return calib;
        break;
      case 3:                                   // Tune potentiometer
        if (next) return next;
        break;
      case 4:                                   // Save configuration data
        menu_long();
      case 5:                                   // Return to the main menu
        if (main) return main;
        return this;
      case 6:
        pCfg->setDefaults(true);
        changed = false;
        if (nextL) return nextL;
        return this;
    }
    tune = true;
  }
  forceRedraw();
  return this;
}

SCREEN* configSCREEN::menu_long(void) {
  if (nextL) {
    if (changed) {
      pCfg->saveConfig(off_timeout, cels);
    }
    return nextL;
  }
  return this;
}

//---------------------------------------- class calibSCREEN [ tip calibration ] -------------------------------
class calibSCREEN : public SCREEN {
  public:
    calibSCREEN(IRON* Iron, DSPL* DSP, ENCODER* Enc, IRON_CFG* Cfg, BUZZER* Buzz) {
      pIron = Iron;
      pD    = DSP;
      pEnc  = Enc;
      pCfg  = Cfg;
      pBz   = Buzz;
      smode = S_CONFIG;
    }
    virtual void init(void);
    virtual void show(void);
    virtual void rotaryValue(int16_t value);
    virtual SCREEN* menu(void);
    virtual SCREEN* menu_long(void);
  private:
    uint16_t  selectTemp(void);                 // Calculate the value of the temperature limit depending on mode
    IRON*     pIron;                            // Pointer to the IRON instance
    DSPL*     pD;                               // Pointer to the DSPLay instance
    ENCODER*  pEnc;                             // Pointer to the rotary encoder instance
    IRON_CFG* pCfg;                             // Pointer to the config instance
    BUZZER*   pBz;                              // Pointer to the buzzer instance
    byte      mode;                             // Which parameter to change: t_min, t_mid, t_max
    uint16_t  real_temp[2][3];                  // Real temperature measured at each of calibration points (real_temp, internal_temp)
    uint16_t  preset_temp;                      // The preset temp in human readable units
    bool      cels;                             // Current celsius/farenheit;
    bool      ready;                            // Whether the temperature has been established
    bool      tune;                             // Whether the parameter is modifiying
    bool      show_current;                     // Whether show the current temperature
    const uint32_t period = 1000;               // Update screen period
    
     
};

void calibSCREEN::init(void) {
  mode = 0;
  pEnc->reset(mode, 0, 2, 1, 0, true);          // 0 - temp_tip[0], 1 - temp_tip[1], 2 - temp_tip[2]
  pIron->switchPower(false);
  tune  = false;
  ready = false;
  show_current = true;
  for (byte i = 0; i < 3; ++i)
    real_temp[0][i] = temp_tip[i];
  pCfg->getCalibrationData(real_temp[1][0], real_temp[1][1], real_temp[1][2]);
  cels        = pCfg->getTempUnits();
  pD->clear();
  pD->msgOff();
  uint16_t temp = selectTemp();
  pD->tSet(temp, pCfg->getTempUnits());
  preset_temp = pIron->getTemp();               // Save the preset temperature in humen readable units
  preset_temp = pCfg->tempHuman(preset_temp);
  forceRedraw();
}

void calibSCREEN::show(void) {
  if (millis() < update_screen) return;
  update_screen = millis() + period;
  int temp = pIron->tempAverage();
  int temp_set = pIron->getTemp();
  if (show_current) {                           // Show the current Iron temperature
    uint16_t tempH = pCfg->tempHuman(temp);     // Translate the temperature into human readable value
    pD->tCurr(tempH);
  }
  byte p = pIron->appliedPower();
  if (!pIron->isOn()) p = 0;
  pD->percent(p);
  if (tune && (abs(temp_set - temp) < 3) && (pIron->tempDispersion() <= 6))  {
    if (!ready) {
      pBz->shortBeep();
      pD->msgReady();
      ready = true;
    }
  }
  if (tune && !pIron->isOn()) {                 // The IRON was switched off by error
    pD->msgOff();
    tune  = false;
    ready = false;
  }
}

void calibSCREEN::rotaryValue(int16_t value) {
  update_screen = millis() + period;
  if (tune) {                                   // change the real value for the temperature
    if (ready) {
      pD->tCurr(value);
      show_current = false;                     // We have started to setup the temperature
    }
  } else {                                      // select the temperature to be calibrated, t_min, t_mid or t_max
    mode = value;
    uint16_t temp = selectTemp();
    pD->tSet(temp, pCfg->getTempUnits());
  }
}

SCREEN* calibSCREEN::menu(void) { 
  if (tune) {                                   // Calibrated value for the temperature limit jus has been setup
    tune = false;
    uint16_t r_temp = pEnc->read();             // Real temperature
    uint16_t temp   = pIron->tempAverage();     // The temperature on the IRON
    pIron->switchPower(false);
    pD->msgOff();
   if (!cels)                                   // Always save the human readable temperature in Celsius
    r_temp = map(r_temp, temp_minF, temp_maxF, temp_minC, temp_maxC);
    real_temp[0][mode] = r_temp;
    real_temp[1][mode] = temp;
    pEnc->reset(mode, 0, 2, 1, 0, true);        // The temperature limit has been adjusted, switch to select mode
  } else {
    tune = true;
    uint16_t temp = selectTemp();
    uint16_t minT = 100;                        // Minimum input temperature (Celsius)
    uint16_t maxT = 550;                        // Maximum input temperature (Celsius)
    if (!cels) {
      minT = map(minT, temp_minC, temp_maxC, temp_minF, temp_maxF);
      maxT = map(maxT, temp_minC, temp_maxC, temp_minF, temp_maxF);
    }
    pEnc->reset(temp, minT, maxT, 1, 5);
    tune = true;
    temp = pCfg->human2temp(temp);
    pIron->setTemp(temp);
    pIron->switchPower(true);
    pD->msgOn();
  }
  ready = false;
  show_current = true;
  forceRedraw();
  return this;
}

SCREEN* calibSCREEN::menu_long(void) {
  pIron->switchPower(false);
  // temp_tip - array of calibration temperatures in Celsius
  uint16_t t_min = map(temp_tip[0], real_temp[0][0], real_temp[0][1], real_temp[1][0], real_temp[1][1]);
  uint16_t t_mid = map(temp_tip[1], real_temp[0][0], real_temp[0][1], real_temp[1][0], real_temp[1][1]);
  t_mid += map(temp_tip[1], real_temp[0][1], real_temp[0][2], real_temp[1][1], real_temp[1][2]) + 1;
  t_mid >>= 1;
  uint16_t t_max = map(temp_tip[2], real_temp[0][1], real_temp[0][2], real_temp[1][1], real_temp[1][2]);
  pCfg->saveCalibrationData(t_min, t_mid, t_max);
  pCfg->savePresetTempHuman(preset_temp);
  uint16_t temp = pCfg->human2temp(preset_temp);
  pIron->setTemp(temp);
  if (nextL) return nextL;
  return this;
}

uint16_t calibSCREEN::selectTemp(void) {
  uint16_t temp_calib = temp_tip[mode];         // Global variable
  if (!cels)                                    // Translate the temperature into the Farenheits
    temp_calib = map(temp_calib, temp_minC, temp_maxC, temp_minF, temp_maxF);
  return temp_calib;
}

//---------------------------------------- class tuneSCREEN [tune the potentiometer ] --------------------------
class tuneSCREEN : public SCREEN {
  public:
    tuneSCREEN(IRON* Iron, DSPL* DSP, ENCODER* ENC, BUZZER* Buzz, IRON_CFG* CFG) {
      pIron = Iron;
      pD    = DSP;
      pEnc  = ENC;
      pBz   = Buzz;
      pCfg  = CFG;
      smode = S_CONFIG;
    }
    virtual void init(void);
    virtual SCREEN* menu(void);
    virtual SCREEN* menu_long(void);
    virtual void show(void);
    virtual void rotaryValue(int16_t value);
  private:
    IRON*     pIron;                            // Pointer to the IRON instance
    DSPL*     pD;                               // Pointer to the display instance
    ENCODER*  pEnc;                             // Pointer to the rotary encoder instance
    BUZZER*   pBz;                              // Pointer to the simple Buzzer instance
    IRON_CFG* pCfg;                             // Pointer to the IRON config
    bool      arm_beep;                         // Whether beep is armed
    byte      max_power;                        // Maximum possible power to be applied
    const uint16_t period = 1000;               // The period in ms to update the screen
};

void tuneSCREEN::init(void) {
  pIron->switchPower(false);
  max_power = pIron->getMaxFixedPower();
  pEnc->reset(75, 0, max_power, 1, 5);          // Rotate the encoder to change the power supplied
  arm_beep = false;
  pD->clear();
  pD->msgTune();
  forceRedraw();
}

void tuneSCREEN::rotaryValue(int16_t value) {
  pIron->fixPower(value);
  forceRedraw();
}

void tuneSCREEN::show(void) {
  if (millis() < update_screen) return;
  update_screen = millis() + period;
  int16_t temp = pIron->tempAverage();
  pD->tCurr(temp);
  byte power = pEnc->read();                    // applied power
  if (!pIron->isOn())
    power = 0;
  else
    power = map(power, 0, max_power, 0, 100);
  pD->percent(power);
  if (arm_beep && (pIron->tempDispersion() < 5)) {
    pBz->shortBeep();
    arm_beep = false;
  }
}
  
SCREEN* tuneSCREEN::menu(void) {                // The rotary button pressed
  if (pIron->isOn()) {
    pIron->fixPower(0);
  } else {
    byte power = pEnc->read();                  // applied power
    pIron->fixPower(power);
  }
  return this;
}

SCREEN* tuneSCREEN::menu_long(void) {
  pIron->fixPower(0);                           // switch off the power
  if (next) return next;
  return this;
}

//---------------------------------------- class pidSCREEN [tune the PID coefficients] -------------------------
class pidSCREEN : public SCREEN {
  public:
    pidSCREEN(IRON* Iron, ENCODER* ENC) {
      pIron = Iron;
      pEnc  = ENC;
      smode = S_CONFIG;
    }
    virtual void init(void);
    virtual SCREEN* menu(void);
    virtual SCREEN* menu_long(void);
    virtual void show(void);
    virtual void rotaryValue(int16_t value);
  private:
    IRON*    pIron;                             // Pointer to the IRON instance
    ENCODER* pEnc;                              // Pointer to the rotary encoder instance
    byte     mode;                              // Which temperature to tune [0-3]: select, Kp, Ki, Kd
    uint32_t update_screen;                     // Time in ms when update the screen (print nre info)
    int      temp_set;
    const uint16_t period = 500;
};

void pidSCREEN::init(void) {
  temp_set = pIron->getTemp();
  mode = 0;                                     // select the element from the list
  pEnc->reset(1, 1, 4, 1, 1, true);             // 1 - Kp, 2 - Ki, 3 - Kd, 4 - temp 
  Serial.println("Select the coefficient (Kp)");
}

void pidSCREEN::rotaryValue(int16_t value) {
  if (mode == 0) {                              // No limit is selected, list the menu
    Serial.print("[");
    for (byte i = 1; i < 4; ++i) {
      int k = pIron->changePID(i, -1);
      Serial.print(k, DEC);
      if (i < 3) Serial.print(", ");
    }
    Serial.print("]; ");
    switch (value) {
      case 1:
        Serial.println("Kp");
        break;
      case 2:
        Serial.println("Ki");
        break;
      case 4:
        Serial.println(F("Temp"));
        break;
      case 3:
      default:
       Serial.println("Kd");
       break;
    }
  } else {
    switch (mode) {
      case 1:
        Serial.print(F("Kp = "));
        pIron->changePID(mode, value);
        break;
      case 2:
        Serial.print(F("Ki = "));
        pIron->changePID(mode, value);
        break;
      case 4:
        Serial.print(F("Temp = "));
        temp_set = value;
        pIron->setTemp(value);
        break;
      case 3:
      default:
        Serial.print(F("Kd = "));
        pIron->changePID(mode, value);
        break;
    }
    Serial.println(value);
  }
}

void pidSCREEN::show(void) {
  if (millis() < update_screen) return;
  update_screen = millis() + period;
  if (pIron->isOn()) {
    char buff[60];
    int temp    = pIron->getCurrTemp();
    uint16_t td = pIron->tempDispersion();
    uint16_t pd = pIron->powerDispersion();
    sprintf(buff, "%3d: td = %3d, pd = %3d --- ", temp_set - temp, td, pd);
    Serial.println(buff);
    //if ((temp_set - temp) > 30) Serial.println("");
  }
}

SCREEN* pidSCREEN::menu(void) {                 // The rotary button pressed
  if (mode == 0) {                              // select upper or lower temperature limit
    mode = pEnc->read();
    if (mode != 4) {
      int k = pIron->changePID(mode, -1);
      pEnc->reset(k, 0, 5000, 1, 5);
    } else {
      pEnc->reset(temp_set, 0, 970, 1, 5);
    }
  } else {                                      // upper or lower temperature limit just setup     
    mode = 0;
    pEnc->reset(1, 1, 4, 1, 1, true);           // 1 - Kp, 2 - Ki, 3 - Kd, 4 - temp
  }
  return this;
}

SCREEN* pidSCREEN::menu_long(void) {
  bool on = pIron->isOn();
  pIron->switchPower(!on);
  if (on)
    Serial.println("The iron is OFF");
  else
    Serial.println("The iron is ON");
  return this;
}
//=================================== End of class declarations ================================================

DSPL       disp(LCD_RS_PIN, LCD_E_PIN, LCD_DB4_PIN, LCD_DB5_PIN, LCD_DB6_PIN, LCD_DB7_PIN);
ENCODER    rotEncoder(R_MAIN_PIN, R_SECD_PIN);
BUTTON     rotButton(R_BUTN_PIN);
IRON       iron(heaterPIN, probePIN);
IRON_CFG   ironCfg;
 
BUZZER     simpleBuzzer(buzzerPIN);

mainSCREEN   offScr(&iron, &disp, &rotEncoder, &simpleBuzzer, &ironCfg);
workSCREEN   wrkScr(&iron, &disp, &rotEncoder, &simpleBuzzer, &ironCfg);
errorSCREEN  errScr(&disp, &simpleBuzzer);
powerSCREEN  powerScr(&iron, &disp, &rotEncoder, &ironCfg);
configSCREEN cfgScr(&iron, &disp, &rotEncoder, &ironCfg);
calibSCREEN  tipScr(&iron, &disp, &rotEncoder, &ironCfg, &simpleBuzzer);
tuneSCREEN   tuneScr(&iron, &disp, &rotEncoder, &simpleBuzzer, &ironCfg);
//pidSCREEN    pidScr(&iron, &rotEncoder);

SCREEN *pCurrentScreen = &offScr;
//SCREEN *pCurrentScreen = &pidScr;

/*
 * The timer1 overflow interrupt handler.
 * Activates the procedure for IRON temperature check @31250 Hz
 * keepTemp() function takes about 160 mks, 5 ticks
 */
const uint32_t period_ticks = (31250 * check_period)/1000-33-5;
// ISR(TIMER1_OVF_vect) {
//   if (iron_off) {                                     // The IRON is switched off, we need to chack the temperature
//     if (++tmr1_count >= 33) {                         // about 1 millisecond
//       TIMSK1 &= ~_BV(TOIE1);                          // disable the overflow interrupts
//       iron.keepTemp();                                // Check the temp. If on, keep the temperature
//       tmr1_count = 0;
//       iron_off = false;
//       TIMSK1 |= _BV(TOIE1);                           // enable the the overflow interrupts
//     }
//   } else {                                            // The IRON is on, check the curent and switch-off the IRON
//     if (++tmr1_count >= period_ticks) {
//       TIMSK1 &= ~_BV(TOIE1);                          // disable the overflow interrupts
//       tmr1_count = 0;
//       OCR1B      = 0;                                 // Switch-off the power to check the temperature
//       iron_off   = true;
//       TIMSK1    |= _BV(TOIE1);                        // enable the overflow interrupts
//     }
//   }
// }

void rotEncChange(void) {
  rotEncoder.changeINTR();
}

void rotPushChange(void) {
  rotButton.changeINTR();
}

// the setup routine runs once when you press reset:
void setup() {
  Serial.begin(9600);//TODO
  pinMode(paddlePIN, INPUT_PULLUP);
  simpleBuzzer.init();
  simpleBuzzer.shortBeep();
  simpleBuzzer.failedBeep();
  initADC();
  disp.init();

  // Load configuration parameters
  ironCfg.init();
  iron.init();
  uint16_t temp = ironCfg.tempPreset();
  iron.setTemp(temp);

  // Initialize rotary encoder
  rotEncoder.init();
  rotButton.init();
  delay(500);
  attachInterrupt(digitalPinToInterrupt(R_MAIN_PIN), rotEncChange,   CHANGE);
  attachInterrupt(digitalPinToInterrupt(R_BUTN_PIN), rotPushChange,  CHANGE);

  // Initialize SCREEN hierarchy
  offScr.next    = &wrkScr;
  offScr.nextL   = &cfgScr;
  wrkScr.next    = &offScr;
  wrkScr.nextL   = &powerScr;
  wrkScr.main    = &offScr;
  errScr.next    = &offScr;
  errScr.nextL   = &offScr;
  powerScr.nextL = &wrkScr;
  cfgScr.next    = &tuneScr;
  cfgScr.nextL   = &offScr;
  cfgScr.main    = &offScr;
  cfgScr.calib   = &tipScr;
  tipScr.nextL   = &offScr;
  tuneScr.next   = &cfgScr;
  tuneScr.main   = &offScr;
  pCurrentScreen->init();

}

// The main loop
void loop() {
  static int16_t old_pos = rotEncoder.read();
  checkPaddle();
  if ( paddleDown  && (pCurrentScreen != &wrkScr)){
        pCurrentScreen = &wrkScr;
        pCurrentScreen->init();
  }
  iron.checkIron();       // Periodically check that the IRON works correctrly
  // Serial.print("getTemp()=");
  // Serial.println(iron.getTemp());
  iron.keepTemp();      
  //delay(1000);//TODO test
  bool iron_on = iron.isOn();
  //dpB("iron_on ", iron_on);
  if ((pCurrentScreen == &wrkScr) && !iron_on) {  // the soldering iron failed
    pCurrentScreen = &errScr;
    pCurrentScreen->init();
  }
  

  SCREEN* nxt = pCurrentScreen->returnToMain();
  if (nxt != pCurrentScreen) {                  // return to the main screen by timeout
    pCurrentScreen = nxt;
    pCurrentScreen->init();
  }

  byte bStatus = rotButton.intButtonStatus();
  switch (bStatus) {
    case 2:                                     // long press;
      nxt = pCurrentScreen->menu_long();
      if (nxt != pCurrentScreen) {
        pCurrentScreen = nxt;
        pCurrentScreen->init();
      } else {
        if (pCurrentScreen->isSetup())
         pCurrentScreen->resetTimeout();
      }
      break;
    case 1:                                     // short press
      nxt = pCurrentScreen->menu();
      if (nxt != pCurrentScreen) {
        pCurrentScreen = nxt;
        pCurrentScreen->init();
      } else {
        if (pCurrentScreen->isSetup())
         pCurrentScreen->resetTimeout();
      }
      break;
    case 0:                                     // Not pressed
    default:
      break;
  }

  int16_t pos = rotEncoder.read();
  if (old_pos != pos) {
    pCurrentScreen->rotaryValue(pos);
    old_pos = pos;
    if (pCurrentScreen->isSetup())
     pCurrentScreen->resetTimeout();
  }

  pCurrentScreen->show();
}

