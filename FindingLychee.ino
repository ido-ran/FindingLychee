#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Wire.h> //I2C library

/*
   Author: Ido Ran <ido.ran@gmail.com>
   This code uses NEO-6M GPS to track location and store the points into an EEPROM 24C256 chip.
   There is a serial interface to export the data and erase the memory.
*/
static const int RXPin = 3, TXPin = 4;
static const uint32_t GPSBaud = 9600;

// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);

struct Time
{
  byte hour;
  byte minute;
  byte second;
};

struct LocationItem
{
  float lat;
  float lng;
  Time time;
};

//////////////////////////////////// EEPROM Memory functions //////////////////////////////////////

const unsigned int SERIALIZED_LENGTH = 10; // float (4 bytes) + float (4 bytes) + time (2 bytes)
const unsigned int MAX_ITEMS_IN_MEMORY = 3275;

void i2c_eeprom_write_byte( int deviceaddress, unsigned int eeaddress, byte data )
{
  // Read the byte in memory, if it's the same do not overwrite it to reduce wear.
  byte memoryByte = i2c_eeprom_read_byte(deviceaddress, eeaddress);
  if (memoryByte == data) return;

  int rdata = data;
  Wire.beginTransmission(deviceaddress);
  Wire.write((int)(eeaddress >> 8));    // Address High Byte
  Wire.write((int)(eeaddress & 0xFF));  // Address Low Byte
  Wire.write(rdata);
  Wire.endTransmission();
  delay(10);
}

byte i2c_eeprom_read_byte( int deviceaddress, unsigned int eeaddress )
{
  byte rdata = 0xFF;
  Wire.beginTransmission(deviceaddress);
  Wire.write((int)(eeaddress >> 8));    // Address High Byte
  Wire.write((int)(eeaddress & 0xFF));  // Address Low Byte
  Wire.endTransmission();
  Wire.requestFrom(deviceaddress, 1);
  if (Wire.available()) rdata = Wire.read();
  return rdata;
}

void write_time(unsigned int address, struct Time &time)
{
  int hour12 = time.hour <= 12 ? time.hour : time.hour - 12;
  int packedTime = hour12 | time.minute << 4 | time.second << 10;
  write_int(address, packedTime);
}

void read_time(unsigned int address, struct Time &time)
{
  int packedTime;
  read_int(address, packedTime);
  time.hour = packedTime & 0xF; // 4 bits (0-12)
  time.minute = (packedTime >> 4) & 0x3F; // 6 bits (0-59)
  time.second = (packedTime >> 10) & 0x3F; // 6 bits (0-59)
}

void write_int(unsigned int address, int &i)
{
  byte* pb = (byte*) &i;
  i2c_eeprom_write_byte(0x50, address + 0, pb[0]);
  i2c_eeprom_write_byte(0x50, address + 1, pb[1]);
}

void read_int(unsigned int address, int &i)
{
  byte* pb = (byte*) &i;
  pb[0] = i2c_eeprom_read_byte(0x50, address + 0);
  pb[1] = i2c_eeprom_read_byte(0x50, address + 1);
}

void write_float(unsigned int address, float &f)
{
  byte* pb = (byte*) &f;
  for (int i = 0; i < 4; i++)
    i2c_eeprom_write_byte(0x50, address + i, pb[i]);
}

void read_float(unsigned int address, float &f)
{
  byte* pb = (byte*) &f;
  for (int i = 0; i < 4; i++)
    pb[i] = i2c_eeprom_read_byte(0x50, address + i);
}

void write_location_item(unsigned int index, struct LocationItem &loc)
{
  int field_addr = 2 + index * SERIALIZED_LENGTH;
  write_float(field_addr, loc.lng);
  field_addr += sizeof(loc.lng);
  write_float(field_addr, loc.lat);
  field_addr += sizeof(loc.lat);
  write_time(field_addr, loc.time);
}

void read_location_item(unsigned int index, struct LocationItem &loc)
{
  int field_addr = 2 + index * SERIALIZED_LENGTH;
  read_float(field_addr, loc.lng);
  field_addr += sizeof(loc.lng);
  read_float(field_addr, loc.lat);
  field_addr += sizeof(loc.lat);
  read_time(field_addr, loc.time);
}

void add_location_item(struct LocationItem loc)
{
  int i = 0;
  read_int(0, i);
  Serial.print("read i="); Serial.print(i);
  Serial.print("MAX="); Serial.print(MAX_ITEMS_IN_MEMORY);
  Serial.println();

  if (i < MAX_ITEMS_IN_MEMORY)
  {
    write_location_item(i, loc);
  
    i++;
    write_int(0, i);
  }
}

//////////////////////////////// GPS Functions /////////////////////////////////

void gps_tracking()
{
  // First ensure the we got GPS fix.
  if (!gps.location.isValid())
  {
    Serial.print("* ");
    printDateTime(gps.date, gps.time);
    Serial.println();
  }
  else
  {
    struct LocationItem loc = {
      .lat = gps.location.lat(),
      .lng = gps.location.lng(),
      .time = {
        .hour = gps.time.hour(),
        .minute = gps.time.minute(),
        .second = gps.time.second()
      }
    };
    add_location_item(loc);
  }

  smartDelay(3000);

  if (millis() > 5000 && gps.charsProcessed() < 10)
    Serial.println(F("No GPS data received: check wiring"));
}

// This custom version of delay() ensures that the gps object
// is being "fed".
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}

//////////////////////////////// Application ///////////////////////////////////

static const int MODE_TRACKING = 0;
static const int MODE_COMMAND = 1;

static const int COMMAND_HELP = 0;
static const int COMMAND_EXPORT = 1;
static const int COMMAND_RESET = 2;

int mode = MODE_TRACKING;

void execute_command_help()
{
  Serial.println("FindingLychee 1.0.0 by Ido Ran");
  Serial.println("0 - help");
  Serial.println("1 - export");
  Serial.println("2 - reset");
}

void execute_command_export()
{
  int itemsCount = 0;
  read_int(0, itemsCount);
  Serial.print("read itemsCount="); Serial.println(itemsCount);

  struct LocationItem loc;
  for (int index = 0; index < itemsCount; index++)
  {
    read_location_item(index, loc);
    print_location_item(loc);
  }  
}

void print_location_item(const struct LocationItem &v)
{
  Serial.print(v.lng, 6); Serial.print(",");
  Serial.print(v.lat, 6); Serial.print(",");
  Serial.print(v.time.hour); Serial.print(":");
  Serial.print(v.time.minute); Serial.print(":");
  Serial.print(v.time.second);
  Serial.println();  
}

void execute_command_reset()
{
  // To be on the safe side, export anything in memory to Serial before erase it.
  execute_command_export();
  
  // Reset item counter to zero.
  int zeroItems = 0;
  write_int(0, zeroItems);
  
  read_int(0, zeroItems);
  Serial.println("Memory reset");
  Serial.print("i="); Serial.println(zeroItems);
}

void execute_command(unsigned int command)
{
  switch (command)
  {
    case COMMAND_HELP:
      execute_command_help();
      break;
    case COMMAND_EXPORT:
      execute_command_export();
      break;
    case COMMAND_RESET:
      execute_command_reset();
      break;
  }
}

void process_serial_command()
{
  if (Serial.available())
  {
    char inChar = (char)Serial.read();
    Serial.print(">>>> "); Serial.println(inChar);
    if (inChar >= '0' && inChar <= '9')
    {
      unsigned int command = inChar - '0';
      execute_command(command);
    }
  }
}

void setup()
{
  Serial.begin(115200);
  ss.begin(GPSBaud);
  
  Wire.begin(); // initialise the connection

  Serial.println(F("Finding Lychee 1.0.0"));
  Serial.print(F("TinyGPS++ library v. ")); Serial.println(TinyGPSPlus::libraryVersion());
  Serial.println(F("by Ido Ran"));
}

void loop()
{
  if (mode == MODE_TRACKING)
  {
    gps_tracking();
  }
  else if (mode == MODE_COMMAND)
  {
    process_serial_command();
  }
  
  if (Serial.available())
  {
    // Change to command mode which stop track GPS location
    mode = MODE_COMMAND;
  }
}

static void printFloat(float val, bool valid, int len, int prec)
{
  if (!valid)
  {
    while (len-- > 1)
      Serial.print('*');
    Serial.print(' ');
  }
  else
  {
    Serial.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i = flen; i < len; ++i)
      Serial.print(' ');
  }
  smartDelay(0);
}

static void printInt(unsigned long val, bool valid, int len)
{
  char sz[32] = "*****************";
  if (valid)
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i = strlen(sz); i < len; ++i)
    sz[i] = ' ';
  if (len > 0)
    sz[len - 1] = ' ';
  Serial.print(sz);
  smartDelay(0);
}

static void printDateTime(TinyGPSDate &d, TinyGPSTime &t)
{
  if (!d.isValid())
  {
    Serial.print(F("********** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d/%02d/%02d ", d.month(), d.day(), d.year());
    Serial.print(sz);
  }

  if (!t.isValid())
  {
    Serial.print(F("******** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d:%02d:%02d ", t.hour(), t.minute(), t.second());
    Serial.print(sz);
  }

  printInt(d.age(), d.isValid(), 5);
  smartDelay(0);
}

static void printStr(const char *str, int len)
{
  int slen = strlen(str);
  for (int i = 0; i < len; ++i)
    Serial.print(i < slen ? str[i] : ' ');
  smartDelay(0);
}
