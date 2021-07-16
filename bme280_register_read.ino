#include <Wire.h>
// this sketch polls data out of the pressure/barometer sensor GY-BME280
// the direct register polling method was chosen to remove the need of external
// libraries(like adafruit) that will potentially slow down the execution of the 
// integrated program. Refer to the bst-bme280-ds002.pdf datasheet to 
// understand the use of the registers!
// Wiring: wire the SCL/SDA pins to the respective pins in arduino board  
// wire the vcc to 3.3V(NOT 5!) and the gnd to the respective gnd 
// optionally, pull-up the lines of SDA,SCL and the pin of CSB with 10K resistors.  
// Leave SDO unwired(needed only for SPI interfacing) 
// Raf


//8-bit registers of the BME280
#define humidity_lsb 0xfe
#define humidity_msb 0xFD

#define temperature_xlsb 0xFC  // the xlsb is used only for extra accuracy(and not needed in our case)
#define temperature_lsb 0xFB
#define temperature_msb 0xFA

#define pressure_xlsb 0xF9  // the xlsb is used only for extra accuracy as above 
#define pressure_lsb 0xF8
#define pressure_msb 0xf7

#define config_register 0xF5
#define ctrl_meas 0xf4
#define status_register 0xf3
#define ctrl_humidity 0xf2

#define reset_register 0xe0
#define chip_id 0xD0

// the hardcoded i2c address of the sensor(found with arduino 12c scanner sketch)
int BME280adress = 0x76;

void setup() {
  Serial.begin(115200);
  Wire.begin();  //initiate the wire library(needed!)
  set_register(BME280adress,reset_register,B10110110);   //   B10110110 is the required sequence to reset the device
  set_register(BME280adress,config_register,B01001000);
  set_register(BME280adress,ctrl_meas, B01101101);  // do one forced measurement cycle
  int hum_ctrl = read_register(BME280adress,ctrl_humidity);
  hum_ctrl = hum_ctrl | 100;  // change only the last 3 digits
  set_register(BME280adress,ctrl_humidity ,hum_ctrl);  // the humidity sensor needs to be activated(carefull:update after ctrl_meas!!)
}
 
 
int temp1,temp2,temp3;
int pres1,pres2,pres3;
int hum1, hum2;

 
void loop() {  
  set_register(BME280adress,ctrl_meas, B01101101);  // do one forced measurement cycle

  //initiate a transmission and  ask for the data from the following registers
  read_register(BME280adress,ctrl_meas);
  read_register(BME280adress,status_register);
  read_register(BME280adress,ctrl_humidity);
  
  // r e a d   t e m p e r a t u r e
  temp1 = read_register(BME280adress,temperature_msb);
  temp2 = read_register(BME280adress,temperature_lsb);
  temp3 = read_register(BME280adress,temperature_xlsb);
  temp3 = temp3 & 11110000;  // only the 4 first bits are used
  long int temperature = temp1;  // long to fit the 16+4 bit number we shove into temperature
  temperature = (temperature << 8) | temp2;  // shift bits to make place for the next 8 bits to be included with logical OR ("|")
  float temperature_float = (float(temperature)+ (float(temp3)/16))/1000;
  Serial.print("Temperature: "); Serial.print(temperature_float,5); Serial.print(" °C, ");
  
  // r e a d   p r e s s u r e
  pres1 = read_register(BME280adress,pressure_msb);
  pres2 = read_register(BME280adress,pressure_lsb);
  pres3 = read_register(BME280adress,pressure_xlsb);
  pres3 = pres3 & 11110000;  // keep only the first 4 digits of the register(see datasheet)
  long int pressure = pres1;  // 32 bit capacity
  pressure = (pressure<<8) | pres2;
  float pressure_float = float(pressure)/256;
  Serial.print("Pres.: "); Serial.print(pressure_float,5);Serial.print(" Pa, ");

  // r e a d    h u m i d i t y
  hum1 = read_register(BME280adress,humidity_msb);
  hum2 = read_register(BME280adress,humidity_lsb);
  unsigned int humidity = hum1 * 256 + hum2;
  Serial.print("Humidity: ");
  Serial.print(humidity);
  Serial.println( " % RH \n");

  delay(100);

  Serial.write(12);
  Serial.println( "\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
}


int read_register(int ic_address,int reg_address){
  Wire.beginTransmission(ic_address);
  Wire.write(reg_address);
  Wire.endTransmission();
  int returned_value;
  Wire.requestFrom(ic_address,byte(1));
  if( Wire.available() == 1)  // check if there are 2 available bytes
    returned_value = Wire.read();
  else
    return -1;

  return returned_value;
  }

  
void set_register(int ic_address, int reg_address, byte bytes_to_write){
  Wire.beginTransmission(ic_address);
  Wire.write(reg_address);
  Wire.write(bytes_to_write); 
  Wire.endTransmission();
  }
