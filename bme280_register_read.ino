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
#define humidity_msb 0xfd

#define temperature_xlsb 0xFC  // the xlsb is used only for extra accuracy(and not needed in our case)
#define temperature_lsb 0xFB
#define temperature_msb 0xFA

#define pressure_xlsb 0xF9  // the xlsb is used only for extra accuracy as above 
#define pressure_lsb 0xF8
#define pressure_msb 0xf7

#define config_register 0xF5
#define ctrl_meas 0xf4
#define status_register 0xf3
#define ctrl_hum 0xf2

#define reset_register 0xe0
#define chip_id 0xD0

// the hardcoded i2c address of the sensor(found with arduino 12c scanner sketch)
int BME280adress = 0x76 ;

void setup() {
  Serial.begin(9600);
  Wire.begin();  //initiate the wire library(needed!)
  
//  set_register(BME280adress,reset_register,B10110110);   //   B10110110 is the required sequence to reset the device
//  delay(100);
  set_register(BME280adress,ctrl_meas, B00100111);  // do one forced measurement cycle, the only right way to address specific registers
  set_register(BME280adress,ctrl_hum ,B00000001);  // the humidity sensor needs to be activated(carefull:update after ctrl_meas!!)
  set_register(BME280adress,ctrl_meas, B00100111);
}
 
 
int temp1,temp2, temp3;
int pres1,pres2,pres3;
int hum1, hum2;

 
void loop() {  
  
  delay(50);  // wait for all the measurements to be done
  
  //initiate a transmission and  ask for the data from the following registers
  read_register(BME280adress,ctrl_meas, "control measuremetn register");
  read_register(BME280adress,status_register, "status_register");
  read_register(BME280adress,ctrl_hum, "***ctrl_hum***");

  // Get the temperature******************************************
  temp1 = read_register(BME280adress,temperature_msb, "temperature_msb");
  temp2 = read_register(BME280adress,temperature_lsb, "temperature_lsb");
  temp3 = read_register(BME280adress,temperature_xlsb, "temperature_xlsb");
  temp3 = temp3 & 11110000;  // only the 4 first bits are used
  //unsigned int temperature = temp1 * 256 + temp2;
  int temperature = temp1;
  temperature = (temperature << 8) | temp2;
  temperature = (temperature << 4) | temp3;
  float temperature_float = float(temperature)/1000;
  Serial.print("Temp.:"); Serial.print(temperature_float,4); Serial.print("°C,\n ");

  // Get the humidity*********************************************
  hum1 = read_register(BME280adress,humidity_msb, "humidity_msb");
  hum2 = read_register(BME280adress,humidity_lsb, "humidity_lsb");
  unsigned int humidity = float(hum1 *256 + hum2)/1024;  // humidity is in Q10.22 format -> divide by 1024
  Serial.print("Hum.: "); Serial.print(humidity, DEC); Serial.print("% RH, \n");
  
  // Get the pressure*********************************************
  pres1 = read_register(BME280adress,pressure_msb, "pressure_msb");
  pres2 = read_register(BME280adress,pressure_lsb, "pressure_lsb");
  pres3 = read_register(BME280adress,pressure_xlsb, "pressure_xlsb");
  pres3 = pres3 & 11110000;
  float pressure = float(pres1 + float(pres2)/256);  // is represented in Q24.8 format  -> divide by 256
  Serial.print("Pres.: "); Serial.print(pressure, 4);Serial.print(" Pa \n");
 
  delay(10000);

  // force mode, make 1 measurement the next loop
  //set_register(BME280adress,ctrl_meas, B00100101);  
}


int read_register(int ic_address,int reg_address, String reg_name){
  int returned_value;
  Wire.beginTransmission(ic_address);
  Wire.write(reg_address);
  
  Wire.requestFrom(ic_address,byte(1));
  if( Wire.available() == 1){  
    Serial.print(reg_name);
    Serial.print(" = ");
    returned_value = Wire.read();
    Wire.endTransmission();
    Serial.print(returned_value, BIN);
  }
  else
    return -1;

  Serial.print("\n");
  return returned_value;
  }

  
void set_register(int ic_address, int reg_address, byte bytes_to_write){
  Wire.beginTransmission(ic_address);
  Wire.write(reg_address);
  Wire.write(bytes_to_write); 
  Wire.endTransmission();
  }
