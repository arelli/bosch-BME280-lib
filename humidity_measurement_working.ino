#include <Wire.h>

#define hum_lsb 0xfe
#define hum_msb 0xFD

#define temp_xlsb 0xFC  
#define temp_lsb 0xFB
#define temp_msb 0xFA

#define press_xlsb 0xF9  
#define press_lsb 0xF8
#define press_msb 0xf7

#define config_register 0xF5
#define ctrl_meas 0xf4
#define status_register 0xf3
#define ctrl_humidity 0xf2

#define reset_register 0xe0
#define reset_seq 0xB6

#define chip_id 0xD0

#define BME280_ADDRESS 0x76


void write_register(byte reg, byte data){
    Wire.beginTransmission(BME280_ADDRESS);
    Wire.write(reg);
    Wire.write(data);
    Wire.endTransmission();
}


byte read_register(byte reg){
    byte response;
    Wire.beginTransmission(BME280_ADDRESS);
    Wire.write(reg);
    Wire.requestFrom(BME280_ADDRESS, 1);
    if(Wire.available() != 1){
      Wire.endTransmission();
      return '\x00';
    }
    response = Wire.read();
    Wire.endTransmission();

    return response;
}

void dump_registers(){
    Serial.print("hum_lsb = ");
    Serial.println(read_register(hum_lsb), HEX);

    Serial.print("hum_msb = ");
    Serial.println(read_register(hum_msb), HEX);


    Serial.print("temp_xlsb = ");
    Serial.println(read_register(temp_xlsb), HEX);    

    Serial.print("temp_lsb = ");
    Serial.println(read_register(temp_lsb), HEX);

    Serial.print("temp_msb = ");
    Serial.println(read_register(temp_msb), HEX);


    Serial.print("press_xlsb = ");
    Serial.println(read_register(press_xlsb), HEX);    

    Serial.print("press_lsb = ");
    Serial.println(read_register(press_lsb), HEX);

    Serial.print("press_msb = ");
    Serial.println(read_register(press_msb), HEX);
}


void setup(){
    Serial.begin(115200);
    Wire.begin();
    write_register(reset_register, reset_seq);
    write_register(ctrl_meas, 00100101);
    write_register(ctrl_humidity, 00000001);
}


void loop(){
    write_register(ctrl_meas, B00100101);
    delay(500);
    dump_registers();
    long int humidity_lsb = read_register(hum_lsb);
    long int humidity_msb = read_register(hum_msb);
    long int humidity = humidity_msb;
    humidity = (humidity<<8) | humidity_lsb;
    Serial.print("Humidity = ");
    Serial.print(humidity);
    Serial.print(" % RH \n");

}
