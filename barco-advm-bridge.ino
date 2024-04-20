/* Barco ADVM I2C Bridge
 * (2023) Martin Hejnfelt (martin@hejnfelt.com)
 *
 * For use with Barco/BarcoNet/Scientific Atlanta ADVM monitors.
 *
 * This changes the I2C communication from monitor MCUs
 * that disables IE1 and IE2 bits.
 *
 * Connect the main I2C line of the monitors PCB to the 
 * hardware I2C (TWI) pins (A4/SDA and A5/SCL) and then specify the desired
 * secondary I2C pins for the communication to the jungle, here D4 (SDA) and D5 (SCL) are used.
 * Remember pull-up resistors on the secondary I2C bus (D4 and D5 below)
 * 2.2-3.3K ones should work fine, just solder between the pin
 * and the Arduinos VCC pin.
 *
 * On the Barco ADVM14, SCL is R45 and SDA is R44.
 *
 * Connect the secondary I2C slave, PE0/SDA1 and PE1/SCL1 through a non-inverting buffer and a
 * 100 Ohm to the main I2C lines, so like this:
 *                   
 * Monitor SDA ----|>--[100]---- PE0
 * Monitor SCL ----|>--[100]---- PE1
 *
 * A small SOT-23 breakout and a SN74LVC34DBV seems a good fit. Power it from the Arduino's 5V/GND pins.
 *
 * This requires a board based on ATMega328PB as it has two I2C slaves. A suitable unit could be:
 * https://github.com/skumlos/atmega328pb-nano
 * 
 * Requires the SoftI2CMaster library
 * https://github.com/felias-fogg/SoftI2CMaster (or install through Library Manager)
 * 
 * Tested on:
 * Barco ADVM14 (branded BarcoNet) (v1.0)
 * Barco ADVM20 (branded Barco) (v1.0)
 * Barco ADVM20 (branded Scientific Atlanta) (v1.1)
 *
 * Expect your monitor to catch fire!
 */

// Version 1.1

#define I2C_TIMEOUT 500
#define I2C_PULLUP 0
#define I2C_FASTMODE 1

#define READ_DELTA_MS (80)

// Only have one active of these, depending on what you want.
#define RGB_SWITCH_REG_VAL (0x3) /* IE1 and IE2 enabled */
//#define RGB_SWITCH_REG_VAL (0xB) /* IE1, IE2 and YUV mode for RGB1 enabled, untested... */

// Uncomment this line if you have the late model Scientific Atlanta version without a TDA8540
//#define SCIENTIFIC_ATLANTA (1)

// These work for Nano 3.0 / Mini Pro
#define SDA_PORT PORTD
#define SDA_PIN 4 // = PD4
#define SCL_PORT PORTD
#define SCL_PIN 5 // = PD5

#define GPIO_BLANKING (13) // LED

#include <SoftI2CMaster.h>
#include <Wire.h>
#include <Wire1.h>

// Address is 10001X1Y, X=<AS pin>=0/GND on ADVM14, Y=R/W
#define TDA9321H_ADDR (69)
#define TDA8540_ADDR  (72)

#define REG_VIDEO_SWITCH0 (0x08)
#define REG_VIDEO_SWITCH1 (0x09)
#define REG_RGB_SWITCH    (0x0A)

#define REG_8540_SW1 (0x00)

#ifdef SCIENTIFIC_ATLANTA
bool coded1 = false;
#endif

bool cvbs9321 = false;
uint8_t input8540 = 0;

void writeRegister(const uint8_t reg, const uint8_t val) {
  i2c_start((TDA9321H_ADDR<<1)|I2C_WRITE);
  i2c_write(reg);
  i2c_write(val);
  i2c_stop(); 
}

enum WriteState {
  WR_REG,
  WR_DATA
};

WriteState writeState = WR_REG;

int currentReg = 0x0;
uint8_t w[15] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

void writeRequest9321(int byteCount) {
  for(int i=0; i<byteCount; ++i) {
    switch(writeState) {
      case WR_REG:
        currentReg = Wire.read();
        i2c_start((TDA9321H_ADDR<<1)|I2C_WRITE);
        i2c_write(currentReg);
        writeState = WR_DATA;
      break;
      case WR_DATA:
        w[currentReg] = Wire.read();
        switch(currentReg) {
          case REG_VIDEO_SWITCH0:
#ifdef SCIENTIFIC_ATLANTA
            coded1 = (w[currentReg] == 0x3);
#else
            cvbs9321 = w[currentReg] & 0x02;
#endif
          break;
          case REG_RGB_SWITCH:
            w[currentReg] |= RGB_SWITCH_REG_VAL;
          break;
        }
        i2c_write(w[currentReg++]);
      break;
    }
  }
  i2c_stop();
  writeState = WR_REG;
}

uint8_t r[4] = { 0x80, 0, 0, 0 };
void readRequest9321() {
  Wire.write(r,4);
}

void read() {
  i2c_start((TDA9321H_ADDR<<1)|I2C_READ);
  r[0] = i2c_read(false); // read one byte
  r[1] = i2c_read(false); // read one byte
  r[2] = i2c_read(false); // read one byte
  r[3] = i2c_read(true); // read one byte and send NAK to terminate
  i2c_stop(); // send stop condition
}

uint8_t w8540[3] = { 0, 0, 0 };
WriteState writeState8540 = WR_REG;
int currentReg8540 = 0x0;

void writeRequest8540(int byteCount) {
  for(int i=0; i<byteCount; ++i) {
    switch(writeState8540) {
      case WR_REG:
        currentReg8540 = Wire1.read();
        writeState8540 = WR_DATA;
      break;
      case WR_DATA:
        w8540[currentReg8540] = Wire1.read();
        switch(currentReg8540) {
          case REG_8540_SW1:
            // When CVBS input 1 / CODED A is selected,
            // 0x80 (1000 0000b) is written to SW1 register.
            // When CVBS input 2 / CODED B is selected,
            // 0xC0 (1100 0000b) is written to SW1 register.
            if(w8540[currentReg8540] & 0x40) {
              input8540 = 2;
            } else {
              input8540 = 1;
            }
          break;
        }
        ++currentReg8540;
      break;
    }
  }
  writeState8540 = WR_REG;
  currentReg8540 = 0x0;
}

void readRequest8540() {
  currentReg8540 = 0x0;
}

void setup() {
  bool iicinit = i2c_init();
  pinMode(GPIO_BLANKING,OUTPUT);
  digitalWrite(GPIO_BLANKING,LOW);
  Wire.begin(TDA9321H_ADDR);
  Wire.onReceive(writeRequest9321);
  Wire.onRequest(readRequest9321);

  Wire1.begin(TDA8540_ADDR);
  Wire1.onReceive(writeRequest8540);
  Wire1.onRequest(readRequest8540);
}

bool blanking = false; 
// Constantly read the status regs to be able to serve them back upon request
void loop() {
  read();
  #ifdef SCIENTIFIC_ATLANTA
  if(coded1 && !blanking) {
    digitalWrite(GPIO_BLANKING,HIGH);
    blanking = true;
  } else if(!coded1 && blanking){
    digitalWrite(GPIO_BLANKING,LOW);
    blanking = false;
  }
  #else
  if(input8540 == 1 && cvbs9321 && !blanking) {
    digitalWrite(GPIO_BLANKING,HIGH);
    blanking = true;
  } else if((input8540 != 1 || !cvbs9321) && blanking){
    digitalWrite(GPIO_BLANKING,LOW);
    blanking = false;
  }
  #endif
  delay(READ_DELTA_MS);
}
