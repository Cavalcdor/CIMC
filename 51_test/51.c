#include <STC8G.h>
#include <intrins.h>

// PWM value definitions for servo control (Unit: ticks)
// PCA9685 resolution: 4096, 50Hz (20ms period)
// PWM value = pulse width (us) / (20ms/4096) = pulse width (us) / 4.8828125
// Adjusted to a more moderate 230-370 range for safer motion.

#define RELAX_SERVO0  240
#define RELAX_SERVO1  370
#define RELAX_SERVO2  370
#define RELAX_SERVO3  230
#define RELAX_SERVO4  370
#define RELAX_SERVO5  240
#define RELAX_SERVO6  370
#define RELAX_SERVO7  240

// B - Spread (摊开) - all fingers open
#define SPREAD_SERVO0  240
#define SPREAD_SERVO1  370
#define SPREAD_SERVO2  370
#define SPREAD_SERVO3  230
#define SPREAD_SERVO4  370
#define SPREAD_SERVO5  240
#define SPREAD_SERVO6  370
#define SPREAD_SERVO7  240

// C - Fist (握拳)
#define FIST_SERVO0   360
#define FIST_SERVO1   230   
#define FIST_SERVO2   280   
#define FIST_SERVO3   360   
#define FIST_SERVO4   240   
#define FIST_SERVO5   360   
#define FIST_SERVO6   240   
#define FIST_SERVO7   360   

// D - Point Index (伸食指) - only index extended
#define POINT_SERVO0   360
#define POINT_SERVO1   230
#define POINT_SERVO2   370
#define POINT_SERVO3   230
#define POINT_SERVO4   240
#define POINT_SERVO5   360
#define POINT_SERVO6   240
#define POINT_SERVO7   360

// E - Thumb Up (伸拇指) - only thumb extended
#define THUMBUP_SERVO0  240
#define THUMBUP_SERVO1  370
#define THUMBUP_SERVO2  280
#define THUMBUP_SERVO3  360
#define THUMBUP_SERVO4  240
#define THUMBUP_SERVO5  360
#define THUMBUP_SERVO6  240
#define THUMBUP_SERVO7  360

// PCA9685 I2C address (A0-A5 selectable)
#define PCA9685_ADDR  0x80   // 8-bit address, shifted, actual 7-bit is 0x40

// PCA9685 register addresses
#define MODE1         0x00
#define MODE2         0x01
#define PRESCALE      0xFE
#define LED0_ON_L     0x06
#define LED0_ON_H     0x07
#define LED0_OFF_L    0x08
#define LED0_OFF_H    0x09

// Servo mapping: 0-1:Thumb, 2-3:Index, 4-5:Middle, 6-7:Pinky
// Test range: adjust these values to find min/max angles for each servo
#define SERVO_MIN  200   // Minimum PWM (adjust per servo if needed)
#define SERVO_MAX  400   // Maximum PWM (adjust per servo if needed)
#define SERVO_CENTER 300 // Neutral position

// Servo-specific min/max (all same initially, tune after testing)
unsigned short servo_min[8] = {200, 200, 200, 200, 200, 200, 200, 200};
unsigned short servo_max[8] = {400, 400, 400, 400, 400, 400, 400, 400};

// Function declarations
void delay_ms(unsigned int ms);
void I2C_Start(void);
void I2C_Stop(void);
void I2C_SendByte(unsigned char dat);
bit I2C_WaitAck(void);
void PCA9685_WriteReg(unsigned char reg, unsigned char val);
void PCA9685_SetPWM(unsigned char channel, unsigned short on, unsigned short off);
void PCA9685_Init(void);
void SetAllServos(unsigned short *pwm_values);
void SetSingleServo(unsigned char channel, unsigned short off);

// I2C interface pins
sbit SCL = P3^2;
sbit SDA = P3^3;

// Delay function (11.0592MHz, ~1ms)
void delay_ms(unsigned int ms)
{
    unsigned int i, j;
    for(i = 0; i < ms; i++)
        for(j = 0; j < 1100; j++);
}

// I2C start condition
void I2C_Start(void)
{
    SDA = 1;
    SCL = 1;
    _nop_(); _nop_();
    SDA = 0;
    _nop_(); _nop_();
    SCL = 0;
}

// I2C stop condition
void I2C_Stop(void)
{
    SDA = 0;
    SCL = 1;
    _nop_(); _nop_();
    SDA = 1;
    _nop_(); _nop_();
}

// Send one byte via I2C
void I2C_SendByte(unsigned char dat)
{
    unsigned char i;
    for(i = 0; i < 8; i++)
    {
        SDA = (dat & 0x80) ? 1 : 0;
        dat <<= 1;
        _nop_();
        SCL = 1;
        _nop_(); _nop_(); _nop_();
        SCL = 0;
    }
}

// Wait for ACK from slave
bit I2C_WaitAck(void)
{
    bit ack;
    SDA = 1;
    _nop_();
    SCL = 1;
    _nop_(); _nop_(); _nop_();
    ack = SDA;
    SCL = 0;
    return ack;
}

// Write to PCA9685 register
void PCA9685_WriteReg(unsigned char reg, unsigned char val)
{
    I2C_Start();
    I2C_SendByte(PCA9685_ADDR);
    I2C_WaitAck();
    I2C_SendByte(reg);
    I2C_WaitAck();
    I2C_SendByte(val);
    I2C_WaitAck();
    I2C_Stop();
}

// Set PWM for specific channel (on: start tick, off: end tick)
void PCA9685_SetPWM(unsigned char channel, unsigned short on, unsigned short off)
{
    unsigned char reg = LED0_ON_L + 4 * channel;
    PCA9685_WriteReg(reg, on & 0xFF);
    PCA9685_WriteReg(reg + 1, on >> 8);
    PCA9685_WriteReg(reg + 2, off & 0xFF);
    PCA9685_WriteReg(reg + 3, off >> 8);
}

// Initialize PCA9685 for 50Hz operation
void PCA9685_Init(void)
{
    unsigned char oldmode;
    // Enter sleep mode for configuration
    PCA9685_WriteReg(MODE1, 0x10);  // sleep
    // Set prescaler for 50Hz: 25MHz/(4096*50)-1 � 121
    PCA9685_WriteReg(PRESCALE, 121);
    // Configure MODE2: totem pole output
    PCA9685_WriteReg(MODE2, 0x04);
    // Exit sleep mode
    oldmode = 0x10;
    PCA9685_WriteReg(MODE1, oldmode & 0xEF);
    delay_ms(5);
}

// Set all 8 servos to specified PWM values (off time, on=0)
void SetAllServos(unsigned short *pwm_values)
{
    unsigned char i;
    for(i = 0; i < 8; i++)
    {
        PCA9685_SetPWM(i, 0, pwm_values[i]);
    }
}

// Set a single servo to a specific PWM value
void SetSingleServo(unsigned char channel, unsigned short off)
{
    PCA9685_SetPWM(channel, 0, off);
}

// Test: sweep a single servo between min and max to find its angle range
void TestSingleServo(unsigned char ch)
{
    unsigned short val;
    
    // Move servo ch to min → hold
    SetSingleServo(ch, servo_min[ch]);
    delay_ms(1500);
    
    // Sweep from min to max in steps
    for(val = servo_min[ch]; val <= servo_max[ch]; val += 10)
    {
        SetSingleServo(ch, val);
        delay_ms(300);
    }
    
    // Hold at max
    delay_ms(1000);
    
    // Sweep back from max to min
    for(val = servo_max[ch]; val >= servo_min[ch]; val -= 10)
    {
        SetSingleServo(ch, val);
        delay_ms(300);
    }
    
    // Back to center
    SetSingleServo(ch, SERVO_CENTER);
    delay_ms(500);
}

// Main program - standalone servo angle tester (no serial needed)
void main(void)
{
    unsigned char i;
    unsigned short neutral[8] = {SERVO_CENTER, SERVO_CENTER, SERVO_CENTER, SERVO_CENTER,
                                 SERVO_CENTER, SERVO_CENTER, SERVO_CENTER, SERVO_CENTER};
    
    P3M0 = 0x00;
    P3M1 = 0x00;
    P5M0 = 0x00;
    P5M1 = 0x00;
    
    I2C_Stop();
    delay_ms(10);
    PCA9685_Init();
    delay_ms(10);
    
    // Start: all servos to center
    SetAllServos(neutral);
    delay_ms(2000);
    
    // Test loop: test each servo individually, then repeat
    while(1)
    {
        for(i = 0; i < 8; i++)
        {
            TestSingleServo(i);
        }
    }
}