#include <STC8G.h>
#include <intrins.h>

// PWM value definitions for servo control (Unit: ticks)
// PCA9685 resolution: 4096, 50Hz (20ms period)
// PWM value = pulse width (us) / (20ms/4096) = pulse width (us) / 4.8828125
// Adjusted to a more moderate 230-370 range for safer motion.

#define RELAX_SERVO0  280
#define RELAX_SERVO1  340
#define RELAX_SERVO2  370
#define RELAX_SERVO3  210
#define RELAX_SERVO4  290
#define RELAX_SERVO5  310
#define RELAX_SERVO6  350
#define RELAX_SERVO7  250

// B - Spread (摊开) - all fingers open
#define SPREAD_SERVO0  310
#define SPREAD_SERVO1  500
#define SPREAD_SERVO2  470
#define SPREAD_SERVO3  110
#define SPREAD_SERVO4  480
#define SPREAD_SERVO5  120
#define SPREAD_SERVO6  440
#define SPREAD_SERVO7  160

// C - Fist (握拳)
#define FIST_SERVO0   280
#define FIST_SERVO1   210   
#define FIST_SERVO2   260   
#define FIST_SERVO3   420   
#define FIST_SERVO4   180   
#define FIST_SERVO5   440   
#define FIST_SERVO6   200   
#define FIST_SERVO7   470   

// D - Point Index (伸食指) - only index extended
#define POINT_SERVO0   280
#define POINT_SERVO1   210
#define POINT_SERVO2   500
#define POINT_SERVO3   110
#define POINT_SERVO4   180
#define POINT_SERVO5   440
#define POINT_SERVO6   200
#define POINT_SERVO7   470

// E - Thumb Up (伸拇指) - only thumb extended
#define THUMBUP_SERVO0  310
#define THUMBUP_SERVO1  500
#define THUMBUP_SERVO2  260
#define THUMBUP_SERVO3  420
#define THUMBUP_SERVO4  180
#define THUMBUP_SERVO5  440
#define THUMBUP_SERVO6  200
#define THUMBUP_SERVO7  470

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

// Flags
bit ready_to_act = 0;       // Command received flag
unsigned char gesture_id = 0;  // 0:A(Relax), 1:B(Spread), 2:C(Fist), 3:D(Point), 4:E(ThumbUp)

// Function declarations
void delay_ms(unsigned int ms);
void I2C_Start(void);
void I2C_Stop(void);
void I2C_SendByte(unsigned char dat);
bit I2C_WaitAck(void);
void PCA9685_WriteReg(unsigned char reg, unsigned char val);
void PCA9685_SetPWM(unsigned char channel, unsigned short on, unsigned short off);
void PCA9685_Init(void);
void UART1_Init(void);
void SendByte(unsigned char dat);
void SendString(unsigned char *str);
void SetAllServos(unsigned short *pwm_values);

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

// Send one byte via UART (disable ES to avoid ISR clearing TI before while loop sees it)
void SendByte(unsigned char dat)
{
    ES = 0;
    SBUF = dat;
    while(!TI);
    TI = 0;
    ES = 1;
}

// Send string via UART
void SendString(unsigned char *str)
{
    while(*str)
    {
        SendByte(*str++);
    }
}

// Initialize UART1 (115200 baud @ 11.0592MHz)
void UART1_Init(void)
{
    SCON = 0x50;       // 8-bit UART, variable baud rate
    AUXR |= 0x40;      // Timer1 in 1T mode (bit6 T1x12 = 1)
    AUXR &= 0xFE;      // UART1 selects Timer1 as baud rate generator
    TMOD &= 0x0F;      // Clear Timer1 mode
    TMOD |= 0x20;      // Timer1 in mode 2 (8-bit auto-reload)
    TH1 = 0xFD;        // 11.0592MHz, 115200 baud
    TL1 = 0xFD;
    TR1 = 1;           // Start Timer1
    ES = 1;            // Enable UART interrupt
    EA = 1;            // Enable global interrupts
}

// UART1 interrupt handler - single character commands: A(Relax) B(Spread) C(Fist) D(Point) E(ThumbUp)
// NOTE: ISR only stores the received char and sets a flag.
// Serial response is handled in main loop to avoid reentrancy issues (WARNING L15).
void UART1_Isr(void) interrupt 4
{
    unsigned char ch;
    
    if(RI)
    {
        RI = 0;
        ch = SBUF;
        
        if(ch >= 'A' && ch <= 'E' && !ready_to_act)
        {
            gesture_id = ch - 'A';
            ready_to_act = 1;
        }
        else
        {
            // Unknown command or already busy - ignore silently in ISR
        }
    }
    
    if(TI)
    {
        TI = 0;  // Clear transmit interrupt flag
    }
}

// Main program
void main(void)
{
    unsigned short relax_pwms[8] = {RELAX_SERVO0, RELAX_SERVO1, RELAX_SERVO2, RELAX_SERVO3,
                                    RELAX_SERVO4, RELAX_SERVO5, RELAX_SERVO6, RELAX_SERVO7};
    unsigned short spread_pwms[8] = {SPREAD_SERVO0, SPREAD_SERVO1, SPREAD_SERVO2, SPREAD_SERVO3,
                                     SPREAD_SERVO4, SPREAD_SERVO5, SPREAD_SERVO6, SPREAD_SERVO7};
    unsigned short fist_pwms[8] = {FIST_SERVO0, FIST_SERVO1, FIST_SERVO2, FIST_SERVO3,
                                   FIST_SERVO4, FIST_SERVO5, FIST_SERVO6, FIST_SERVO7};
    unsigned short point_pwms[8] = {POINT_SERVO0, POINT_SERVO1, POINT_SERVO2, POINT_SERVO3,
                                    POINT_SERVO4, POINT_SERVO5, POINT_SERVO6, POINT_SERVO7};
    unsigned short thumbup_pwms[8] = {THUMBUP_SERVO0, THUMBUP_SERVO1, THUMBUP_SERVO2, THUMBUP_SERVO3,
                                      THUMBUP_SERVO4, THUMBUP_SERVO5, THUMBUP_SERVO6, THUMBUP_SERVO7};
    
    P3M0 = 0x00;
    P3M1 = 0x00;   // Configure P3 as quasi-bidirectional (standard 8051 mode)
    P5M0 = 0x00;
    P5M1 = 0x00;
    
    I2C_Stop();    // Initialize I2C bus
    delay_ms(10);
    PCA9685_Init();
    delay_ms(10);
    
    UART1_Init();
    
    // Start in relax position
    SetAllServos(relax_pwms);
    
    SendString("\r\n=== Hand Gesture Controller Ready ===\r\n");
    SendString("Commands: A=Relax B=Spread C=Fist D=Point E=ThumbUp\r\n");
    SendString("Current: A > Relax (放松)\r\n");

    // Command mode: wait for serial commands (A/B/C/D/E)
    // NOTE: All SendString calls are in main loop (not ISR) to avoid WARNING L15.
    while(1)
    {
        if(ready_to_act)
        {
            ready_to_act = 0;
            SendString("Executing... ");
            switch(gesture_id)
            {
                case 0: SetAllServos(relax_pwms); SendString("A > Relax (放松)\r\n"); break;
                case 1: SetAllServos(spread_pwms); SendString("B > Spread (摊开)\r\n"); break;
                case 2: SetAllServos(fist_pwms); SendString("C > Fist (握拳)\r\n"); break;
                case 3: SetAllServos(point_pwms); SendString("D > Point (伸食指)\r\n"); break;
                case 4: SetAllServos(thumbup_pwms); SendString("E > ThumbUp (伸拇指)\r\n"); break;
                default: break;
            }
        }
    }
}