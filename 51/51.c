#include <STC8G.h>
#include <intrins.h>

// PWM value definitions for servo control (Unit: ticks)
// PCA9685 resolution: 4096, 50Hz (20ms period)
// PWM value = pulse width (us) / (20ms/4096) = pulse width (us) / 4.8828125
// Adjusted to a more moderate 230-370 range for safer motion.

#define FIST_SERVO0   360
#define FIST_SERVO1   230   
#define FIST_SERVO2   280   
#define FIST_SERVO3   360   
#define FIST_SERVO4   240   
#define FIST_SERVO5   360   
#define FIST_SERVO6   240   
#define FIST_SERVO7   360   

#define RELAX_SERVO0  240
#define RELAX_SERVO1  370
#define RELAX_SERVO2  370
#define RELAX_SERVO3  230
#define RELAX_SERVO4  370
#define RELAX_SERVO5  240
#define RELAX_SERVO6  370
#define RELAX_SERVO7  240


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
bit fist_cmd = 0;           // 0:Relax, 1:Fist

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

// Send one byte via UART
void SendByte(unsigned char dat)
{
    SBUF = dat;
    while(!TI);
    TI = 0;
}

// Send string via UART
void SendString(unsigned char *str)
{
    while(*str)
    {
        SendByte(*str++);
    }
}

// Initialize UART1
void UART1_Init(void)
{
    SCON = 0x50;       // 8-bit UART, variable baud rate
    AUXR |= 0x40;      // Timer1 in 1T mode
    AUXR &= 0xFE;      // Timer1 clock not divided
    TMOD &= 0x0F;      // Clear Timer1 mode
    TMOD |= 0x20;      // Timer1 in mode 2 (8-bit auto-reload)
    TH1 = 0xFA;        // 11.0592MHz, 115200 baud (FA)
    TL1 = 0xFA;
    TR1 = 1;           // Start Timer1
    ES = 1;            // Enable UART interrupt
    EA = 1;            // Enable global interrupts
}

// UART1 interrupt handler
void UART1_Isr(void) interrupt 4
{
    static unsigned char rx_buffer[10];
    static unsigned char index = 0;
    unsigned char ch;
    
    if(RI)
    {
        RI = 0;
        ch = SBUF;
        
        // Parse command: "Fist"
        if(index == 0 && ch == 'F')
        {
            index = 1;
            rx_buffer[0] = 'F';
        }
        else if(index == 1 && ch == 'i')
        {
            index = 2;
            rx_buffer[1] = 'i';
        }
        else if(index == 2 && ch == 's')
        {
            index = 3;
            rx_buffer[2] = 's';
        }
        else if(index == 3 && ch == 't')
        {
            index = 4;
            rx_buffer[3] = 't';
            rx_buffer[4] = '\0';
            // Activate Fist command
            fist_cmd = 1;
            ready_to_act = 1;
            index = 0;
            // Send confirmation
            SendByte('F');
            SendByte('i');
            SendByte('s');
            SendByte('t');
            SendByte(' ');
            SendByte('O');
            SendByte('K');
            SendByte('\r');
            SendByte('\n');
        }
        else if(index == 0 && ch == 'R')
        {
            index = 5;
            rx_buffer[0] = 'R';
        }
        else if(index == 5 && ch == 'e')
        {
            index = 6;
            rx_buffer[1] = 'e';
        }
        else if(index == 6 && ch == 'l')
        {
            index = 7;
            rx_buffer[2] = 'l';
        }
        else if(index == 7 && ch == 'a')
        {
            index = 8;
            rx_buffer[3] = 'a';
        }
        else if(index == 8 && ch == 'x')
        {
            index = 9;
            rx_buffer[4] = 'x';
            rx_buffer[5] = '\0';
            // Activate Relax command
            fist_cmd = 0;
            ready_to_act = 1;
            index = 0;
            // Send confirmation
            SendByte('R');
            SendByte('e');
            SendByte('l');
            SendByte('a');
            SendByte('x');
            SendByte(' ');
            SendByte('O');
            SendByte('K');
            SendByte('\r');
            SendByte('\n');
        }
        else
        {
            // Invalid command, reset
            index = 0;
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
    unsigned short fist_pwms[8] = {FIST_SERVO0, FIST_SERVO1, FIST_SERVO2, FIST_SERVO3,
                                   FIST_SERVO4, FIST_SERVO5, FIST_SERVO6, FIST_SERVO7};
    unsigned short relax_pwms[8] = {RELAX_SERVO0, RELAX_SERVO1, RELAX_SERVO2, RELAX_SERVO3,
                                    RELAX_SERVO4, RELAX_SERVO5, RELAX_SERVO6, RELAX_SERVO7};
    
    P3M0 = 0x00;
    P3M1 = 0x00;   // Configure P3 as push-pull
    P5M0 = 0x00;
    P5M1 = 0x00;
    
    I2C_Stop();    // Initialize I2C bus
    delay_ms(10);
    PCA9685_Init();
    delay_ms(10);
    
    // Start in relax position
    SetAllServos(relax_pwms);
    
    UART1_Init();

    // Auto mode: switch between relax and fist every 4 seconds.
    fist_cmd = 0;
    SetAllServos(relax_pwms);

    while(1)
    {
        delay_ms(2000);
        fist_cmd ^= 1;
        if(fist_cmd)
            SetAllServos(fist_pwms);
        else
            SetAllServos(relax_pwms);
    }
}