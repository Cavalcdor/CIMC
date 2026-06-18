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
// ===== Interactive Serial Tuning Tool =====
// Commands over UART (115200 baud):
//   0-7     Select single servo
//   f0-f3   Select finger pair (f0=thumb, f1=index, f2=middle, f3=pinky)
//   +       Increment PWM (single mode) / bend (pair mode: first↑, second↓)
//   -       Decrement PWM (single mode) / straighten (pair mode: first↓, second↑)
//   s       Print all 8 servo PWMs
//   c       Center all servos to 300
//   h       Print this help
//   n       Cycle step: 1 -> 5 -> 10 -> 20 -> 50 -> 1 ...
//   a       Auto sweep current finger pair (200-400, 10-step)
//   p       Print current selection info

// ===== State =====
unsigned short servo_pwms[8] = {300, 300, 300, 300, 300, 300, 300, 300};
unsigned char selected_servo = 0;
unsigned char selected_finger = 0;
bit single_mode = 1;            // 1=single servo, 0=finger pair
unsigned char step_size = 10;

const unsigned char step_table[5] = {1, 5, 10, 20, 50};
unsigned char step_index = 2;    // start at 10

const unsigned char finger_names[4] = {'0', '1', '2', '3'}; // Thumb, Index, Middle, Pinky
const unsigned char *finger_labels[4] = {"Thumb", "Index", "Middle", "Pinky"};

// ===== Function Declarations =====
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
void UART1_Init(void);
void SendByte(unsigned char dat);
void SendString(unsigned char *str);
void SendNum(unsigned short val);
void PrintStatus(void);
void PrintHelp(void);
void PrintSelection(void);
void CenterAll(void);
void ApplyServo(unsigned char ch);
void AutoSweepFinger(unsigned char f);

// I2C interface pins
sbit SCL = P3^2;
sbit SDA = P3^3;

// ===== Delay (11.0592MHz, ~1ms) =====
void delay_ms(unsigned int ms)
{
    unsigned int i, j;
    for(i = 0; i < ms; i++)
        for(j = 0; j < 1100; j++);
}

// ===== I2C Primitives =====
void I2C_Start(void)
{
    SDA = 1;
    SCL = 1;
    _nop_(); _nop_();
    SDA = 0;
    _nop_(); _nop_();
    SCL = 0;
}

void I2C_Stop(void)
{
    SDA = 0;
    SCL = 1;
    _nop_(); _nop_();
    SDA = 1;
    _nop_(); _nop_();
}

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

// ===== PCA9685 =====
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

void PCA9685_SetPWM(unsigned char channel, unsigned short on, unsigned short off)
{
    unsigned char reg = LED0_ON_L + 4 * channel;
    PCA9685_WriteReg(reg, on & 0xFF);
    PCA9685_WriteReg(reg + 1, on >> 8);
    PCA9685_WriteReg(reg + 2, off & 0xFF);
    PCA9685_WriteReg(reg + 3, off >> 8);
}

void PCA9685_Init(void)
{
    PCA9685_WriteReg(MODE1, 0x10);
    PCA9685_WriteReg(PRESCALE, 121);
    PCA9685_WriteReg(MODE2, 0x04);
    PCA9685_WriteReg(MODE1, 0x00);
    delay_ms(5);
}

// ===== Servo Control =====
void SetAllServos(unsigned short *pwm_values)
{
    unsigned char i;
    for(i = 0; i < 8; i++)
    {
        PCA9685_SetPWM(i, 0, pwm_values[i]);
    }
}

void SetSingleServo(unsigned char channel, unsigned short off)
{
    PCA9685_SetPWM(channel, 0, off);
}

// Apply current PWM value of servo ch to hardware
void ApplyServo(unsigned char ch)
{
    SetSingleServo(ch, servo_pwms[ch]);
}

// ===== UART Transmit =====
void SendByte(unsigned char dat)
{
    SBUF = dat;
    while(!TI);
    TI = 0;
}

void SendString(unsigned char *str)
{
    while(*str)
        SendByte(*str++);
}

void SendNum(unsigned short val)
{
    unsigned char buf[4];
    unsigned char i = 0;
    if(val >= 100) { buf[i++] = '0' + (val / 100); val %= 100; }
    else           { buf[i++] = ' '; }
    if(val >= 10)  { buf[i++] = '0' + (val / 10); val %= 10; }
    else           { buf[i++] = ' '; }
    buf[i++] = '0' + val;
    buf[i] = '\0';
    SendString(buf);
}

// ===== Print Functions =====
void PrintHelp(void)
{
    SendString("\r\n--- Servo Tuning Commands ---\r\n");
    SendString(" 0-7     Select servo N (single mode)\r\n");
    SendString(" f0-f3   Select finger pair (0=T 1=I 2=M 3=P)\r\n");
    SendString(" +       +step (single) / bend (pair: first+ second-)\r\n");
    SendString(" -       -step (single) / straighten (pair: first- second+)\r\n");
    SendString(" s       Status: print all 8 PWMs\r\n");
    SendString(" c       Center all servos (300)\r\n");
    SendString(" n       Cycle step size\r\n");
    SendString(" a       Auto sweep selected finger pair\r\n");
    SendString(" p       Print current selection\r\n");
    SendString(" h       This help\r\n> ");
}

void PrintSelection(void)
{
    SendString("\r\n[");
    if(single_mode)
    {
        SendString("S");
        SendByte('0' + selected_servo);
        SendString("] PWM=");
        SendNum(servo_pwms[selected_servo]);
    }
    else
    {
        unsigned char base = selected_finger * 2;
        SendString("F");
        SendByte('0' + selected_finger);
        SendString(" ");
        SendString((unsigned char *)finger_labels[selected_finger]);
        SendString(": S");
        SendByte('0' + base);
        SendString("=");
        SendNum(servo_pwms[base]);
        SendString(" S");
        SendByte('0' + base + 1);
        SendString("=");
        SendNum(servo_pwms[base + 1]);
        SendString("  (+bend/-straighten)");
    }
    SendString("\r\n> ");
}

void PrintStatus(void)
{
    unsigned char i;
    SendString("\r\n");
    for(i = 0; i < 8; i++)
    {
        SendString("S");
        SendByte('0' + i);
        SendString(":");
        SendNum(servo_pwms[i]);
        SendString("  ");
        if(i == 1 || i == 3 || i == 5) SendString("\r\n");  // group by finger
    }
    SendString("\r\n> ");
}

// ===== Actions =====
void CenterAll(void)
{
    unsigned char i;
    for(i = 0; i < 8; i++)
    {
        servo_pwms[i] = 300;
        ApplyServo(i);
    }
    SendString("\r\nAll centered (300)\r\n> ");
}

// Auto sweep: sweep current finger pair from edge to edge
void AutoSweepFinger(unsigned char f)
{
    unsigned char base = f * 2;
    signed short val;
    SendString("\r\n--- Auto Sweep F");
    SendByte('0' + f);
    SendString(" ---\r\n");

    // Phase 1: both to center first
    servo_pwms[base] = 300;
    servo_pwms[base + 1] = 300;
    ApplyServo(base);
    ApplyServo(base + 1);
    delay_ms(1000);

    // Phase 2: sweep one direction (first↑, second↓)
    for(val = 0; val <= 100; val += 10)
    {
        servo_pwms[base] = 300 + val;
        servo_pwms[base + 1] = 300 - val;
        ApplyServo(base);
        ApplyServo(base + 1);
        delay_ms(400);
    }
    delay_ms(800);

    // Phase 3: sweep back to center then other direction (first↓, second↑)
    for(val = 100; val >= -100; val -= 10)
    {
        servo_pwms[base] = 300 + val;
        servo_pwms[base + 1] = 300 - val;
        ApplyServo(base);
        ApplyServo(base + 1);
        delay_ms(400);
    }
    delay_ms(800);

    // Phase 4: back to center
    for(val = -100; val <= 0; val += 10)
    {
        servo_pwms[base] = 300 + val;
        servo_pwms[base + 1] = 300 - val;
        ApplyServo(base);
        ApplyServo(base + 1);
        delay_ms(400);
    }

    SendString("F");
    SendByte('0' + f);
    SendString(" sweep done\r\n> ");
}

// ===== UART1 Init =====
void UART1_Init(void)
{
    SCON = 0x50;
    AUXR |= 0x40;
    AUXR &= 0xFE;
    TMOD &= 0x0F;
    TMOD |= 0x20;
    TH1 = 0xFA;
    TL1 = 0xFA;
    TR1 = 1;
    ES = 1;
    EA = 1;
}

// ===== UART1 Interrupt - Command Parser =====
void UART1_Isr(void) interrupt 4
{
    unsigned char ch;
    static bit await_f = 0;

    if(RI)
    {
        RI = 0;
        ch = SBUF;

        // Handle two-char prefix: 'f' + '0'..'3'
        if(await_f)
        {
            await_f = 0;
            if(ch >= '0' && ch <= '3')
            {
                selected_finger = ch - '0';
                single_mode = 0;
                PrintSelection();
            }
            else
            {
                SendString("\r\n? ");  // invalid after 'f'
            }
            return;
        }

        // Single-character commands
        switch(ch)
        {
            case '0': case '1': case '2': case '3':
            case '4': case '5': case '6': case '7':
                selected_servo = ch - '0';
                single_mode = 1;
                PrintSelection();
                break;

            case 'f':
            case 'F':
                await_f = 1;
                break;

            case '+':
                if(single_mode)
                {
                    if(servo_pwms[selected_servo] + step_size <= 500)
                    {
                        servo_pwms[selected_servo] += step_size;
                        ApplyServo(selected_servo);
                    }
                    SendString("\r\nS");
                    SendByte('0' + selected_servo);
                    SendString("=");
                    SendNum(servo_pwms[selected_servo]);
                    SendString("\r\n> ");
                }
                else
                {
                    unsigned char base = selected_finger * 2;
                    bit changed = 0;
                    if(servo_pwms[base] + step_size <= 500)
                    {
                        servo_pwms[base] += step_size;
                        ApplyServo(base);
                        changed = 1;
                    }
                    if(servo_pwms[base + 1] >= step_size)
                    {
                        servo_pwms[base + 1] -= step_size;
                        ApplyServo(base + 1);
                        changed = 1;
                    }
                    if(!changed) { SendString("\r\nlimit\r\n> "); break; }
                    SendString("\r\nF");
                    SendByte('0' + selected_finger);
                    SendString(": S");
                    SendByte('0' + base);
                    SendString("=");
                    SendNum(servo_pwms[base]);
                    SendString(" S");
                    SendByte('0' + base + 1);
                    SendString("=");
                    SendNum(servo_pwms[base + 1]);
                    SendString("\r\n> ");
                }
                break;

            case '-':
                if(single_mode)
                {
                    if(servo_pwms[selected_servo] >= step_size)
                    {
                        servo_pwms[selected_servo] -= step_size;
                        ApplyServo(selected_servo);
                    }
                    SendString("\r\nS");
                    SendByte('0' + selected_servo);
                    SendString("=");
                    SendNum(servo_pwms[selected_servo]);
                    SendString("\r\n> ");
                }
                else
                {
                    unsigned char base = selected_finger * 2;
                    bit changed = 0;
                    if(servo_pwms[base] >= step_size)
                    {
                        servo_pwms[base] -= step_size;
                        ApplyServo(base);
                        changed = 1;
                    }
                    if(servo_pwms[base + 1] + step_size <= 500)
                    {
                        servo_pwms[base + 1] += step_size;
                        ApplyServo(base + 1);
                        changed = 1;
                    }
                    if(!changed) { SendString("\r\nlimit\r\n> "); break; }
                    SendString("\r\nF");
                    SendByte('0' + selected_finger);
                    SendString(": S");
                    SendByte('0' + base);
                    SendString("=");
                    SendNum(servo_pwms[base]);
                    SendString(" S");
                    SendByte('0' + base + 1);
                    SendString("=");
                    SendNum(servo_pwms[base + 1]);
                    SendString("\r\n> ");
                }
                break;

            case 's':
            case 'S':
                PrintStatus();
                break;

            case 'c':
            case 'C':
                CenterAll();
                break;

            case 'n':
            case 'N':
                step_index++;
                if(step_index >= 5) step_index = 0;
                step_size = step_table[step_index];
                SendString("\r\nStep=");
                if(step_size < 10) SendByte(' ');
                SendNum(step_size);
                SendString("\r\n> ");
                break;

            case 'a':
            case 'A':
                if(!single_mode)
                    AutoSweepFinger(selected_finger);
                else
                    SendString("\r\nSelect finger pair first (f0-f3)\r\n> ");
                break;

            case 'p':
            case 'P':
                PrintSelection();
                break;

            case 'h':
            case 'H':
                PrintHelp();
                break;

            default:
                SendString("\r\n?\r\n> ");
                break;
        }
    }

    if(TI)
    {
        TI = 0;
    }
}

// ===== Main =====
void main(void)
{
    unsigned char i;
    unsigned short neutral[8];

    for(i = 0; i < 8; i++)
    {
        servo_pwms[i] = 300;
        neutral[i] = 300;
    }

    P3M0 = 0x00;
    P3M1 = 0x00;
    P5M0 = 0x00;
    P5M1 = 0x00;

    I2C_Stop();
    delay_ms(10);
    PCA9685_Init();
    delay_ms(10);

    // All servos to center
    SetAllServos(neutral);
    delay_ms(500);

    // Init UART
    UART1_Init();

    // Welcome: send 3 times to ensure serial is ready
    for(i = 0; i < 3; i++)
    {
        delay_ms(100);
        SendString("\r\n=== Servo Tuning Tool ===\r\n");
        SendString("115200 baud | Send 'h' for help\r\n> ");
    }

    // Idle loop - everything happens in interrupt
    while(1)
    {
        // Nothing to do here; commands handled by UART ISR
    }
}