#include <stdio.h>
#include <p18f4520.h>
#include <delays.h>
#include <i2c.h>
#pragma config OSC = HS
#pragma config MCLRE = ON
#pragma config WDT = OFF
#pragma config LVP = OFF
int giay, s, m, h, s0;
unsigned char seg[10] = {0xC0, 0xF9, 0xA4, 0xB0, 0x99, 0x92, 0x82, 0xF8, 0x80, 0x90};
unsigned char chedo_cd, chedo_td, chedo_fre;
char LCD[32];
#define clk PORTCbits.RC0
#define lat PORTCbits.RC1
#define data PORTCbits.RC2
#define RS PORTEbits.RE2
#define RW PORTEbits.RE1
#define EN PORTEbits.RE0
#define DATA PORTD
#define CD PORTBbits.RB1
#define TD PORTBbits.RB0
#define FRE PORTBbits.RB2
#define WR1 PORTCbits.RC5
#define WB1 PORTCbits.RC6
#define B1 PORTAbits.RA0
#define Y1 PORTAbits.RA1
#define R1 PORTAbits.RA2
#define B2 PORTAbits.RA3
#define Y2 PORTAbits.RA4
#define R2 PORTAbits.RA5
#define WR2 PORTBbits.RB3
#define WB2 PORTBbits.RB4
unsigned int lan1, lan2, lan3, lan4;
unsigned int x, chedo;
unsigned int time, time1;
void ngat_timer0(void);
#pragma code ngat_cao = 0x08
void ngat_cao(void)
{
    _asm goto ngat_timer0 // jump to interrupt routine
        _endasm
}
//----------------------------------------------------------------------------
// High priority interrupt routine

#pragma code
#pragma interrupt ngat_timer0

void ngat_timer0()
{
    if (INTCONbits.TMR0IF == 1)
    {
        T0CONbits.TMR0ON = 0; // tat bo timer
        giay--;
        INTCONbits.TMR0IF = 0;         // xoa co tran
        TMR0H = (65535 - 32500) / 256; // nap gia tri cho TMR0L,TMR0H de duoc thoi gian 1s
        TMR0L = (65535 - 62500) % 256;
        T0CONbits.TMR0ON = 1; // bat bo timer
    }
}
//=======================================giao tiep voi I2C===========================================
void get_time()
{
    StartI2C();
    IdleI2C();
    WriteI2C(0xd0);
    IdleI2C();
    WriteI2C(0x00);
    IdleI2C();
    RestartI2C();
    IdleI2C();
    WriteI2C(0xd1);
    IdleI2C();
    s = ReadI2C();
    IdleI2C();
    AckI2C();
    IdleI2C();
    m = ReadI2C();
    IdleI2C();
    AckI2C();
    IdleI2C();
    h = ReadI2C();
    IdleI2C();
    NotAckI2C();
    IdleI2C();
    StopI2C();
}
void set_time(unsigned char s, unsigned char m, unsigned char h)
{
    StartI2C();
    IdleI2C();
    WriteI2C(0xd0);
    IdleI2C();
    WriteI2C(0x00);
    IdleI2C();
    WriteI2C(s);
    IdleI2C();
    WriteI2C(m);
    IdleI2C();
    WriteI2C(h);
    IdleI2C();
    StopI2C();
}
char bcd_int(int x)
{
    return (((x >> 4) & 0x0f) * 10) + (x & 0x0f);
}
int int_bcd(int x)
{
    char N[10] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09};
    int a, b;
    a = x / 10;
    b = x % 10;
    return ((N[a] << 4) & 0xf0) + N[b];
}
//========================================================dieu chinh thanh ghi dich==========================================================================
void ICGHIDICH(unsigned char ch4, unsigned char dv4, unsigned char ch3, unsigned char dv3, unsigned char ch2, unsigned char dv2, unsigned char ch1, unsigned char dv1)
{
    char i;
    for (i = 0; i < 8; i++)
    { // lay tung byte day ra (8 byte max truoc)
        if ((dv1 & 0x80) == 0)
            data = 0;
        else
            data = 1;
        clk = 0;
        Delay10TCYx(5);
        clk = 1;
        dv1 = dv1 << 1; // lui 1 byte ve phia trai//
    }
    for (i = 0; i < 8; i++)
    {
        if ((ch1 & 0x80) == 0)
            data = 0;
        else
            data = 1;
        clk = 0;
        Delay10TCYx(5);
        clk = 1;
        ch1 = ch1 << 1;
    }
    for (i = 0; i < 8; i++)
    {
        if ((dv2 & 0x80) == 0)
            data = 0;
        else
            data = 1;
        clk = 0;
        Delay10TCYx(5);
        clk = 1;
        dv2 = dv2 << 1;
    }
    for (i = 0; i < 8; i++)
    {
        if ((ch2 & 0x80) == 0)
            data = 0;
        else
            data = 1;
        clk = 0;
        Delay10TCYx(5);
        clk = 1;
        ch2 = ch2 << 1;
    }
    for (i = 0; i < 8; i++)
    {
        if ((dv3 & 0x80) == 0)
            data = 0;
        else
            data = 1;
        clk = 0;
        Delay10TCYx(5);
        clk = 1;
        dv3 = dv3 << 1;
    }
    for (i = 0; i < 8; i++)
    {
        if ((ch3 & 0x80) == 0)
            data = 0;
        else
            data = 1;
        clk = 0;
        Delay10TCYx(5);
        clk = 1;
        ch3 = ch3 << 1;
    }
    for (i = 0; i < 8; i++)
    {
        if ((dv4 & 0x80) == 0)
            data = 0;
        else
            data = 1;
        clk = 0;
        Delay10TCYx(5);
        clk = 1;
        dv4 = dv4 << 1;
    }
    for (i = 0; i < 8; i++)
    {
        if ((ch4 & 0x80) == 0)
            data = 0;
        else
            data = 1;
        clk = 0;
        Delay10TCYx(5);
        clk = 1;
        ch4 = ch4 << 1;
    }
    lat = 0;
    Delay10TCYx(5);
    lat = 1;
}
//=======================================thiet lap thoi gian==========================================
void thapdiem(void)
{
    if (giay == 0)
        giay = 59;
    {
        lan1 = lan3 = giay;
        lan2 = lan4 = giay - 5;
        PORTA = 0B00100001; // L24 do, L13 Xanh
        WB2 = WR1 = 1;
        PORTD = 11111111;
        WB1 = WR2 = 0;
    }
    if (giay < 35 && giay >= 30)
    {
        lan1 = lan3 = giay - 30;
        lan2 = lan4 = giay - 30;
        PORTA = 0B00100010; // L24 do, L13 Vang
        WB2 = WR1 = 1;
        WB1 = WR2 = 0;
    }
    if (giay < 30 && giay >= 5)
    {
        lan1 = lan3 = giay;
        lan2 = lan4 = giay - 5;
        PORTA = 0B00001100; // L24 xanh,L13 do
        WB2 = WR1 = 0;
        WB1 = WR2 = 1;
    }
    if (giay < 5)
    {
        lan1 = lan3 = giay;
        lan2 = lan4 = giay;
        PORTA = 0B00010100; // L24 Vang, L13 do
        WB2 = WR1 = 0;
        WB1 = WR2 = 1;
    }
    ICGHIDICH(seg[lan4 / 10], seg[lan4 % 10], seg[lan3 / 10], seg[lan3 % 10], seg[lan2 / 10], seg[lan2 % 10], seg[lan1 / 10], seg[lan1 % 10]); // xuat ra leb 7 doan(ser de dich du lieu)
}
void caodiem(void)
{
    if (giay == 0)
        giay = 89;
    {
        lan1 = lan3 = giay;
        lan2 = lan4 = giay - 5;
        PORTA = 0B00100001; // L24 do, L13 Xanh
        WB2 = WR1 = 1;
        WB1 = WR2 = 0;
    }
    if (giay < 50 && giay >= 45)
    {
        lan1 = lan3 = giay - 45;
        lan2 = lan4 = giay - 45;
        PORTA = 0B00100010; // L24 do, L13 Vang
        WB2 = WR1 = 1;
        WB1 = WR2 = 0;
    }
    if (giay < 45 && giay >= 5)
    {
        lan1 = lan3 = giay;
        lan2 = lan4 = giay - 5;
        PORTA = 0B00001100; // L24 xanh,L13 do
        WB2 = WR1 = 0;
        WB1 = WR2 = 1;
    }
    if (giay < 5)
    {
        lan1 = lan3 = giay;
        lan2 = lan4 = giay;
        PORTA = 0B00010100; // L24 Vang, L13 do
        WB2 = WR1 = 0;
        WB1 = WR2 = 1;
    }
    ICGHIDICH(seg[lan4 / 10], seg[lan4 % 10], seg[lan3 / 10], seg[lan3 % 10], seg[lan2 / 10], seg[lan2 % 10], seg[lan1 / 10], seg[lan1 % 10]);
}
void lcd_cmd(unsigned char x)
{
    RW = 0;
    RS = 0;
    DATA &= 0X0f;
    EN = 1;
    DATA |= x & 0Xf0;
    EN = 0;
    Delay1KTCYx(10);
    DATA &= 0X0f;
    EN = 1;
    DATA |= ((x << 4) & 0xf0);
    EN = 0;
    Delay1KTCYx(10);
}
void lcd_data(unsigned char x)
{
    RW = 0;
    RS = 1;
    DATA &= 0X0f;
    EN = 1;
    DATA |= x & 0Xf0;
    EN = 0;
    Delay1KTCYx(10);
    DATA &= 0X0f;
    EN = 1;
    DATA |= ((x << 4) & 0xf0);
    EN = 0;
    Delay1KTCYx(10);
}
void lcd_init(void)
{
    lcd_cmd(0x03);
    lcd_cmd(0x02);
    lcd_cmd(0x28);
    lcd_cmd(0x0c);
    lcd_cmd(0x06);
    lcd_cmd(0x01);
}
void lcd_str(unsigned char *str)
{
    while (*str)
    {
        lcd_data(*str);
        str++;
    }
}

void main()
{
    TRISA = 0X00;
    TRISD = 0X00;
    TRISE = 0X00;
    TRISC = 0X00;
    TRISB = 0x07;
    ADCON1 = 0X0F;
    RCONbits.IPEN = 1;      // su dung uu tien ngat
    INTCONbits.GIEH = 1;    // cho phep tat ca cac ngat uu tien cao hoat dong
    INTCONbits.TMR0IE = 1;  // cho phep ngat timer 0 hoat dong
    INTCON2bits.TMR0IP = 1; // chon muc ngat cao
    // INTCONbits.GIEL=1; // cho phep tat ca cac ngat uu tien cao hoat dong
    /******* khoi tao timer0 tao tre 1s****/
    T0CONbits.T08BIT = 0; // chon che do 16bits
    T0CONbits.T0CS = 0;   // chon xung he thong
    T0CONbits.PSA = 0;    // su dung bo chia tan
    T0CONbits.T0PS2 = 1;  // he so chia tan =16
    T0CONbits.T0PS1 = 0;
    T0CONbits.T0PS0 = 0;
    TMR0H = (65535 - 62500) / 256; // nap gia tri cho TMR0L,TMR0H de duoc thoi gian 1s
    TMR0L = (65535 - 62500) % 256;
    T0CONbits.TMR0ON = 1; // bat bo timer
    /*************GTTG*********/
    OpenI2C(MASTER, SLEW_OFF);
    SSPADD = 0X63; // CHU Y
    lcd_init();
    // set_time(0x00,0x25,0x13);
    lan1 = 0;
    lan2 = 0;
    lan3 = 0;
    lan4 = 0;
    giay = 0;
    chedo_cd = chedo_td = chedo_fre = 0;
    while (1)
    {
        get_time();
        s = bcd_int(s);
        m = bcd_int(m);
        h = bcd_int(h);
        lcd_cmd(0x80);
        sprintf(&LCD[0], "    %d%d:%d%d:%d%d     ", h / 10, h % 10, m / 10, m % 10, s / 10, s % 10);
        lcd_str(&LCD[0]);
        time = h + 1;
        lcd_cmd(0xC0);
        sprintf(&LCD[0], "    %d     ", time);
        lcd_str(&LCD[0]);
        if ((6 < time && time <= 18) || (20 < time && time <= 22))
        {
            goto chedo1;
        }
        if (time > 18 && time <= 20)
        {
            goto chedo2;
        }
        if ((time > 22 && time <= 24) || (0 < time && time <= 6))
        {
            goto chedo3;
        }
    chedo1:
        if ((6 < time && time <= 18) || (20 < time && time <= 22) && (chedo_td == 0))
        {
            giay = 59;
            while (1)
            {
                thapdiem();
                get_time();
                s = bcd_int(s);
                m = bcd_int(m);
                h = bcd_int(h);
                lcd_cmd(0x80);
                sprintf(&LCD[0], "    %d%d:%d%d:%d%d     ", h / 10, h % 10, m / 10, m % 10, s / 10, s % 10);
                lcd_str(&LCD[0]);
                lcd_cmd(0xC0);
                sprintf(&LCD[0], " Gio Thap Diem  ");
                lcd_str(&LCD[0]);
                time = h + 1;
                if (TD == 0)
                {
                    while (TD == 0)
                        ;
                    chedo_cd = chedo_td = chedo_fre = 0;
                }
                if (CD == 0)
                {
                    while (CD == 0)
                        ;
                    chedo_td = chedo_fre = 1;
                    chedo_cd = 0;
                    time = 19;
                    goto chedo2;
                }
                if (FRE == 0)
                {
                    while (FRE == 0)
                        ;
                    chedo_td = chedo_cd = 1;
                    chedo_fre = 0;
                    time = 23;
                    goto chedo3;
                }
                if (time > 18 && time <= 20 && chedo_cd == 0)
                {
                    if (chedo_cd == 0)
                        goto chedo2;
                }
                if ((time > 22 && time <= 24) || (0 < time && time <= 6) && (chedo_fre == 0))
                {
                    if (chedo_fre == 0)
                        goto chedo3;
                }
            }
        }
    chedo2:
        if (time > 18 && time <= 20 && chedo_cd == 0)
        {
            giay = 89;
            while (1)
            {
                caodiem();
                get_time();
                s = bcd_int(s);
                m = bcd_int(m);
                h = bcd_int(h);
                time = h + 1;
                lcd_cmd(0x80);
                sprintf(&LCD[0], "    %d%d:%d%d:%d%d     ", h / 10, h % 10, m / 10, m % 10, s / 10, s % 10);
                lcd_str(&LCD[0]);
                lcd_cmd(0xC0);
                sprintf(&LCD[0], " Gio Cao Diem  ");
                lcd_str(&LCD[0]);
                if (TD == 0)
                {
                    while (TD == 0)
                        ;
                    chedo_cd = chedo_fre = 1;
                    chedo_td = 0;
                    time = 7;
                    goto chedo1;
                }
                if (CD == 0)
                {
                    while (CD == 0)
                        ;
                    chedo_cd = chedo_td = chedo_fre = 0;
                }
                if (FRE == 0)
                {
                    while (FRE == 0)
                        ;
                    chedo_td = chedo_cd = 1;
                    chedo_fre = 0;
                    time = 23;
                    goto chedo3;
                }
                if ((6 < time && time <= 18) || (20 < time && time <= 22) && (chedo_td == 0))
                {
                    if (chedo_td == 0)
                        goto chedo1;
                }
            }
        }
    chedo3:
        if ((time > 10 && time <= 24) || (0 < time && time <= 6) && (chedo_fre == 0))
        {
            while (1)
            {
                Y2 = 1;
                PORTA = 0B00010010;
                get_time();
                s = bcd_int(s);
                m = bcd_int(m);
                h = bcd_int(h);
                time = h + 1;
                lcd_cmd(0x80);
                sprintf(&LCD[0], "    %d%d:%d%d:%d%d     ", h / 10, h % 10, m / 10, m % 10, s / 10, s % 10);
                lcd_str(&LCD[0]);
                lcd_cmd(0xC0);
                sprintf(&LCD[0], "    FREE Time ");
                lcd_str(&LCD[0]);
                if (TD == 0)
                {
                    while (TD == 0)
                        ;
                    chedo_cd = chedo_fre = 1;
                    chedo_td = 0;
                    time = 7;
                    goto chedo1;
                }
                if (CD == 0)
                {
                    while (CD == 0)
                        ;
                    chedo_td = chedo_fre = 0;
                    chedo_cd = 1;
                    time = 19;
                    goto chedo2;
                }
                if (FRE == 0)
                {
                    while (FRE == 0)
                        ;
                    chedo_td = chedo_cd = chedo_fre = 0;
                }
                if ((6 < time && time <= 18) || (20 < time && time <= 22) && (chedo_td == 0))
                {
                    if (chedo_td == 0)
                        goto chedo1;
                }
            }
        }
    }
}
