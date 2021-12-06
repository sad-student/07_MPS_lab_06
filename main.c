#include <msp430.h>
#include "CTS_Layer.h"
#include <math.h>

typedef unsigned char uchar;


#define SCROL_CTL                   0x40  //Scroll image up by SL rows (SL = last 5 bits), range:0-63

#define SET_MIRROR_COL              0xA0  //Normal mirror SEG (column) mapping (set bit0 to mirror columns)

#define SET_MIRROR_ROW              0xC0  //Normal mirror COM (row) mapping (set bit3 to mirror rows)

#define ALL_PIXEL_ON                0xA4  //Disable all pixel on (last bit 1 to turn on all pixels - does not affect memory)

#define LCD_INVERSE                 0xA6  //Inverse display off (last bit 1 to invert display - does not affect memory)

#define BIAS_RATIO_VCC              0xA2  //Set voltage bias ratio (BR = bit0)

#define POW_CTL                     0x2F  //Set Power control - booster, regulator, and follower on

#define SET_CONTRAST_RESISTOR       0x27  //Set internal resistor ratio Rb/Ra to adjust contrast
#define MSB_SELECT_VOLUME            0x81  //Set Electronic Volume "PM" to adjust contrast
#define LSB_SELECT_VOLUME            0x10  //Set Electronic Volume "PM" to adjust contrast (PM = last 5 bits)

#define ADV_CTL_MSB                 0xFA  //Set temp. compensation curve to -0.11%/C
#define ADV_CTL_LSB                 0x90

#define COLUMN_ADR_MSB              0x10  //Set SRAM col. addr. before write, last 4 bits = ca4-ca7
#define COLUMN_ADR_LSB              0x00  //Set SRAM col. addr. before write, last 4 bits = ca0-ca3
#define PAGE_ADR                    0xB0  //Set SRAM page addr (pa = last 4 bits), range:0-8

#define LCD_EN                      0xAF  //Enable display (exit sleep mode & restore power)

#define ROWS 7
#define COLUMNS 5
#define PAGES 1
#define DELAY 500
#define COLUMN_OFFSET_BIG 30
#define COLUMN_OFFSET_NONE 0

#define M_PI 3.1415926535

void SetupButtons();
void SetupLCD();
void Delay(long int value);
void Dogs102x6_writeCommand(uchar *sCmd, uchar i);
void Clear(void);
void __LCD_SetAddress(uchar page, uchar column);
void Dogs102x6_writeData(uchar *sData, uchar i);
void ShowNumber(int number);
int DigitLength(int number);
char CMA3000_RW_SPI(unsigned char mosi_byte1, unsigned char mosi_byte2);

uchar inversions[2][1] = {{0xA6}, {0xA7}};
int bit_level_mg_values[] = {4571, 2286, 1142, 571, 286, 143, 71};
const double rad_degree = 180.0 / M_PI;
unsigned char whatChecking = 0;
struct Element* keypressed;

// 4 columns (+2 offset) and 9 rows. Each byte => 8 rows == 1 page
uchar plus[PAGES][COLUMNS]  = {0x1C, 0x7F, 0x7F, 0x1C, 0x00};
uchar minus[PAGES][COLUMNS] = {0x1C, 0x1C, 0x1C, 0x1C, 0x00};
uchar digits[10][PAGES][COLUMNS] = {
  {0x7F, 0x41, 0x41, 0x7F, 0x00}, // digit 0
  {0x09, 0x11, 0x7F, 0x01, 0x00}, // digit 1
  {0x23, 0x45, 0x49, 0x31, 0x00}, // digit 2
  {0x49, 0x49, 0x49, 0x7F, 0x00}, // digit 3
  {0x78, 0x08, 0x08, 0x7F, 0x00}, // digit 4
  {0x79, 0x49, 0x49, 0x4F, 0x00}, // digit 5
  {0x7F, 0x49, 0x49, 0x4F, 0x00}, // digit 6
  {0x40, 0x40, 0x40, 0x7F, 0x00}, // digit 7
  {0x7F, 0x49, 0x49, 0x7F, 0x00}, // digit 8
  {0x79, 0x49, 0x49, 0x7F, 0x00}  // digit 9
};

int number = 0;
int column_offset = COLUMN_OFFSET_BIG; // 0 - default is COLUMN_OFFSET_BIG, 1 - mirror horizonta is COLUMN_OFFSET_NONE

uchar LCD_INIT_COMMANDS_PART_1[7] = {
    SCROL_CTL,
    SET_MIRROR_COL,
    SET_MIRROR_ROW,
    ALL_PIXEL_ON,
    LCD_INVERSE,
    BIAS_RATIO_VCC,
    POW_CTL
};
uchar LCD_INIT_COMMANDS_PART_2[6] = {
    SET_CONTRAST_RESISTOR,
    MSB_SELECT_VOLUME,
    LSB_SELECT_VOLUME,
    ADV_CTL_MSB,
    ADV_CTL_LSB,
    LCD_EN,
};
int AMOUNT_OF_COMMANDS_1 = 7;
int AMOUNT_OF_COMMANDS_2 = 6;

void setUp(uint16_t level)
{
    PMMCTL0_H = PMMPW_H;

    SVSMHCTL = SVSHE + SVSHRVL0 * level + SVMHE + SVSMHRRL0 * level;
    SVSMLCTL = SVSLE + SVMLE + SVSMLRRL0 * level;

    while ((PMMIFG & SVSMLDLYIFG) == 0);

    PMMIFG &= ~(SVMLVLRIFG + SVMLIFG);
    PMMCTL0_L = PMMCOREV0 * level;

    if ((PMMIFG & SVMLIFG))
        while ((PMMIFG & SVMLVLRIFG) == 0);
    SVSMLCTL = SVSLE + SVSLRVL0 * level + SVMLE + SVSMLRRL0 * level;
    PMMCTL0_H = 0x00;
}


long int myAbs(volatile long int number){
    if(number < 0){
        return -1 * number;
    }
    return number;
}

void SetupTimer()
{
    // setup timer
    TA0CTL = TASSEL__SMCLK | MC__UP | ID__1 | TACLR;     // SMCLK, UP-mode
    long int time = 32768;
    long int period = time / 2;
    TA0CCR0 = time;
    TA0CCR1 = period;
    TA0CCTL1 = OUTMOD_3;
}

void SetupADC()
{

    REFCTL0 &= ~REFMSTR;        // turn of REF block

    ADC12CTL0 =
        ADC12SHT0_8
        + ADC12REFON
        + ADC12ON;

    // Internal ref = 1.5V

    ADC12CTL1 =
        ADC12SHP
        + ADC12SHS_1
        + ADC12SSEL_0
        + ADC12CONSEQ_2;  // enable sample timer

    ADC12MCTL0 =
        ADC12SREF_1
        + ADC12INCH_10;    // ADC i/p ch A10 = temp sense i/pf

    ADC12IE = ADC12IE0;                         // ADC_IFG upon conv result-ADCMEMO

    __delay_cycles(100);                        // delay to allow Ref to settle

    ADC12CTL0 |= ADC12ENC;
}

#define CALADC12_15V_30C *((unsigned int *)0x1A1A) // Temperature Sensor Calibration-30 C
#define CALADC12_15V_85C *((unsigned int *)0x1A1C) // Temperature Sensor Calibration-85 C


#pragma vector = ADC12_VECTOR
__interrupt void ADC12_ISR() {
    TA0CTL &= ~MC__UP;
    TA0CTL |= MC__STOP | TACLR;
    TA0CCTL1 &= ~BIT2;

    // also clears ADC12IFG0 flag
    unsigned short int probe = ADC12MEM0 & 0x0FFF;
    ADC12MEM0 = 0;

    volatile long int temp1 = probe;
    volatile long int temp2 = temp1 - CALADC12_15V_30C;
    volatile float temp3 = temp2 * (85 - 30);
    volatile float temp4 = temp3 / (CALADC12_15V_85C - CALADC12_15V_30C);
    temp4 += 30;
    volatile long int deg_c = (long int)temp4;

    Clear();
    ShowNumber(deg_c);

    ADC12CTL0 &= ~ADC12ENC;
}

int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;

    uint8_t i;

    SetupLCD();

    /* Initialize IO */
    P1DIR = 0xFF;
    P2DIR = 0xFF;
    P8DIR = 0xFF;
    P1OUT = 0;
    P2OUT = 0;
    P8OUT = 0;

    UCSCTL3 = SELREF_2;                       // Set DCO FLL reference = REFO
    UCSCTL4 |= SELA_2;                        // Set ACLK = REFO

    __bis_SR_register(SCG0);                  // Disable the FLL control loop
    UCSCTL0 = 0x0000;                         // Set lowest possible DCOx, MODx
    UCSCTL1 = DCORSEL_7;
    // Select DCO range 50MHz operation
    UCSCTL2 = FLLD_1 + 762;                   // Set DCO Multiplier for 25MHz
                                            // (N + 1) * FLLRef = Fdco
                                            // (762 + 1) * 32768 = 25MHz
                                            // Set FLL Div = fDCOCLK/2
    __bic_SR_register(SCG0);                  // Enable the FLL control loop

    /*  establish baseline */
    TI_CAPT_Init_Baseline(&keypad);
    TI_CAPT_Update_Baseline(&keypad, 5);
    SetupADC();
    SetupTimer();
    ADC12CTL0 |= ADC12ENC;
    setUp(0x01);setUp(0x02);setUp(0x03);
    __bis_SR_register(GIE);

    while (1)
    {
        P1OUT &= ~BIT1;
        P8OUT &= ~BIT1;
        keypressed = (struct Element *) TI_CAPT_Buttons(&keypad);
        Clear();

        if (keypressed && keypressed == &PAD1)
        {
            P1OUT |= BIT1;

            if (!(ADC12CTL1 & ADC12BUSY)) // if there is no active operation
            {
                SetupTimer();
                ADC12CTL0 |= ADC12ENC;
            }
        }
        __delay_cycles(900000);
    }


    return 0;
}

void init_A1(void)
{
    TA1CCR0 = 20000;//5242;//5242 for 0.005 s
    TA1CTL |= TASSEL__SMCLK;
    TA1CTL |= TACLR;

    TA1CTL &= ~MC__UP;
    //TA1CCTL0 |= CCIE;
}


void SetupLCD(void)
{
    // Reset LCD
    P5DIR |= BIT7;  // port init for LCD operations
    P5OUT &= ~BIT7; // set RST (active low)
    P5OUT |= BIT7;  // reset RST (inactive is high)

    // Delay for at least 5ms
    Delay(550);

    // Choosing slave
    P7DIR |= BIT4;  // select LCD for chip
    P7OUT &= ~BIT4; // CS is active low

    // Setting up LCD_D/C
    P5DIR |= BIT6;  // Command/Data for LCD
    P5OUT &= ~BIT6; // CD low for command

    // Set up P4.1 -- SIMO, P4.3 -- SCLK (select PM_UCB1CLK)
    P4SEL |= BIT1 | BIT3;
    P4DIR |= BIT1 | BIT3;

    // Set up backlit
    P7DIR |= BIT6;  // init
    P7OUT |= BIT6;  // backlit
    P7SEL &= ~BIT6; // USE PWM to controll brightness

    // Deselect slave
    P7OUT |= BIT4;  // CS = 1 (Deselect LCD) (stop setting it up)

    UCB1CTL1 |= UCSWRST;    // set UCSWRST bit to disabel USCI and change its control registeres

    UCB1CTL0 = (
        UCCKPH  &   // UCCKPH - 1: change out on second signal change, capture input on first one)
        ~UCCKPL |   // UCCKPL - 0: active level is 1
        UCMSB   |   // MSB comes first, LSB is next
        UCMST   |   // Master mode
        UCSYNC  |   // Synchronious mode
        UCMODE_0    // 3 pin SPI mode
    );

    // set SMCLK as source and keep RESET
    UCB1CTL1 = UCSSEL_2 | UCSWRST;

    // set frequency divider
    UCB1BR0 = 0x01; // LSB to 1
    UCB1BR1 = 0;    // MSB to 0

    UCB1CTL1 &= ~UCSWRST;   // enable USCI
    UCB1IFG &= ~UCRXIFG;    // reset int flag (which is set after input shift register gets data)
    Dogs102x6_writeCommand(LCD_INIT_COMMANDS_PART_1, AMOUNT_OF_COMMANDS_1);

    // delay to wait at least 120 ms
    Delay(12500);

    Dogs102x6_writeCommand(LCD_INIT_COMMANDS_PART_2, AMOUNT_OF_COMMANDS_2);
}

void Delay(long int value)
{
    volatile long int i = 0;
    volatile long int temp = 0;
    for (; i < value; i++)
    {
        temp++;
    }
}

void Dogs102x6_writeCommand(uchar *sCmd, uchar i)
{
    // CS Low
    P7OUT &= ~BIT4;

    // CD Low
    P5OUT &= ~BIT6;
    while (i)
    {
        // USCI_B1 TX buffer ready?
        while (!(UCB1IFG & UCTXIFG)) ;

        // Transmit data
        UCB1TXBUF = *sCmd;

        // Increment the pointer on the array
        sCmd++;

        // Decrement the Byte counter
        i--;
    }

    // Wait for all TX/RX to finish
    while (UCB1STAT & UCBUSY);

    // Dummy read to empty RX buffer and Clear any overrun conditions
    UCB1RXBUF;

    // CS High
    P7OUT |= BIT4;
}

void Clear(void)
{
    uchar lcd_data[] = {0x00};
    uchar page, column;

    for (page = 0; page < 8; page++)
    {
        __LCD_SetAddress(page, 0);
        for (column = 0; column < 132; column++)
        {
            Dogs102x6_writeData(lcd_data, 1);
        }
    }
}


void __LCD_SetAddress(uchar page, uchar column)
{
    uchar cmd[1];

    if (page > 7)
    {
        page = 7;
    }

    if (column > 101)
    {
        column = 101;
    }

    cmd[0] = PAGE_ADR + (7 - page);
    uchar command_high = 0x00;
    uchar command_low = 0x00;
    uchar column_address[] = { COLUMN_ADR_MSB, COLUMN_ADR_LSB };

    command_low = (column & 0x0F);
    command_high = (column & 0xF0);
    command_high = (command_high >> 4);

    column_address[0] = COLUMN_ADR_LSB + command_low;
    column_address[1] = COLUMN_ADR_MSB + command_high;

    Dogs102x6_writeCommand(cmd, 1);
    Dogs102x6_writeCommand(column_address, 2);
}



void Dogs102x6_writeData(uchar *sData, uchar i)
{
    // CS Low
    P7OUT &= ~BIT4;
    //CD High
    P5OUT |= BIT6;

    while (i)
    {
        // USCI_B1 TX buffer ready?
        while (!(UCB1IFG & UCTXIFG));

        // Transmit data and increment pointer
        UCB1TXBUF = *sData++;

        // Decrement the Byte counter
        i--;
    }

    // Wait for all TX/RX to finish
    while (UCB1STAT & UCBUSY);

    // Dummy read to empty RX buffer and Clear any overrun conditions
    UCB1RXBUF;

    // CS High
    P7OUT |= BIT4;
}

int DigitLength(int number) {
    number = abs(number);
    if (number >= 10000) return 5;
    if (number >= 1000) return 4;
    if (number >= 100) return 3;
    if (number >= 10) return 2;
    return 1;
}


void ShowNumber(int number)
{
    volatile int length = 1;
    volatile int digit = 0;
    volatile int j = 0;
    volatile int i = 10;

    length = DigitLength(number);

    int temp = number;
    for(j = 0; j < length; j++)
    {
        digit = (int)(temp % 10);

        digit = digit < 0 ? (-1) * digit : digit;

        if (digit < 10)
            {
            __LCD_SetAddress(0, column_offset + (length-j) * COLUMNS);
            Dogs102x6_writeData(digits[digit][0], COLUMNS);
        }

        temp /= 10;
    }

    if (number >= 0)
    {
        __LCD_SetAddress(0, column_offset);
        Dogs102x6_writeData(plus[0], COLUMNS);
    }
    else
    {
        __LCD_SetAddress(0, column_offset);
        Dogs102x6_writeData(minus[0], COLUMNS);
    }
}

int mg_convert(signed char data_axis)
{
    int mg_data = 0;
    unsigned char i;
    unsigned char mask = 0b01000000;
    unsigned char twos_complement;

    if(data_axis & BIT7) //NEGATIVE NUMBER
    {
        twos_complement = 256 - data_axis;
        for(i = 0; i < 7; i++)
        {
            mg_data += (twos_complement & mask) != 0 ? bit_level_mg_values[i] : 0;
            mask >>= 1;
        }
        mg_data *= -1;
    }
    else
    {
        for(i = 0; i < 7; i++)
        {
            mg_data += (data_axis & mask) != 0 ? bit_level_mg_values[i] : 0;
            mask >>= 1;
        }
    }

    return mg_data;
}

