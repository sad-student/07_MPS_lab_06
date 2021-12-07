#include <msp430.h>
#include "CTS_Layer.h"
#include <math.h>
#include "font.h"

#define TAxCCR_05Hz 0xffff /* timer upper bound count value */
#define BUTTON_DELAY 0x0380
#define DISPLAY_RATE 5

unsigned int display_available = 1;

void writeCommand(unsigned char *sCmd, unsigned char i);
void writeData(unsigned char *sData, unsigned char i);
void setPosition(unsigned char page, unsigned char col);
void printNumber(int num);
void printSymbol(int index, unsigned char page, unsigned int col);
void Delay(long int value);

struct Element* keypressed;

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

//void SetupTimer()
//{
//    // setup timer
//    TA0CTL = TASSEL__SMCLK | MC__UP | ID__1 | TACLR;     // SMCLK, UP-mode
//    long int time = 32768;
//    long int period = time / 2;
//    TA0CCR0 = time;
//    TA0CCR1 = period;
//    TA0CCTL1 = OUTMOD_3;
//}

void SetupADC()
{

//    REFCTL0 &= ~REFMSTR;        // turn of REF block
//
//    ADC12CTL0 =
//        ADC12SHT0_8
//        + ADC12REFON
//        + ADC12ON;
//
//    // Internal ref = 1.5V
//
//    ADC12CTL1 =
//        ADC12SHP
//        + ADC12SHS_1
//        + ADC12SSEL_0
//        + ADC12CONSEQ_2;  // enable sample timer
//
//    ADC12MCTL0 =
//        ADC12SREF_1
//        + ADC12INCH_10;    // ADC i/p ch A10 = temp sense i/pf
//
//    ADC12IE = ADC12IE0;                         // ADC_IFG upon conv result-ADCMEMO
//
//    __delay_cycles(100);                        // delay to allow Ref to settle
//
//    ADC12CTL0 |= ADC12ENC;

    //

    REFCTL0 &= ~REFMSTR;        // turn off REF block
	// TODO: set reference voltage
	ADC12MCTL0 = (ADC12MCTL0 & (~0x0ff)) | ((ADC12INCH_10 & (0x0f)) | (ADC12SREF_1 & (0x070)) | (0 & (0x070) | (ADC12EOS & (0x80))));

	// TODO: set divider
	// Set sampling mode (12-15, 9, 7-5, 4-3, 2-1)
	ADC12CTL1 =
			(ADC12CTL1 & (~0x0fd7e)) | ((ADC12CSTARTADD_0 & (0x0f000)) | (ADC12SHS_0 & (0x0c00)) | (ADC12SHP & (0x0100)) |
					(ADC12DIV_0 & (0x0e0)) | (ADC12SSEL_0 & (0x018)) | (ADC12CONSEQ_0 & (0x05)));
	// Required for temperature sensor
	ADC12CTL0 = (ADC12CTL0 & (~0x020)) | (ADC12REFON & (0x020));
	// Enable ADC
	ADC12CTL0 = (ADC12CTL0 & (~0x010)) | (ADC12ON & (0x010));
	ADC12CTL2 = (ADC12CTL2 & (~0x01b0)) | ((~ADC12PDIV & (0x0100)) | (~ADC12TCOFF & (0x080)) | (ADC12RES_2 & (0x030)));

//	// TODO: set divider
//	// Set sampling mode (12-15, 9, 7-5, 4-3, 2-1)
//	ADC12CTL1 =
//			(ADC12CTL1 & (~0x0f17e)) | ((ADC12CSTARTADD_0 & (0x0f000)) | (ADC12SHP & (0x0100)) |
//					(ADC12DIV_0 & (0x0e0)) | (ADC12SSEL_3 & (0x018)) | (ADC12CONSEQ_0 & (0x05)));
	// Resolution and divider
	//ADC12CTL2 = (ADC12CTL2 & (~0x01b0)) | ((ADC12PDIV & (0x0100)) | (ADC12RES_2 & (0x030)));
	// Start sampling
	// ADC12CTL0 = (ADC12CTL0 & (~0x01)) | (ADC12SC & (0x01));

	// Enable interrupts
	ADC12IE = ADC12IE0;

    __delay_cycles(100);                        // delay to allow Ref to settle

	ADC12CTL0 |= ADC12ENC;
}

#define CALADC12_15V_30C *((unsigned int *)0x1A1A) // Temperature Sensor Calibration-30 C
#define CALADC12_15V_85C *((unsigned int *)0x1A1C) // Temperature Sensor Calibration-85 C

#pragma vector = ADC12_VECTOR
__interrupt void ADC12_ISR() {
//    TA0CTL &= ~MC__UP;
//    TA0CTL |= MC__STOP | TACLR;
//    TA0CCTL1 &= ~BIT2;
	ADC12CTL0 = (ADC12CTL0 & (~0x01)) | (~ADC12SC & (0x01));

    // also clears ADC12IFG0 flag
    unsigned short int probe = ADC12MEM0 & 0x0FFF;
    ADC12MEM0 = 0;

    volatile long int temp1 = probe;
    volatile long int temp2 = temp1 - CALADC12_15V_30C;
    volatile float temp3 = temp2 * (85 - 30);
    volatile float temp4 = temp3 / (CALADC12_15V_85C - CALADC12_15V_30C);
    temp4 += 30;
    volatile long int deg_c = (long int)temp4;

    printNumber(deg_c);

//    ADC12CTL0 &= ~ADC12ENC;
}

#pragma vector = TIMER2_A1_VECTOR
__interrupt void TA2_handler(void){
	switch(TA2IV){
		case TA2IV_TACCR2:
			// Display
			display_available = 1;
			TA2CCTL2 = (TA2CCTL2 & (~0x010)) | (~CCIE & (0x010)); // & ~CCIE;
			break;
		default:
			break;
	}
}

void writeCommand(unsigned char *sCmd, unsigned char i) {
    // Store current GIE state
    unsigned int gie = __get_SR_register() & GIE;
    // Make this operation atomic
    __disable_interrupt();
    // CS Low
    P7OUT &= ~BIT4;
    // CD Low
    P5OUT &= ~BIT6;
    while (i){
        // USCI_B1 TX buffer ready?
        while (!(UCB1IFG & UCTXIFG)) ;
        // Transmit data
        UCB1TXBUF = *sCmd++;
        // Decrement the Byte counter
        i--;
    }

    // Wait for all TX/RX to finish
    while (UCB1STAT & UCBUSY) ;
    // Dummy read to empty RX buffer and clear any overrun conditions
    UCB1RXBUF;
    // CS High
    P7OUT |= BIT4;
    // Restore original GIE state
    __bis_SR_register(gie);
}

void writeData(unsigned char *sData, unsigned char i) {
    // Store current GIE state
    unsigned int gie = __get_SR_register() & GIE;
    // Make this operation atomic
    __disable_interrupt();
	// CS Low
	P7OUT &= ~BIT4;
	//CD High
	P5OUT |= BIT6;

	while (i){
	  // USCI_B1 TX buffer ready?
	  while (!(UCB1IFG & UCTXIFG)) ;
	  // Transmit data and increment pointer
	  UCB1TXBUF = *sData++;
	  // Decrement the Byte counter
	  i--;
	}

	// Wait for all TX/RX to finish
	while (UCB1STAT & UCBUSY) ;
	// Dummy read to empty RX buffer and clear any overrun conditions
	UCB1RXBUF;
	// CS High
	P7OUT |= BIT4;

    // Restore original GIE state
    __bis_SR_register(gie);
}

#define SET_COLUMN_ADDRESS_MSB		0x10
#define SET_COLUMN_ADDRESS_LSB		0x00
#define SET_PAGE_ADDRESS			0xB0
void setPosition(unsigned char page, unsigned char col){
	unsigned char cmd_6[3] = {SET_COLUMN_ADDRESS_MSB, SET_COLUMN_ADDRESS_LSB, SET_PAGE_ADDRESS};
	cmd_6[0] = (cmd_6[0] & (~0x0f)) | ((col >> 4) & (0x0f));
	cmd_6[1] = (cmd_6[1] & (~0x0f)) | (col & 0x0f);
	cmd_6[2] = (cmd_6[2] & (~0x0f)) | (page & 0x0f);
	writeCommand(cmd_6, 3);
}

void printNumber(int num){
	if (display_available == 0) {
		return;
	}
	display_available = 0;

	unsigned char current_page = 0;
	unsigned char current_col = 0;

	if (num < 0) {
		num = -num;
		printSymbol(11, current_page, current_col);
	} else {
		printSymbol(10, current_page, current_col);
	}
	current_col += 8;

	unsigned char indices[4] = {13, 12, 12, 12};
	unsigned int current_pos = sizeof(indices);
	do {
		indices[--current_pos] = num % 10;
		num /= 10;
	} while (num > 0);

	int i;
	for (i = current_pos; i < current_pos + sizeof(indices); ++i){
		printSymbol(indices[i % sizeof(indices)], current_page, current_col);
		current_col += 8;
	}
	TA2CCR2 = TA2R + (TAxCCR_05Hz / DISPLAY_RATE);
	TA2CCTL2 = (TA2CCTL2 & (~0x010)) | CCIE;
}

void printSymbol(int index, unsigned char page, unsigned int col){
	setPosition(page+1, col);
	writeData(_font[index], 6);
	setPosition(page, col);
	writeData(_font[index] + 6, 6);
}

int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;
    __bis_SR_register(GIE);

    uint8_t i;

    /* Initialize IO */
    P1DIR = 0xFF;
    P1OUT = 0;

//    UCSCTL3 = SELREF_2;                       // Set DCO FLL reference = REFO
//    UCSCTL4 |= SELA_2;                        // Set ACLK = REFO
//
//    __bis_SR_register(SCG0);                  // Disable the FLL control loop
//    UCSCTL0 = 0x0000;                         // Set lowest possible DCOx, MODx
//    UCSCTL1 = DCORSEL_7;
//    // Select DCO range 50MHz operation
//    UCSCTL2 = FLLD_1 + 762;                   // Set DCO Multiplier for 25MHz
//                                            // (N + 1) * FLLRef = Fdco
//                                            // (762 + 1) * 32768 = 25MHz
//                                            // Set FLL Div = fDCOCLK/2
//    __bic_SR_register(SCG0);                  // Enable the FLL control loop

    UCSCTL3 = (UCSCTL3 & (~0x070)) | SELREF__XT1CLK;
	UCSCTL3 = (UCSCTL3 & (~0x07)) | FLLREFDIV__2;
	UCSCTL2 = (UCSCTL2 & (~0x0cff)) | ((8 - 1) & (0x0cff)); // FLLN multiplier
	UCSCTL2 = (UCSCTL2 & (~0x07000)) | FLLD__8; // FLLD divider
	UCSCTL1 = (UCSCTL1 & (~0x070)) | DCORSEL_1;

	UCSCTL4 = (UCSCTL4 & (~0x070)) | SELS__DCOCLKDIV;
	UCSCTL5 = (UCSCTL5 & (~0x070)) | DIVS__1;

	TA2CTL = (TA2CTL & (~0x0300)) | TASSEL__SMCLK;
	TA2CTL = (TA2CTL & (~0x030)) | MC__CONTINOUS;
	TA2CTL = (TA2CTL & (~0x0c0)) | ID__4;
	TA2CTL |= TACLR;
	TA2CCTL1 = (TA2CCTL1 & (~0x0100)) & ~CAP;
	TA2CCTL2 = (TA2CCTL2 & (~0x0100)) & ~CAP;
	TA2CCTL1 = (TA2CCTL1 & (~0x010)) & ~CCIE;
	TA2CCTL2 = (TA2CCTL2 & (~0x010)) & ~CCIE;

	// UART initialization
	// Initialize USCI_B1 for SPI Master operation
	UCB1IFG = (UCB1IFG & (~0x03)) | ((~UCTXIFG & (0x02)) | (~UCRXIFG & (0x01)));
	UCB1IE = (UCB1IE & (~0x03)) | ((~UCTXIE & 0x02)| (~UCRXIE & 0x01));
	// UCB1IE = (UCB1IE & (~0x03)) | UCTXIE | UCRXIE;
	// Put state machine in reset
	UCB1CTL1 |= UCSWRST;
	//3-pin, 8-bit SPI master; Clock phase - data captured first edge, change second edge; MSB
	UCB1CTL0 = UCCKPH + UCMSB + UCMST + UCMODE_0 + UCSYNC;
	// Use SMCLK, keep RESET
	UCB1CTL1 = UCSSEL_2 + UCSWRST;
	UCB1BR1 = 0;
	UCB1BR0 = 0x02;
	// Release USCI state machine
	UCB1CTL1 &= ~UCSWRST;


    // LCD initialization
	// Chip select and screen backlight
	P7SEL &= ~(BIT4 | BIT6);
	P7DIR |= (BIT4 | BIT6);

	// Reset and Command/Data
	P5SEL &= ~(BIT6 | BIT7);
	P5DIR |= (BIT6 | BIT7);

	// Option select SIMO and select CLK
	P4SEL |= (BIT1 | BIT3);
	P4DIR |= (BIT1 | BIT3);

	// Port initialization for LCD operation
	// CS is active low
	P7OUT |= BIT4;
	// Reset is active low
	P5OUT &= BIT7;
	// Reset is active low
	P5OUT |= BIT7;
	// Enable screen backlight
	P7OUT |= BIT6;



#define SET_SCROLL_LINE		0x40
#define SET_MX				0xA0
#define SET_MY				0xC0
#define SET_ALL_PIXEL_ON	0xA4
#define SET_INVERSE_DISPLAY		0xA6
#define SET_PM_MSB			0x81
#define SET_PM_LSB			0x00
#define SET_POWER_CONTROL	0x28
#define SET_PC				0x20
#define SET_BIAS_RATIO		0xA2
#define SET_ADV_CONTROL_MSB	0xFA
#define SET_ADV_CONTROL_LSB	0x10
#define SET_DISPLAY_ENABLE	0xAE
#define SYSTEM_RESET		0xE2

	{
		unsigned char cmd_1[] = {
			SET_SCROLL_LINE,		// 0
			SET_MX,					// 1
			SET_MY,					// 2
			SET_ALL_PIXEL_ON,		// 3
			SET_INVERSE_DISPLAY,	// 4
			SET_PM_MSB,				// 5
			SET_PM_LSB,				// 6
			SET_POWER_CONTROL,		// 7
			SET_PC,					// 8
			SET_BIAS_RATIO,			// 9
			SET_ADV_CONTROL_MSB,	// 10
			SET_ADV_CONTROL_LSB,	// 11
			SET_DISPLAY_ENABLE		// 12
		};

		cmd_1[1] = (cmd_1[1] & (~0x01)) | (BIT0 & 0x01);
		cmd_1[2] = (cmd_1[2] & (~0x08)) | (BIT3 & 0x08);
		cmd_1[3] = (cmd_1[3] & (~0x01)) | BIT0;
		cmd_1[6] = (cmd_1[6] & (~0x03f)) | (0x030 & (0x03f));
		cmd_1[7] = (cmd_1[7] & (~0x07)) | ((BIT0 | BIT1 | BIT2) & (0x07));
		cmd_1[8] = (cmd_1[8] & (~0x07)) | (0x4 & (0x07));
		cmd_1[11] = (cmd_1[11] & (~0x083)) | ((BIT7) & (0x83));
		cmd_1[12] = (cmd_1[12] & (~0x01)) | BIT0;

		writeCommand(cmd_1, sizeof(cmd_1));

		int i, j = 0;
		for (i = 0; i < 132; i+=sizeof(_font[12])){
			for (j = 0; j < 8; ++j){
				setPosition(j, i);
				writeData(_font[12], sizeof(_font[12]));
			}
		}

		cmd_1[3] = (cmd_1[3] & (~0x01)) | (~BIT0 & (0x01));
		writeCommand(cmd_1 + 3, 1);
	}

    /*  establish baseline */
    TI_CAPT_Init_Baseline(&keypad);
    TI_CAPT_Update_Baseline(&keypad, 5);
    SetupADC();
//    SetupTimer();
//    ADC12CTL0 |= ADC12ENC;
    setUp(0x01);setUp(0x02);setUp(0x03);

    while (1)
    {
        P1OUT &= ~BIT1;
        keypressed = (struct Element *) TI_CAPT_Buttons(&keypad);

        if (keypressed && keypressed == &PAD1)
        {
            P1OUT |= BIT1;

            if (!(ADC12CTL1 & ADC12BUSY)) // if there is no active operation
            {
//                SetupTimer();
//                ADC12CTL0 |= ADC12ENC;


            	ADC12CTL0 = (ADC12CTL0 & (~0x01)) | (ADC12SC & (0x01));
            }
        }
        __delay_cycles(900000);
    }
    return 0;
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

