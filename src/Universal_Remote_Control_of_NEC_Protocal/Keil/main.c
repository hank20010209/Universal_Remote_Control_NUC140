#include <stdio.h>
#include <string.h>
#include "NUC100Series.h"
#include "MCU_init.h"
#include "SYS_init.h"

#define SIGNAL_BUFFER_SIZE 16

volatile uint32_t central_time_10us = 0;
volatile uint32_t central_time_5ms = 0;

// For record signals
char signals_names[SIGNAL_BUFFER_SIZE][17] = {" SAMPLE0", " SAMPLE1", " SAMPLE2", " SAMPLE3"};
uint8_t signals[SIGNAL_BUFFER_SIZE][4] = {{0}};
uint8_t signals_index = 4;

// For User Interface
uint8_t PAGE_INDEX = 0;
uint8_t PAGE_FLAG = 0;
uint8_t CURSOR_ROW_MIN_INDEX = 0;
uint8_t CURSOR_ROW_MAX_INDEX = 0;
uint8_t CURSOR_COLUMN_MIN_INDEX = 0;
uint8_t CURSOR_COLUMN_MAX_INDEX = 0;

volatile uint8_t SCROLL_INDEX = 0;
volatile uint8_t CURSOR_ROW_INDEX = 0;
volatile uint8_t CURSOR_COLUMN_INDEX = 0;
volatile uint8_t CURSOR_STATE = 0;
volatile uint8_t CURSOR_FLAG = 0;
volatile uint8_t CURSOR_SHOW = 0;
volatile uint8_t CURSOR_CHANGE = 0;
volatile uint8_t NAME_INDEX = 0;
volatile uint8_t PAGE_TIMER = 0;
char main_page[2][17] = {" Capture", " Transmit"};
char capture_page[4][17] = {"Capturing...", "Captured: ", " Save", " Cancel"};
char name_page[4][17] = {"Input Name:", "OokamiMio", "ABCDEFGHIJKLMNOP", "QRSTUVWXYZ-DELOK"};
char record_page[1][17] = {"No Signal Record"};
char print_buffer[17] = "";

// For signal process
uint32_t time_buffer[256];
uint8_t signal_buffer[256];
volatile uint32_t SIGNAL_CAPTURE_START_TIME = 0;
volatile uint8_t SIGNAL_CAPTURE_INDEX = 0;
volatile uint8_t SIGNAL_CAPTURE_STATE = 0;
volatile uint8_t SIGNAL_CAPTURE_WDT = 0;
volatile uint8_t SIGNAL_CAPTURE_END = 0;

// For keypad scan
volatile uint8_t KEYLINE_INDEX = 0;
volatile uint8_t CAPTURED_KEY = 0;
volatile uint8_t PREVIOUS_KEY = 0;
volatile uint8_t CURRENT_KEY = 0;

void Init_Timer0(void);
void TMR0_IRQHandler(void);
void Init_Timer1(void);
void TMR1_IRQHandler(void);
void Init_Timer2(void);
void TMR2_IRQHandler(void);
void Init_Signal_Capturer(void);
void GPCDEF_IRQHandler(void);
void Init_Keypad_Capturer(void);
void GPAB_IRQHandler(void);
void Init_EXTINT(void);
void EINT1_IRQHandler(void);
void display_cursor(void);
void clear_cursor(void);
void display_name_block(void);
void clear_name_block(void);

// For central time 10us
void Init_Timer0(void)
{
    TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, 100000);
    TIMER_EnableInt(TIMER0);
    NVIC_EnableIRQ(TMR0_IRQn);
    TIMER_Start(TIMER0);
}
void TMR0_IRQHandler(void)
{
    central_time_10us++;
    TIMER_ClearIntFlag(TIMER0); // Clear Timer1 time-out interrupt flag
}

// For signal capture wdt
// Frequency 1000Hz (1ms)
void Init_Timer1(void)
{
    TIMER_Open(TIMER1, TIMER_PERIODIC_MODE, 1000);
    TIMER_EnableInt(TIMER1);
    NVIC_EnableIRQ(TMR1_IRQn);
}
void TMR1_IRQHandler(void)
{
    SIGNAL_CAPTURE_WDT++;

    // End 60ms end the signal capture and reset capture variable;
    // Decode signal time line and record;
    if (SIGNAL_CAPTURE_WDT > 60)
    {
        // Ensure capture the signal header
        if (SIGNAL_CAPTURE_INDEX > 2)
        {
            uint32_t header_high = time_buffer[1] - time_buffer[0];
            uint32_t header_low = time_buffer[2] - time_buffer[1];
            // Check signal match NEC header format +- 5%
            if (header_high > 850 && header_high < 950 && header_low > 425 && header_low < 475)
            {
                uint32_t data = 0, reverse_data = 0;
                uint8_t i, j, size, split[4];

                /*
                for(i = 0; i < SIGNAL_CAPTURE_INDEX; i++)printf("bits: %d time: %d\n", signal_buffer[i], time_buffer[i] - time_buffer[i-1]);
                printf("\n");
                */
                for (i = 4, size = 0; i < SIGNAL_CAPTURE_INDEX && size < 32; i += 2, size++)
                {
                    data = (data << 1) | (time_buffer[i] - time_buffer[i - 1] > 100);
                }

                if (size == 32)
                {
                    for (i = 0; i < 4; i++)
                    {
                        for (j = 0; j < 8; j++)
                        {
                            reverse_data |= (((data & (1 << (i * 8 + j))) ? 1 : 0) << ((i + 1) * 8 - j - 1));
                        }
                    }
                    // printf("%08X,  %08X\n", data, reverse_data);

                    // split to SendNEC function format.
                    memcpy(split, &reverse_data, sizeof(uint32_t));
                    signals[signals_index][0] = split[3];
                    signals[signals_index][1] = split[2];
                    signals[signals_index][2] = split[1];
                    signals[signals_index][3] = split[0];

                    // signals_index++;
                    // printf("signal_index:%d\n", signals_index);
                    NVIC_DisableIRQ(GPCDEF_IRQn);
                    // switch page & set cursor
                    PAGE_FLAG = 1;
                    PAGE_INDEX = 11;
                    CURSOR_SHOW = 1;
                }
            }
        }

        // Reinit capture variable
        SIGNAL_CAPTURE_START_TIME = 0;
        SIGNAL_CAPTURE_INDEX = 0;
        SIGNAL_CAPTURE_STATE = 0;
        SIGNAL_CAPTURE_WDT = 0;
        SIGNAL_CAPTURE_END = 1;

        // Disable capture wdt
        TIMER_Stop(TIMER1);
        // Disable capturer
    }
    TIMER_ClearIntFlag(TIMER1); // Clear Timer1 time-out interrupt flag
}

void Init_Timer2(void)
{
    TIMER_Open(TIMER2, TIMER_PERIODIC_MODE, 200);
    TIMER_EnableInt(TIMER2);
    NVIC_EnableIRQ(TMR2_IRQn);
    TIMER_Start(TIMER2);
}
void TMR2_IRQHandler(void)
{
    central_time_5ms++;
    // 100ms cycle
    if (central_time_5ms % 10 == 0)
    {
        KEYLINE_INDEX = (KEYLINE_INDEX + 1) % 4;
        CAPTURED_KEY = 0;

        if (KEYLINE_INDEX == 0)
        {
            PA0 = 1, PA1 = 1, PA2 = 1, PA3 = 1, PA4 = 1, PA5 = 0;
        }
        else if (KEYLINE_INDEX == 1)
        {
            PA0 = 1, PA1 = 1, PA2 = 1, PA3 = 1, PA4 = 0, PA5 = 1;
        }
        else if (KEYLINE_INDEX == 2)
        {
            PA0 = 1, PA1 = 1, PA2 = 1, PA3 = 0, PA4 = 1, PA5 = 1;
        }
        else if (KEYLINE_INDEX == 3)
        {
            PREVIOUS_KEY = CURRENT_KEY;
            CURRENT_KEY = 0;
        }
        NVIC_EnableIRQ(GPAB_IRQn);
    }

    if (central_time_5ms % 100 == 0)
    {
        CURSOR_FLAG = CURSOR_SHOW;
    }
    if (central_time_5ms % 200 == 0 && PAGE_INDEX == 31 && PAGE_TIMER)
    {
        PAGE_TIMER--;
        if (PAGE_TIMER == 0)
        {
            PAGE_INDEX = 0;
            PAGE_FLAG = 1;
            CURSOR_ROW_INDEX = 0;
            CURSOR_COLUMN_INDEX = 0;
            CURSOR_ROW_MIN_INDEX = 0;
            CURSOR_ROW_MAX_INDEX = 1;
            CURSOR_SHOW = 1;
            CURSOR_CHANGE = 1;
        }
    }

    TIMER_ClearIntFlag(TIMER2); // Clear Timer1 time-out interrupt flag
}

void Init_Signal_Capturer(void)
{
    GPIO_SetMode(PC, BIT0, GPIO_MODE_QUASI);
    GPIO_EnableInt(PC, 0, GPIO_INT_BOTH_EDGE);
    // NVIC_EnableIRQ(GPCDEF_IRQn);
    NVIC_SetPriority(GPCDEF_IRQn, 3);
}

void GPCDEF_IRQHandler(void)
{
    // Capture start
    if (SIGNAL_CAPTURE_START_TIME == 0)
    {
        SIGNAL_CAPTURE_START_TIME = central_time_10us;

        // Enable capture wdt
        TIMER_Start(TIMER1);
    }

    SIGNAL_CAPTURE_STATE = !SIGNAL_CAPTURE_STATE;

    signal_buffer[SIGNAL_CAPTURE_INDEX] = SIGNAL_CAPTURE_STATE;
    time_buffer[SIGNAL_CAPTURE_INDEX] = central_time_10us - SIGNAL_CAPTURE_START_TIME;

    SIGNAL_CAPTURE_INDEX++;

    SIGNAL_CAPTURE_WDT = 0; // feed dog

    PC->ISRC = PC->ISRC; // clear all GPC pins
}

void Init_Keypad_Capturer(void)
{
    GPIO_SetMode(PA, (BIT0 | BIT1 | BIT2 | BIT3 | BIT4 | BIT5), GPIO_MODE_QUASI);
    GPIO_EnableInt(PA, 0, GPIO_INT_LOW);
    GPIO_EnableInt(PA, 1, GPIO_INT_LOW);
    GPIO_EnableInt(PA, 2, GPIO_INT_LOW);
    NVIC_EnableIRQ(GPAB_IRQn);
    NVIC_SetPriority(GPAB_IRQn, 3);
    GPIO_SET_DEBOUNCE_TIME(GPIO_DBCLKSRC_LIRC, GPIO_DBCLKSEL_256);
    GPIO_ENABLE_DEBOUNCE(PA, (BIT0 | BIT1 | BIT2));
}

void GPAB_IRQHandler(void)
{
    NVIC_DisableIRQ(GPAB_IRQn);
    if (PA->ISRC & BIT0)
    { // check if PA0 interrupt occurred
        PA0 = 1;
        PA->ISRC |= BIT0; // clear PA0 interrupt status
        if (PA3 == 0)
            CAPTURED_KEY = 3, PA3 = 1;
        if (PA4 == 0)
            CAPTURED_KEY = 6, PA4 = 1;
        if (PA5 == 0)
            CAPTURED_KEY = 9, PA5 = 1;
    }
    else if (PA->ISRC & BIT1)
    { // check if PA1 interrupt occurred
        PA1 = 1;
        PA->ISRC |= BIT1; // clear PA1 interrupt status
        if (PA3 == 0)
            CAPTURED_KEY = 2, PA3 = 1;
        if (PA4 == 0)
            CAPTURED_KEY = 5, PA4 = 1;
        if (PA5 == 0)
            CAPTURED_KEY = 8, PA5 = 1;
    }
    else if (PA->ISRC & BIT2)
    { // check if PB14 interrupt occurred
        PA2 = 1;
        PA->ISRC |= BIT2; // clear PA interrupt status
        if (PA3 == 0)
            CAPTURED_KEY = 1, PA3 = 1;
        if (PA4 == 0)
            CAPTURED_KEY = 4, PA4 = 1;
        if (PA5 == 0)
            CAPTURED_KEY = 7, PA5 = 1;
    }
    if (!CURRENT_KEY && CAPTURED_KEY)
        CURRENT_KEY = CAPTURED_KEY;

    PA->ISRC = PA->ISRC; // clear all GPB pins
}

void Init_EXTINT(void)
{
    // Configure EINT1 pin and enable interrupt by rising and falling edge trigger
    GPIO_SetMode(PB, BIT15, GPIO_MODE_INPUT);
    GPIO_EnableEINT1(PB, 15, GPIO_INT_RISING); // RISING, FALLING, BOTH_EDGE, HIGH, LOW
    NVIC_EnableIRQ(EINT1_IRQn);

    // Enable interrupt de-bounce function and select de-bounce sampling cycle time
    GPIO_SET_DEBOUNCE_TIME(GPIO_DBCLKSRC_LIRC, GPIO_DBCLKSEL_64);
    GPIO_ENABLE_DEBOUNCE(PB, BIT15);
}

void EINT1_IRQHandler(void)
{
    // printf("page index: %d\n", PAGE_INDEX);
    if (PAGE_INDEX % 10 == 1)
    {
        if (PAGE_INDEX == 1)
        {
            NVIC_DisableIRQ(GPCDEF_IRQn);
            TIMER_Stop(TIMER1);
            SIGNAL_CAPTURE_START_TIME = 0;
            SIGNAL_CAPTURE_INDEX = 0;
            SIGNAL_CAPTURE_STATE = 0;
            SIGNAL_CAPTURE_WDT = 0;
            SIGNAL_CAPTURE_END = 1;

            CURSOR_ROW_INDEX = 0;
            CURSOR_COLUMN_INDEX = 0;
            CURSOR_ROW_MIN_INDEX = 0;
            CURSOR_ROW_MAX_INDEX = 1;
            CURSOR_SHOW = 1;
            CURSOR_CHANGE = 1;
            PAGE_INDEX = 0;
        }
        else if (PAGE_INDEX == 11)
        {
            CURSOR_SHOW = 0;
            CURSOR_ROW_INDEX = 2;
            NVIC_EnableIRQ(GPCDEF_IRQn);
            PAGE_INDEX = 1;
        }
        else if (PAGE_INDEX == 21)
        {
            clear_name_block();
            CURSOR_SHOW = 1;
            CURSOR_ROW_INDEX = 2;
            CURSOR_COLUMN_INDEX = 0;
            CURSOR_ROW_MIN_INDEX = 2;
            CURSOR_ROW_MAX_INDEX = 3;
            CURSOR_COLUMN_MIN_INDEX = 0;
            CURSOR_COLUMN_MAX_INDEX = 15;
            PAGE_INDEX = 11;
        }
    }
    else if (PAGE_INDEX % 10 == 2)
    {
        CURSOR_ROW_INDEX = 0;
        CURSOR_COLUMN_INDEX = 0;
        CURSOR_ROW_MIN_INDEX = 0;
        CURSOR_ROW_MAX_INDEX = 1;
        CURSOR_SHOW = 1;
        CURSOR_CHANGE = 1;
        PAGE_INDEX = 0;
    }

    PAGE_FLAG = 1;
    GPIO_CLR_INT_FLAG(PB, BIT15); // Clear GPIO interrupt flag
}

void display_cursor(void)
{
    if (CURSOR_FLAG && CURSOR_CHANGE)
    {
        printC(CURSOR_COLUMN_INDEX * 8, CURSOR_ROW_INDEX * 16, (CURSOR_STATE = !CURSOR_STATE) ? '>' : ' ');
        CURSOR_FLAG = 0;
    }
}
void clear_cursor(void)
{
    if (CURSOR_SHOW)
    {
        printC(CURSOR_COLUMN_INDEX * 8, CURSOR_ROW_INDEX * 16, ' ');
        CURSOR_STATE = 0;
    }
}
void display_name_block(void)
{
    if (PAGE_INDEX == 21 && CURSOR_CHANGE)
    {
        if (CURSOR_ROW_INDEX == 2 || CURSOR_COLUMN_INDEX < 11)
        {
            printInverseC(CURSOR_COLUMN_INDEX * 8, CURSOR_ROW_INDEX * 16, name_page[CURSOR_ROW_INDEX][CURSOR_COLUMN_INDEX]);
        }
        else if (CURSOR_COLUMN_INDEX < 14)
        {
            printInverseC(11 * 8, CURSOR_ROW_INDEX * 16, name_page[CURSOR_ROW_INDEX][11]);
            printInverseC(12 * 8, CURSOR_ROW_INDEX * 16, name_page[CURSOR_ROW_INDEX][12]);
            printInverseC(13 * 8, CURSOR_ROW_INDEX * 16, name_page[CURSOR_ROW_INDEX][13]);
        }
        else
        {
            printInverseC(14 * 8, CURSOR_ROW_INDEX * 16, name_page[CURSOR_ROW_INDEX][14]);
            printInverseC(15 * 8, CURSOR_ROW_INDEX * 16, name_page[CURSOR_ROW_INDEX][15]);
        }
    }
}
void clear_name_block(void)
{
    if (PAGE_INDEX == 21)
    {
        if (CURSOR_ROW_INDEX == 2 || CURSOR_COLUMN_INDEX < 11)
        {
            printC(CURSOR_COLUMN_INDEX * 8, CURSOR_ROW_INDEX * 16, name_page[CURSOR_ROW_INDEX][CURSOR_COLUMN_INDEX]);
        }
        else if (CURSOR_COLUMN_INDEX < 14)
        {
            printC(11 * 8, CURSOR_ROW_INDEX * 16, name_page[CURSOR_ROW_INDEX][11]);
            printC(12 * 8, CURSOR_ROW_INDEX * 16, name_page[CURSOR_ROW_INDEX][12]);
            printC(13 * 8, CURSOR_ROW_INDEX * 16, name_page[CURSOR_ROW_INDEX][13]);
        }
        else
        {
            printC(14 * 8, CURSOR_ROW_INDEX * 16, name_page[CURSOR_ROW_INDEX][14]);
            printC(15 * 8, CURSOR_ROW_INDEX * 16, name_page[CURSOR_ROW_INDEX][15]);
        }
        CURSOR_CHANGE = 1;
    }
}

int main(void)
{
    SYS_Init();
    Init_Timer0();
    Init_Timer1();
    Init_Timer2();
    Init_EXTINT();
    Init_Signal_Capturer();
    Init_Keypad_Capturer();
    IrDA_NEC_TxRx_Init();

    GPIO_SetMode(PD, BIT14, GPIO_MODE_OUTPUT);
    PD14 = 0;
    init_LCD();
    clear_LCD();
    print_Line(0, main_page[0]);
    print_Line(1, main_page[1]);
    CURSOR_ROW_MIN_INDEX = 0;
    CURSOR_ROW_MAX_INDEX = 1;
    CURSOR_SHOW = 1;
    CURSOR_CHANGE = 1;
    SCROLL_INDEX = 0;
    // printf("start debug\n");

    while (1)
    {
        // Press key
        if (CAPTURED_KEY > 0 && PREVIOUS_KEY == 0)
        {
            switch (CAPTURED_KEY)
            {
            case 2:
                if (CURSOR_ROW_INDEX > CURSOR_ROW_MIN_INDEX)
                {
                    clear_cursor();
                    clear_name_block();
                    CURSOR_ROW_INDEX--;
                }
                else if (PAGE_INDEX == 2 && CURSOR_ROW_INDEX == CURSOR_ROW_MIN_INDEX && SCROLL_INDEX)
                {
                    SCROLL_INDEX--;
                    PAGE_FLAG = 1;
                }
                break;
            case 8:
                if (CURSOR_ROW_INDEX < CURSOR_ROW_MAX_INDEX)
                {
                    clear_cursor();
                    clear_name_block();
                    CURSOR_ROW_INDEX++;
                }
                else if (PAGE_INDEX == 2 && CURSOR_ROW_INDEX == CURSOR_ROW_MAX_INDEX && SCROLL_INDEX + 3 < signals_index - 1)
                {
                    SCROLL_INDEX++;
                    PAGE_FLAG = 1;
                }
                break;
            case 4:
                if (CURSOR_COLUMN_INDEX > CURSOR_COLUMN_MIN_INDEX)
                {
                    clear_cursor();
                    clear_name_block();
                    CURSOR_COLUMN_INDEX--;
                    if (PAGE_INDEX == 21 && CURSOR_ROW_INDEX == 3)
                    {
                        if (CURSOR_COLUMN_INDEX == 14 || CURSOR_COLUMN_INDEX == 15)
                        {
                            CURSOR_COLUMN_INDEX = 13;
                        }
                        else if (CURSOR_COLUMN_INDEX == 11 || CURSOR_COLUMN_INDEX == 12)
                        {
                            CURSOR_COLUMN_INDEX = 10;
                        }
                    }
                }
                else if (CURSOR_COLUMN_MIN_INDEX != CURSOR_COLUMN_MAX_INDEX)
                {
                    clear_cursor();
                    clear_name_block();
                    CURSOR_COLUMN_INDEX = CURSOR_COLUMN_MAX_INDEX;
                }
                break;
            case 6:
                if (CURSOR_COLUMN_INDEX < CURSOR_COLUMN_MAX_INDEX)
                {
                    clear_cursor();
                    clear_name_block();
                    CURSOR_COLUMN_INDEX++;
                    if (PAGE_INDEX == 21 && CURSOR_ROW_INDEX == 3)
                    {
                        if (CURSOR_COLUMN_INDEX == 12 || CURSOR_COLUMN_INDEX == 13)
                        {
                            CURSOR_COLUMN_INDEX = 14;
                        }
                        else if (CURSOR_COLUMN_INDEX == 15)
                        {
                            CURSOR_COLUMN_INDEX = 0;
                        }
                    }
                }
                else if (CURSOR_COLUMN_MIN_INDEX != CURSOR_COLUMN_MAX_INDEX)
                {
                    clear_cursor();
                    clear_name_block();
                    CURSOR_COLUMN_INDEX = CURSOR_COLUMN_MIN_INDEX;
                }
                break;
            case 5:
                if (PAGE_INDEX % 10 == 0)
                {
                    // first selection
                    if (CURSOR_ROW_INDEX == 0)
                    {
                        // start signal capturer
                        CURSOR_SHOW = 0;
                        CURSOR_ROW_INDEX = 2;
                        CURSOR_ROW_MIN_INDEX = 2;
                        CURSOR_ROW_MAX_INDEX = 3;
                        NVIC_EnableIRQ(GPCDEF_IRQn);
                        PAGE_INDEX = 1;
                    }
                    else
                    {
                        CURSOR_SHOW = signals_index > 0;
                        CURSOR_ROW_INDEX = 0;
                        CURSOR_ROW_MIN_INDEX = 0;
                        CURSOR_ROW_MAX_INDEX = signals_index < 4 ? signals_index - 1 : 3;
                        PAGE_INDEX = 2;
                    }
                    PAGE_FLAG = 1;
                }
                else if (PAGE_INDEX % 10 == 1)
                {
                    if (PAGE_INDEX == 11)
                    {
                        if (CURSOR_ROW_INDEX == 2)
                        {
                            // save signal
                            clear_cursor();
                            CURSOR_SHOW = 0;
                            CURSOR_ROW_INDEX = 2;
                            CURSOR_COLUMN_INDEX = 0;
                            CURSOR_ROW_MIN_INDEX = 2;
                            CURSOR_ROW_MAX_INDEX = 3;
                            CURSOR_COLUMN_MIN_INDEX = 0;
                            CURSOR_COLUMN_MAX_INDEX = 15;
                            PAGE_INDEX = 21;
                        }
                        else
                        {
                            // restart signal capturer
                            CURSOR_SHOW = 0;
                            CURSOR_ROW_INDEX = 2;
                            NVIC_EnableIRQ(GPCDEF_IRQn);
                            PAGE_INDEX = 1;
                        }
                        PAGE_FLAG = 1;
                    }
                    else if (PAGE_INDEX == 21)
                    {
                        if (CURSOR_ROW_INDEX == 2 && NAME_INDEX < 15)
                        {
                            print_buffer[NAME_INDEX] = ('A' + CURSOR_COLUMN_INDEX);
                            printC(NAME_INDEX * 8, 16, ('A' + CURSOR_COLUMN_INDEX));
                            NAME_INDEX++;
                            print_buffer[NAME_INDEX] = '\0';
                        }
                        else if (CURSOR_ROW_INDEX == 3)
                        {
                            if (CURSOR_COLUMN_INDEX < 10 && NAME_INDEX < 15)
                            {
                                print_buffer[NAME_INDEX] = ('Q' + +CURSOR_COLUMN_INDEX);
                                printC(NAME_INDEX * 8, 16, ('Q' + CURSOR_COLUMN_INDEX));
                                NAME_INDEX++;
                                print_buffer[NAME_INDEX] = '\0';
                            }
                            else if (CURSOR_COLUMN_INDEX < 11 && NAME_INDEX < 15)
                            {
                                print_buffer[NAME_INDEX] = '-';
                                printC(NAME_INDEX * 8, 16, '-');
                                NAME_INDEX++;
                                print_buffer[NAME_INDEX] = '\0';
                            }
                            else if (CURSOR_COLUMN_INDEX == 11 || CURSOR_COLUMN_INDEX == 12 || CURSOR_COLUMN_INDEX == 13)
                            {
                                NAME_INDEX--;
                                print_buffer[NAME_INDEX] = '\0';
                                printC(NAME_INDEX * 8, 16, ' ');
                            }
                            else if (CURSOR_COLUMN_INDEX == 14 || CURSOR_COLUMN_INDEX == 15)
                            {
                                // save name
                                if (NAME_INDEX == 0)
                                {
                                    sprintf(signals_names[signals_index], " %02X%02X%02X%02X", signals[signals_index][0], signals[signals_index][1], signals[signals_index][2], signals[signals_index][3]);
                                }
                                else
                                {
                                    sprintf(signals_names[signals_index], " %s", print_buffer);
                                    NAME_INDEX = 0;
                                }

                                signals_index++;
                                PAGE_INDEX = 31;
                                PAGE_FLAG = 1;
                            }
                        }
                    }
                }
                else if (PAGE_INDEX % 10 == 2)
                {
                    if (signals_index > 0)
                    {
                        // avoid timer stop sendnec
                        TIMER_Stop(TIMER0);
                        TIMER_Stop(TIMER2);
                        SendNEC(signals[CURSOR_ROW_INDEX + SCROLL_INDEX]);
                        // printf("%d %02X%02X%02X%02X\n",CURSOR_ROW_INDEX + SCROLL_INDEX, signals[signals_index][0], signals[CURSOR_ROW_INDEX + SCROLL_INDEX][1], signals[CURSOR_ROW_INDEX + SCROLL_INDEX][2], signals[CURSOR_ROW_INDEX + SCROLL_INDEX][3]);
                        CLK_SysTickDelay(300000);
                        TIMER_Start(TIMER0);
                        TIMER_Start(TIMER2);
                    }
                }
                break;
            }

            CAPTURED_KEY = 0;
        }

        // LCD
        if (PAGE_FLAG)
        {
            clear_LCD();
            if (PAGE_INDEX % 10 == 0)
            {
                print_Line(0, main_page[0]);
                print_Line(1, main_page[1]);
            }
            else if (PAGE_INDEX % 10 == 1)
            {
                if (PAGE_INDEX == 1)
                {
                    print_Line(0, capture_page[0]);
                }
                else if (PAGE_INDEX == 11)
                {
                    // printf("capture!\n");
                    sprintf(print_buffer, "%02X%02X%02X%02X", signals[signals_index][0], signals[signals_index][1], signals[signals_index][2], signals[signals_index][3]);
                    print_Line(0, capture_page[1]);
                    print_Line(1, print_buffer);
                    print_Line(2, capture_page[2]);
                    print_Line(3, capture_page[3]);
                }
                else if (PAGE_INDEX == 21)
                {
                    print_Line(0, name_page[0]);
                    print_Line(2, name_page[2]);
                    print_Line(3, name_page[3]);
                    display_name_block();
                }
                else if (PAGE_INDEX == 31)
                {
                    sprintf(print_buffer, "Signal Saved!");
                    print_Line(1, print_buffer);
                    PAGE_TIMER = 2;
                }
            }
            else if (PAGE_INDEX % 10 == 2)
            {
                if (signals_index)
                {
                    print_Line(0, signals_names[SCROLL_INDEX]);
                    print_Line(1, signals_names[SCROLL_INDEX + 1]);
                    print_Line(2, signals_names[SCROLL_INDEX + 2]);
                    print_Line(3, signals_names[SCROLL_INDEX + 3]);
                }
                else
                {
                    print_Line(0, record_page);
                }
            }
            PAGE_FLAG = 0;
        }
        display_cursor();
        display_name_block();
    }
}
