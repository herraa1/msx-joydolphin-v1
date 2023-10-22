/*
 * msx-joydolphin, a Gamecube controller to MSX joystick adapter
 * Copyright (C) 2023 Albert Herranz
 *
 * This code is released under GPL V2.0
 *
 * Valid-License-Identifier: GPL-2.0-only
 * SPDX-URL: https://spdx.org/licenses/GPL-2.0-only.html
 *
 * Based on previous work by:
 * - Daniel Jose Viana "Danjovic" NSX-64 project, https://github.com/Danjovic/MSX/tree/master/NSX-64/firmware/nsx-64
 * - NicoHood Nintendo library, https://github.com/NicoHood
 *
 * Other references:
 * - http://www.int03.co.uk/crema/hardware/gamecube/gc-control.html
 * - https://www.msx.org/wiki/General_Purpose_port
 */

#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/interrupt.h>
#include "Nintendo.h"

/*
 * Debugging helpers
 */
/* #define DEBUG */

#ifdef DEBUG
  #define DEBUG_PRINTLN(x)  Serial.println (x)
#else
  #define DEBUG_PRINTLN(x)
#endif

/*
 * Timing constants
 */
#define _10ms   2499 /* OCR1 values for sleep period */

/* Nintendo Gamecube controller pad axis valid range */
#define GCN_AXIS_MIN 0
#define GCN_AXIS_MAX 255

/*
 * Arduino Nano pins for MSX signals.
 * Use PORTB for all MSX joystick arrow and buttons signals.
 * This allows to set them all at the same time.
 */
#define PORT_MSX_JOYSTICK PORTB
#define DDR_MSX_JOYSTICK  DDRB

/* MSX general purpose signals as mapped to Arduino Nano PORTB bits */
const int PORT_MSX_JOYSTICK_UP       = 0; /* PB0, MSX joystick pin1 */
const int PORT_MSX_JOYSTICK_DOWN     = 1; /* PB1, MSX joystick pin2 */
const int PORT_MSX_JOYSTICK_LEFT     = 2; /* PB2, MSX joystick pin3 */
const int PORT_MSX_JOYSTICK_RIGHT    = 3; /* PB3, MSX joystick pin4 */
const int PORT_MSX_JOYSTICK_TRIGGER1 = 4; /* PB4, MSX joystick pin6 */
const int PORT_MSX_JOYSTICK_TRIGGER2 = 5; /* PB5, MSX joystick pin7 */ /* LED_BUILTIN */

/* Arduino Nano pin for MSX joystick pin8 signal */
const int MSX_JOYSTICK_STROBE   = 2; /* PD2, MSX joystick pin8, not used for now */

/* Nintendo Gamecube joystick controller */
CGamecubeController GamecubeController1(7); /* PD7 for DAT I/O */

/* sleeping while a wavebird is connected doesn't work reliably */
volatile bool sleep_is_safe = false;

/*
 * Discrete thresholds for Nintengo Gamecube joystick analog pad axis:
 * - Values below the min threshold on the horizontal/vertical axis indicate
 *   left/down active, respectively.
 * - Values above the max threshold on the horizontal/vertical axis indicate
 *   right/up active, respectively.
 */
int axis_threshold_min, axis_threshold_max;

/* serial output width control */
uint8_t output_width_count = 0;

/*
 * Timer 1 CTC Interrupt handler
 */
ISR(TIMER1_COMPA_vect)
{
    set_timer1(_10ms); 
}

/*
 * Timer 1 timeout configuration
 */
inline void set_timer1(uint16_t overflow_ticks)
{
    cli(); /* disable interrupts */

    TCCR1A = 0; 
    TCCR1B = 0;
    TCNT1  = 0; /* initialize counter value to 0 */
    OCR1A = overflow_ticks; /* set compare register */

    TCCR1B |= (1 << WGM12);	                 /* CTC mode */
    TCCR1B |= (0<<CS12)|(1<<CS11)|(1<<CS10); /* prescaler = 64 */
    TIMSK1 |= (1 << OCIE1A);                 /* ISR Timer1/Compare */

    sei(); /* enable interrupts */
}

void print_hex8(uint8_t data)
{
    char tmp[2+1];
    byte nibble;

    nibble = (data >> 4) | 48;
    tmp[0] = (nibble > 57) ? nibble + (byte)39 : nibble;
    nibble = (data & 0x0f) | 48;
    tmp[1] = (nibble > 57) ? nibble + (byte)39 : nibble;
    tmp[2] = 0;

    Serial.print(tmp);
}

void print_hex16(uint16_t data)
{
    print_hex8((uint8_t)(data >> 8));
    print_hex8((uint8_t)(data & 0xff));
}

void print_rolling_sequence(void)
{
    static char rolling_chars[] = { '-', '\\', '|', '/' };
    static uint8_t rolling_index = 0;

    //Serial.write(8);
    Serial.print(rolling_chars[rolling_index++]);
    if (rolling_index >= sizeof(rolling_chars))
        rolling_index = 0;
}

inline bool is_wired_controller(uint16_t device)
{
    return (device & 0xff) == 0x09;
}

void setup()
{
    int axis_threshold;

    /* initialize joystick signals */
    update_msx_signals(0xff);

    /* use the serial port for debugging purposes */
    Serial.begin(9600);
    Serial.println("msx-joydolphin-v1");

    /*
     * Calculate Nintendo Gamecube controller analog pad thresholds.
     * Analog pad needs to be pushed from its origin in one direction at least
     * 1/6 of the complete run to get active
     */
    axis_threshold = (GCN_AXIS_MAX - GCN_AXIS_MIN) /3;
    axis_threshold_min = GCN_AXIS_MIN + axis_threshold;
    axis_threshold_max = GCN_AXIS_MAX - axis_threshold;

    GamecubeController1.begin(); /* initialize controller */
    delayMicroseconds(100);      /* allow some time for initialization */

    /* setup Timer 1 to wake up processor */
    set_timer1(_10ms);
}

void loop()
{
    loop_gamecube_controller();

    if (sleep_is_safe) {
        /* put CPU to sleep */
        set_sleep_mode(SLEEP_MODE_IDLE);
        sleep_enable();

        /*
        * Note: As Timer 0 is used by Arduino housekeeping it should be turned off
        * otherwise it will wake up CPU and hassle the interrupt interleave
        * mechanism used by this code
        */
        power_timer0_disable();

        sleep_mode();
        sleep_disable();
    }
}

/*
 * We use the TX led to indicate different conditions:
 * - continuous on  : pad or button pressed
 * - continuous off : pad or button not pressed
 * - fast blinking  : Nintengo Gamecube controller is not connected or
 *                    Wavebird not linked
 */
void update_led_status(uint8_t signals)
{
    /* turn on the TX led only if there is any button/pad active */
    if (signals != 0xff) {
        print_hex8(signals);
        output_width_count += 2;
        if (output_width_count > 79) {
            output_width_count = 0;
            Serial.println("");
        }
    }
}

inline void update_msx_signals(uint8_t signals)
{
    /* write all signal states at once to MSX side */
    PORT_MSX_JOYSTICK = signals;
    DDR_MSX_JOYSTICK = ~signals;
}

void loop_gamecube_controller()
{
    static bool was_connected = false;
    bool is_connected;
    uint16_t device;
    Gamecube_Report_t report;

    /*
     * MSX joystick signals:
     * - 1=inactive (pull up to +5V)
     * - 0=active (pull down to msx joystick ground, assumes pin8 is also pulled down to GND)
     */
    uint8_t msx_joystick_signals = 0xff;

    is_connected = GamecubeController1.connected();
    if (!is_connected) {
        /* write rolling sequence while not connected */
        print_rolling_sequence();
        /* wrap output at 80 columns ... */
        if (output_width_count++ > 79) {
            output_width_count = 0;
            Serial.println("");
        }
        was_connected = false;
    } else {
        if (!was_connected) {
            Serial.print("READY ");
            device = GamecubeController1.getStatus().device;
            print_hex16(device);
            /* sleep is safe only for wired controllers */
            sleep_is_safe = is_wired_controller(device);
            if (sleep_is_safe)
                Serial.print(" sleeping");
            Serial.println("!");
            output_width_count = 0;
            was_connected = true;
        }
    }

    if (GamecubeController1.read()) {
        report = GamecubeController1.getReport();

        /* Directional buttons (D-Pad) */
        if (report.dup) {
            msx_joystick_signals &= ~(1<<PORT_MSX_JOYSTICK_UP);
            DEBUG_PRINTLN("DUP");
        }
        if (report.ddown) {
            msx_joystick_signals &= ~(1<<PORT_MSX_JOYSTICK_DOWN);
            DEBUG_PRINTLN("DDOWN");
        }
        if (report.dleft) {
            msx_joystick_signals &= ~(1<<PORT_MSX_JOYSTICK_LEFT);
            DEBUG_PRINTLN("DLEFT");
        }
        if (report.dright) {
            msx_joystick_signals &= ~(1<<PORT_MSX_JOYSTICK_RIGHT);
            DEBUG_PRINTLN("DRIGHT");
        }

        /*
         * Digital Equivalent to directional buttons using the analog stick.
         * From center/origin: up and right is positive, down and left is negative.
         */

        /* before activating UP check that DOWN is not activated */
        if ((report.yAxis > axis_threshold_max) && (msx_joystick_signals & (1<<PORT_MSX_JOYSTICK_DOWN))) {
            msx_joystick_signals &= ~(1<<PORT_MSX_JOYSTICK_UP);
            DEBUG_PRINTLN("UP");
        }
        /* before activating DOWN check that UP is not activated */
        if ((report.yAxis < axis_threshold_min) && (msx_joystick_signals & (1<<PORT_MSX_JOYSTICK_UP))) {
            msx_joystick_signals &= ~(1<<PORT_MSX_JOYSTICK_DOWN);
            DEBUG_PRINTLN("DOWN");
        }

        /* before activating LEFT check that RIGHT is not activated */
        if ((report.xAxis < axis_threshold_min) && (msx_joystick_signals & (1<<PORT_MSX_JOYSTICK_RIGHT))) {
            msx_joystick_signals &= ~(1<<PORT_MSX_JOYSTICK_LEFT);
            DEBUG_PRINTLN("LEFT");
        }
        /* before activating RIGHT check that LEFT is not activated */
        if ((report.xAxis > axis_threshold_max) && (msx_joystick_signals & (1<<PORT_MSX_JOYSTICK_LEFT))) {
            msx_joystick_signals &= ~(1<<PORT_MSX_JOYSTICK_RIGHT);
            DEBUG_PRINTLN("RIGHT");
        }

        /* trigger buttons */
        if (report.a) {
            msx_joystick_signals &= ~(1<<PORT_MSX_JOYSTICK_TRIGGER1);
            DEBUG_PRINTLN("TRIG1");
        }
        if (report.b) {
            msx_joystick_signals &= ~(1<<PORT_MSX_JOYSTICK_TRIGGER2);
            DEBUG_PRINTLN("TRIG2");
        }

        /*
         * update led status state after a successfull Nintendo Gamecube
         * controller update
         */
        update_led_status(msx_joystick_signals);
    }

    /* unconditionally update MSX joystick signals */
    update_msx_signals(msx_joystick_signals);
}
