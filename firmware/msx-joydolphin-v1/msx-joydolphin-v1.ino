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

/*
 * NOTE
 *
 * The msx-joydolphin v1 adapter does not follow the joystick circuit diagram described in the
 * MSX Technical Data Book (Page 27, "1.4.6 Joysticks").
 *
 * This adapter ignores pin8 status and puts signals for joystick arrows/buttons in high impedance mode
 * (disconnected, pulled up to +5V by MSX pull-ups) except if the stick or trigger(s) are actuated in which
 * case the corresponding signals are pulled down to GND.
 *
 * This is functionally equivalent to a standard MSX joystick when pin8 is set to LOW which is required by software
 * to get the status of a joystick stick or trigger(s).
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
const int MSX_JOYSTICK_STROBE   = 2; /* PD2, MSX joystick pin8 */ /* not used */

/* Nintendo Gamecube joystick controller */
CGamecubeController GamecubeController1(7); /* PD7 for DAT I/O */

volatile uint8_t curr_msx_joystick_signals = 0xff;

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

void setup()
{
    int axis_threshold;

    /* initialize all joystick signals to high impedance */
    __update_msx_signals(0xff);

    /* use the serial port for debugging purposes */
    Serial.begin(9600);
    Serial.println("msx-joydolphin-v1 20240217_1");

    /*
     * Calculate Nintendo Gamecube controller analog pad thresholds.
     * Analog pad needs to be pushed from its origin in one direction at least
     * 1/6 of the complete run to get active
     */
    axis_threshold = (GCN_AXIS_MAX - GCN_AXIS_MIN) /3;
    axis_threshold_min = GCN_AXIS_MIN + axis_threshold;
    axis_threshold_max = GCN_AXIS_MAX - axis_threshold;

    GamecubeController1.begin(); /* initialize controller */
}

void loop()
{
    loop_gamecube_controller();
}

inline void __update_msx_signals(uint8_t signals)
{
    /*
     * signals variable bit definitions
     * bit5  bit4  bit3  bit2  bit1  bit0
     * TRG2  TRG1  RIGHT LEFT  DOWN  UP
     *
     * - bit set   : joystick arrow/button is not pressed
     *               configure associated GPIO as HIGH/INPUT (high impedance, logic "Z")
     * - bit unset : joystick arrow/button is pressed
     *               configure associated GPIO as LOW/OUTPUT (logic "0")
     *
     * In high impendance, the MSX receives a logic "1" signal because of the
     * MSX PSG circuit internal pull-ups.
     * In logic "0", the MSX receives a logic "0" as the GPIO drives the signal to GND.
     *
     * WARNING! IMPORTANT! WARNING! IMPORTANT!
     *
     * We _MUST NOT_ and we _DON'T_ set the Arduino GPIOs controlling the MSX signals as HIGH/OUTPUT
     * as that could fry the Arduino or PSG if this adapter is connected to the MSX _AND_ any
     * MSX software _NOT DESIGNED FOR DRIVING JOYSTICKS_ but other peripherals is currently
     * executing on the MSX in this specific circumstances:
     * - adapter connected to port #1, TRG1 not pressed
     *   - Bit0 of PSG Reg15 is clear (0), driving joystick #1 Pin6 to GND
     *   - (GameCube controller A button is not pressed)
     *   - Arduino D12/PB4 is incorrectly configured as HIGH/OUTPUT, driving joystick #1 Pin6 to +5V
     *   - we get bus contention on joystick #1 Pin6 bus
     * - adapter connected to port #1, TRG2 not pressed
     *   - Bit1 of PSG Reg15 is clear (0), driving joystick #1 Pin7 to GND
     *   - (GameCube controller B button is not pressed)
     *   - Arduino D13/PB5 is incorrectly configured as HIGH/OUTPUT, driving joystick #1 Pin7 to +5V
     *   - we get bus contention on joystick #1 Pin7 bus
     * - adapter connected to port #2, TRG1 not pressed
     *   - Bit2 of PSG Reg15 is clear (0), driving joystick #2 Pin6 to GND
     *   - (GameCube controller A button is not pressed)
     *   - Arduino D12/PB4 is incorrectly configured as HIGH/OUTPUT, driving joystick #2 Pin6 to +5V
     *   - we get bus contention on joystick #2 Pin6 bus
     * - adapter connected to port #2, TRG2 not pressed
     *   - Bit3 of PSG Reg15 is clear (0), driving joystick #2 Pin7 to GND
     *   - (GameCube controller B button is not pressed)
     *   - Arduino D13/PB5 is incorrectly configured as HIGH/OUTPUT, driving joystick #2 Pin7 to +5V
     *   - we get bus contention on joystick #2 Pin7 bus
     *
     * Thus we always _MUST_ and we _DO_ set GPIOs either as HIGH/INPUT ("Z") or LOW/OUTPUT ("0"),
     * as this firmware does.
     *
     * Note that the described failure scenario is impossible for MSX software written to interface
     * with joysticks because, in that case, bits 0-3 of PSG Reg15 are always set, not clear.
     * But we keep it safe anyway, just in case.
     */

    /*
     * NOTE! IMPORTANT! NOTE! IMPORTANT!
     *
     * This specific firmware version supports msx-joydolphin v1 boards which use D13/PB5 connected
     * to the TRG2 signal (pin7 of joystick connector).
     * D13/PB5 is (unfortunately) also connected within the Arduino Nano V3 board internally to a LED
     * (marked with "L" on the silkscreen) in series with a 1K resistor.
     * That Arduino internal 1K resistor form a voltage divider with a 10K pull-up resistor present in the
     * MSX circuitry (PSG uses open-collector with external pull-ups for those signals) which could
     * cause TRG2 to misbehave [1].
     * It will also cause the "L" LED to pull current from the MSX +5V via the MSX pull-up.
     *
     * To avoid problems with msx-joydolphin v1 Build1 adapters it is necessary to (choose one):
     * - Add a external pull up of 560 Ohm from D13/PB5 to +5V and keep the "L" LED and 1K resistor.
     * - Desolder either the "L" LED or the 1K resistor (in series with the "L" LED) from the Arduino
     *   Nano V3 board to convert D13/PB5 to a "normal" GPIO.
     *   Note that in recent Arduino Nano V3 boards it is not possible to desolder the 1K resistor as it
     *   is part of a resistor array connected too to the other three LEDs (power, rx, tx).
     * For Build2 and later adapters (when available) nothing needs to be done.
     *
     * [1] https://forum.arduino.cc/t/arduino-nano-pin-d13-usage/474254
     */

    /* write all signal states at once to MSX side */
    PORT_MSX_JOYSTICK = (signals);
    DDR_MSX_JOYSTICK = (~signals);
}

void loop_gamecube_controller()
{
    static bool was_connected = false;
    bool is_connected;
    uint16_t device;
    Gamecube_Report_t report;

    is_connected = GamecubeController1.connected();
    if (!is_connected) {
        /*
         * Make sure that no arrow/buttons stay pressed when
         * a gamecube controller is not connected.
         */
        curr_msx_joystick_signals = 0xff;
    }

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
            was_connected = true;
            device = GamecubeController1.getStatus().device;
            Serial.print("READY ");
            print_hex16(device);
            Serial.println("!");
            output_width_count = 0;
        }
    }

    /*
     * We need to read even if disconnected to make sure
     * that wavebird controllers connect properly (they use
     * a two stage connection process, as the dongle is always
     * connected but the controller only connects when activated).
     */
    if (GamecubeController1.read()) {
        uint8_t msx_joystick_signals = 0xff;

        report = GamecubeController1.getReport();

        /* Directional buttons (D-Pad) */
        if (report.dup)
            msx_joystick_signals &= ~(1<<PORT_MSX_JOYSTICK_UP);
        if (report.ddown)
            msx_joystick_signals &= ~(1<<PORT_MSX_JOYSTICK_DOWN);
        if (report.dleft)
            msx_joystick_signals &= ~(1<<PORT_MSX_JOYSTICK_LEFT);
        if (report.dright)
            msx_joystick_signals &= ~(1<<PORT_MSX_JOYSTICK_RIGHT);

        /*
        * Digital Equivalent to directional buttons using the analog stick.
        * From center/origin: up and right is positive, down and left is negative.
        */

        /* before activating UP check that DOWN is not activated */
        if ((report.yAxis > axis_threshold_max) && (msx_joystick_signals & (1<<PORT_MSX_JOYSTICK_DOWN)))
            msx_joystick_signals &= ~(1<<PORT_MSX_JOYSTICK_UP);

        /* before activating DOWN check that UP is not activated */
        if ((report.yAxis < axis_threshold_min) && (msx_joystick_signals & (1<<PORT_MSX_JOYSTICK_UP)))
            msx_joystick_signals &= ~(1<<PORT_MSX_JOYSTICK_DOWN);

        /* before activating LEFT check that RIGHT is not activated */
        if ((report.xAxis < axis_threshold_min) && (msx_joystick_signals & (1<<PORT_MSX_JOYSTICK_RIGHT)))
            msx_joystick_signals &= ~(1<<PORT_MSX_JOYSTICK_LEFT);

        /* before activating RIGHT check that LEFT is not activated */
        if ((report.xAxis > axis_threshold_max) && (msx_joystick_signals & (1<<PORT_MSX_JOYSTICK_LEFT)))
            msx_joystick_signals &= ~(1<<PORT_MSX_JOYSTICK_RIGHT);

        /* trigger buttons */
        if (report.a)
            msx_joystick_signals &= ~(1<<PORT_MSX_JOYSTICK_TRIGGER1);
        if (report.b)
            msx_joystick_signals &= ~(1<<PORT_MSX_JOYSTICK_TRIGGER2);

        curr_msx_joystick_signals = msx_joystick_signals;
    }
    __update_msx_signals(curr_msx_joystick_signals);
}
