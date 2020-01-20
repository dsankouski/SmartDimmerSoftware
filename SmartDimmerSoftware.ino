#include "modbus_slave/ModbusRtu.h"
#include "Arduino.h"
#include "analogComp.h"
#include <util/atomic.h>
#include "timer1.h"
#include "timer3.h"

#define BOARD_ID 1

#define CH_1_OT_LED 13
#define CH_1_OC_LED 6
// Channel 1 switch control output
#define CH_1_OUTPUT 5
#define CH_1_MANUAL_INPUT 10
#define CH_2_OT_LED 12
#define CH_2_OC_LED 8
// Channel 2 switch control output
#define CH_2_OUTPUT 9
#define CH_2_MANUAL_INPUT 11
#define VAC_SENSE_INPUT 7
#define VAC_SENSE_POS A0
#define VAC_SENSE_NEG A4
#define VCC_HALF_SENSE A5
#define CS_1 A2
#define TS_1 A3
#define CS_2 A1
#define TS_2 A6
#define trigger_pulse_length 30

#define MANUAL_SW_MASK 4
#define MODBUS_SERIAL_BAUD_RATE 115200
#define COMPARATOR_OFFSET_ERROR 2
#define ZERO_CROSSING_LATENCY 9

Modbus slave(BOARD_ID, 0, 0);
uint8_t is_ch1_on = 0;
uint8_t is_ch2_on = 0;
uint16_t ch_1_ctl_remote_sw = 0;
uint16_t ch_2_ctl_remote_sw = 0;


uint16_t high_length = 0;
uint16_t low_length = 0;
uint16_t length_previous = 0;
// uint16_t period = 312;
uint16_t half_wave_duration = period / 2;
volatile uint16_t ch_1_trigger_offset = 0xFFFF;
volatile uint16_t ch_2_trigger_offset = 0xFFFF;

#define DEBUG_ENABLE
#ifdef DEBUG_ENABLE
#define DEBUG_SERIAL_BAUD_RATE 256000
#define DEBUG_COUNTER_LIMIT 200
uint16_t debug_time_counter = 0;
#endif

/*
 * Modbus data table
 *
 * +------------------------------------------------------------------------------
 * | Register   Bit  Name                   Type      Modbus addr  Access
 * +------------------------------------------------------------------------------
 * |au16data[0]
 * |            0    CH_1_OUTPUT                                     Read/Write
 * |au16data[1]
 * |            16    CH_2_OUTPUT                                     Read/Write
 * |au16data[2]                             Discrete
 * |            0     CH_1_OT_LED                                  Read
 * |            1     CH_1_OC_LED                                  Read
 * |            2     CH_1_MANUAL_INPUT                               Read
 * |            3     CH_1_OUTPUT                                  Read
 * |au16data[3]
 * |            #     CH_2_OT_LED                                  Read
 * |            #     CH_2_OC_LED                                  Read
 * |            #    CH_2_MANUAL_INPUT                                Read
 * |            #    CH_2_OUTPUT                                   Read
 * |au16data[4]                             Input
 * |            64    voltage_reading
 * |au16data[5]                             Input
 * |            ##    current_reading_ch1
 * |au16data[6]                             Input
 * |            ##    current_reading_ch2
 * |au16data[7]                             Input
 * |            96    tempreture_reading_ch1
 * |au16data[8]                             Input
 * |            ##    tempreture_reading_ch2
 * |au16data[9]                             Input
 * |            ##    power_reading_ch1
 * |au16data[10]                             Input
 * |            128    power_reading_ch2
 * |au16data[11]
 * |            ##    soft_start_ch1        Holding                Read/Write
 * |au16data[12]
 * |            ##   soft_start_ch2        Holding                 Read/Write
 * |au16data[13]
 * |            160   power_setting_ch1     Holding                Read/Write
 * |au16data[14]
 * |            176   power_setting_ch2     Holding                Read/Write
 * +-----------------------------------------------------------------------------
 */
uint16_t au16data[15];


#ifdef DEBUG_ENABLE
void dump_modbus_data() {
	Serial.print("Modbus data array:");
	Serial.println();
	for (uint8_t i = 0; i < sizeof(au16data) / 2; i++) {
		Serial.print("reg ");
		Serial.print(i);
		Serial.print(": ");
		Serial.println(au16data[i], DEC);
	}

	Serial.println("###################");
}
#endif

/* analogComparator interrupt */
void zero_crossing_handler() {
ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
	uint16_t tcnt3 = TCNT3;
	uint16_t tcnt1 = TCNT1;
    OCR3A = tcnt3 + ch_1_trigger_offset;
    OCR1A = tcnt1 + ch_2_trigger_offset;
    }

	if (TIFR1 & COMPARE_MATCH_1A_MASK > 0) {
        TIFR1 |= COMPARE_MATCH_1A_MASK;
	} else {
		TCCR1C |= 1 << FOC1A;
	}
	if (TIFR3 & COMPARE_MATCH_3A_MASK > 0) {
        TIFR3 |= COMPARE_MATCH_3A_MASK;
	} else {
		TCCR3C |= 1 << FOC3A;
	}

    TCCR3A &= TOGGLE_ON_COMPARE_3A_AND_MASK;
    TCCR1A &= TOGGLE_ON_COMPARE_1A_AND_MASK;
    TIMSK3 |= OUTPUT_COMPARE_3A_INT_ENABLE_OR_MASK;
    TIMSK1 |= OUTPUT_COMPARE_1A_INT_ENABLE_OR_MASK;

    if (ACSR & ACO_MASK) {
                high_length = tcnt3 - length_previous;
                length_previous = tcnt3;
           } else {
                low_length = tcnt3 - length_previous;
                length_previous = tcnt3;
           }
}

void io_poll() {
  uint8_t man_sw_control_ch1 = digitalRead(CH_1_MANUAL_INPUT);
  uint8_t man_sw_control_ch2 = digitalRead(CH_2_MANUAL_INPUT);
  uint8_t man_sw_control_ch1_old = (au16data[2] & MANUAL_SW_MASK) >> 2;
  uint8_t man_sw_control_ch2_old = (au16data[3] & MANUAL_SW_MASK) >> 2;

  uint8_t status_ch1;
  status_ch1 = digitalRead(CH_1_OT_LED);
  status_ch1 += digitalRead(CH_1_OC_LED) << 1;
  status_ch1 += man_sw_control_ch1 << 2;

  uint8_t status_ch2;
  status_ch2 = digitalRead(CH_2_OT_LED);
  status_ch2 += digitalRead(CH_2_OC_LED) << 1;
  status_ch2 += man_sw_control_ch2 << 2;

  au16data[5] = ch_1_trigger_offset;
  au16data[6] = ch_2_trigger_offset;
  au16data[7] = high_length;
  au16data[8] = low_length;
  au16data[9] = period;
  half_wave_duration = period / 2;

  ch_1_trigger_offset = half_wave_duration - (half_wave_duration * au16data[13]) / 100;
  ch_2_trigger_offset = half_wave_duration - (half_wave_duration * au16data[14]) / 100;
  if (ch_1_trigger_offset > ZERO_CROSSING_LATENCY) {
    ch_1_trigger_offset -= ZERO_CROSSING_LATENCY;
  } else {
    ch_1_trigger_offset = 0;
  }
  if (ch_2_trigger_offset > ZERO_CROSSING_LATENCY) {
    ch_2_trigger_offset -= ZERO_CROSSING_LATENCY;
  } else {
    ch_2_trigger_offset = 0;
  }

  if (man_sw_control_ch1 != man_sw_control_ch1_old) {
    is_ch1_on = man_sw_control_ch1;
  }
  if (man_sw_control_ch2 != man_sw_control_ch2_old) {
    is_ch2_on = man_sw_control_ch2;
  }
  if (ch_1_ctl_remote_sw != au16data[0]) {
    is_ch1_on = au16data[0];
  }
  if (ch_2_ctl_remote_sw != au16data[1]) {
    is_ch2_on = au16data[1];
  }

  ch_1_ctl_remote_sw = au16data[0];
  ch_2_ctl_remote_sw = au16data[1];
  au16data[2] = status_ch1;
  au16data[3] = status_ch2;
}

void io_setup() {
  digitalWrite(CH_1_OUTPUT, HIGH);
  digitalWrite(CH_2_OUTPUT, HIGH);
  digitalWrite(CH_1_OT_LED, LOW);
  digitalWrite(CH_2_OT_LED, LOW);
  digitalWrite(CH_1_OC_LED, HIGH);
  digitalWrite(CH_2_OC_LED, HIGH);

  /* All outputs are active low */
  pinMode(CH_1_OT_LED, OUTPUT);
  pinMode(CH_2_OT_LED, OUTPUT);
  pinMode(CH_1_OC_LED, OUTPUT);
  pinMode(CH_2_OC_LED, OUTPUT);
  pinMode(CH_1_OUTPUT, OUTPUT);
  pinMode(CH_2_OUTPUT, OUTPUT);

  pinMode(CH_1_MANUAL_INPUT, INPUT_PULLUP);
  pinMode(CH_2_MANUAL_INPUT, INPUT_PULLUP);
  pinMode(7, INPUT);
  pinMode(TS_1, INPUT);
  pinMode(TS_2, INPUT);
  pinMode(CS_1, INPUT);
  pinMode(CS_2, INPUT);
  pinMode(VAC_SENSE_POS, INPUT);
  pinMode(VAC_SENSE_NEG, INPUT);
  pinMode(VCC_HALF_SENSE, INPUT);
}

void setup() {
  io_setup();

  analogReference(INTERNAL);
  //Turn on adc (needed to init internal analogReference)
  analogRead(A0);

  // INTERNAL_REFERENCE should be replaced with AIN+
  // AIN+ -> PE6 pin
  analogComparator.setOn(AIN0, INTERNAL_REFERENCE);

  //     Turn on ADC again after comparator setup
  ADCSRA |= 1 << ADEN;

  /* timers setup */
  timer_1_setup();
  timer_3_setup();

  digitalWrite(CH_1_OUTPUT, HIGH);
  digitalWrite(CH_2_OUTPUT, HIGH);

  analogComparator.enableInterrupt(zero_crossing_handler, CHANGE);
//   /* Need to enable ACIC in analog comparator to trigger timer capture */
  ACSR |= 1 << ACIC;

  slave.begin(MODBUS_SERIAL_BAUD_RATE);
  #ifdef DEBUG_ENABLE
  Serial.begin(DEBUG_SERIAL_BAUD_RATE);
  #endif
}

void loop() {
#ifdef DEBUG_ENABLE
  debug_time_counter++;
  if (debug_time_counter == DEBUG_COUNTER_LIMIT) {
    Serial.write(27);       // ESC command
    Serial.print("[2J");    // clear screen command
    Serial.write(27);
    Serial.print("[H");     // cursor to home command
  }
  #endif

  slave.poll( au16data, 15);

  #ifdef DEBUG_ENABLE
  if (debug_time_counter == DEBUG_COUNTER_LIMIT) {
    dump_modbus_data();
  }
  #endif
  io_poll();

  #ifdef DEBUG_ENABLE
  if (debug_time_counter == DEBUG_COUNTER_LIMIT) {
    debug_time_counter = 0;
  }
  #endif
}
