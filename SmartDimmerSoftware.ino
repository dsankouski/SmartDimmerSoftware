#include "modbus_slave/ModbusRtu.h"
#include "Arduino.h"
#include "analogComp.h"
#include <util/atomic.h>

#define BOARD_ID 1

#define CH_1_OT_LED 13
#define CH_1_OC_LED 6
// Channel 1 switch control output
#define CH_1_CTL 5
#define CH_1_MANUAL_SW 10
#define CH_2_OT_LED 12
#define CH_2_OC_LED 8
// Channel 2 switch control output
#define CH_2_CTL 9
#define CH_2_MANUAL_SW 11
#define AMP_SENSE_POS A0
#define AMP_SENSE_NEG A4
#define VCC_HALF_SENSE A5
#define CS_1 A2
#define TS_1 A3
#define CS_2 A1
#define TS_2 A6

#define MANUAL_SW_MASK 4

Modbus slave(BOARD_ID, 0, 0);
uint16_t ch_1_ctl_remote_sw = 0;
uint16_t ch_2_ctl_remote_sw = 0;

uint16_t ICR_previous = 0;
uint16_t period = 160;
uint16_t error = 0;

#define DEBUG_ENABLE
#ifdef DEBUG_ENABLE
uint8_t debug_time_counter = 0;
#endif

/*
 * Modbus data table
 *
 * +------------------------------------------------------------------------------
 * | Register   Bit  Name                   Type      Modbus addr  Access
 * +------------------------------------------------------------------------------
 * |au16data[0]
 * |            0    CH_1_CTL                                     Read/Write
 * |au16data[1]
 * |            16    CH_2_CTL                                     Read/Write
 * |au16data[2]                             Discrete
 * |            0     CH_1_OT_LED                                  Read
 * |            1     CH_1_OC_LED                                  Read
 * |            2     CH_1_MANUAL_SW                               Read
 * |            3     CH_1_OUTPUT                                  Read
 * |au16data[3]
 * |            #     CH_2_OT_LED                                  Read
 * |            #     CH_2_OC_LED                                  Read
 * |            #    CH_2_MANUAL_SW                                Read
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

/**
  * This timer is used for determining zero crossing period
  * todo: switch control pins to control it with 1 timer.
  */
void timer_1_setup() {
	/* prescaler 1/1024 */
    /* CSn2:0 = 101 */
    TCCR1B |= 1 << CS12;
    TCCR1B |= 1 << CS10;
    TCCR1B &= ~(1 << CS11);

    /* Fast PWM , TOP = OCIE1A*/
	TCCR1A |= 1 << WGM10;
	TCCR1A |= 1 << WGM11;
	TCCR1B |= 1 << WGM12;
	TCCR1B |= 1 << WGM13;

	ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
	    OCR1AH = 0xFF;
        OCR1AL = 0xFF;
	}

	/* Interrupt on Overflow (TOP), and on input capture*/
// 	TIMSK1 |= 1 << TOIE1;
	TIMSK1 |= 1 << ICIE1;
	/* Need to enable ACIC in analog comparator to trigger timer capture */
	ACSR |= 1 << ACIC;
}

/* timer capture event */
ISR(TIMER1_CAPT_vect){
ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
	uint16_t new_period = ICR1 - ICR_previous;
    ICR_previous = ICR1;

    if (new_period > 250){
        period = new_period;
        /* todo: include error of timer 3 */
	    ICR3 = period / 2;
    }
	}
}
/* analogComparator interrupt */
void zero_crossing_handler() {
    TCNT3 = 0;
}
// /* timer overflow */
// ISR(TIMER1_OVF_vect){
// 	if (input_capture_flag > 0) {
//
// 	}
// }

/* *
 * This timer controls AC phase on channel 1
 */
void timer_3_setup() {
    	/* prescaler 1/1024 */
    	/* CSn2:0 = 101 */
    	TCCR3B |= 1 << CS12;
        TCCR3B |= 1 << CS10;
        TCCR3B &= ~(1 << CS11);

    	/* Fast PWM , TOP = ICR*/
    	TCCR3A &= ~(1 << WGM30);
    	TCCR3A |= 1 << WGM31;
    	TCCR3B |= 1 << WGM32;
    	TCCR3B |= 1 << WGM33;


	/* Clear OCnA/OCnB/OCnC on compare match, set OCnA/OCnB/OCnC at TOP */
    	TCCR3A |= 1 << COM3A1;
    	/* Interrupt on Overflow (TOP), OCRnA*/
//     	TIMSK3 |= 1 << TOIE3
//     	TIMSK3 |= 1 << OCIE3A

// ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
// ICR3 = 0x3FFF;
// }
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
	    OCR3AH = 0x00;
        OCR3AL = 0x50;
	}
}

void io_poll() {
  uint8_t man_sw_control_ch1 = digitalRead(CH_1_MANUAL_SW);
  uint8_t man_sw_control_ch2 = digitalRead(CH_2_MANUAL_SW);
  uint8_t man_sw_control_ch1_old = (au16data[2] & MANUAL_SW_MASK) >> 2;
  uint8_t man_sw_control_ch2_old = (au16data[3] & MANUAL_SW_MASK) >> 2;

  uint8_t status_ch1;
  status_ch1 = digitalRead(CH_1_OT_LED);
  status_ch1 += digitalRead(CH_1_OC_LED) << 1;
  status_ch1 += man_sw_control_ch1 << 2;
  /* reading this bit causes COM3A1 bit to 0  int timer 3*/
//   status_ch1 += digitalRead(CH_1_CTL) << 3;

  uint8_t status_ch2;
  status_ch2 = digitalRead(CH_2_OT_LED);
  status_ch2 += digitalRead(CH_2_OC_LED) << 1;
  status_ch2 += man_sw_control_ch2 << 2;
//   status_ch2 += digitalRead(CH_2_CTL) << 3;

  au16data[5] = analogRead(CS_1);
  au16data[6] = analogRead(CS_2);
  au16data[7] = analogRead(TS_1);
  au16data[8] = analogRead(TS_2);
  au16data[9] = period;
  au16data[10] = ICR3;
  au16data[11] = OCR3A;
  au16data[12] = TCCR3A;
  au16data[13] = TCCR3B;
  au16data[14] = TIMSK3;

//   if (man_sw_control_ch1 != man_sw_control_ch1_old) {
//     digitalWrite(CH_1_CTL, man_sw_control_ch1);
//   }
//   if (man_sw_control_ch2 != man_sw_control_ch2_old) {
//     digitalWrite(CH_2_CTL, man_sw_control_ch2);
//   }
//   if (ch_1_ctl_remote_sw != au16data[0]) {
//     digitalWrite(CH_1_CTL, au16data[0] == 0);
//   }
//   if (ch_2_ctl_remote_sw != au16data[1]) {
//     digitalWrite(CH_2_CTL, au16data[1] == 0);
//   }

  ch_1_ctl_remote_sw = au16data[0];
  ch_2_ctl_remote_sw = au16data[1];
  au16data[2] = status_ch1;
  au16data[3] = status_ch2;
}

void io_setup() {
//   digitalWrite(CH_1_CTL, HIGH);
  digitalWrite(CH_2_CTL, HIGH);
  digitalWrite(CH_1_OT_LED, LOW);
  digitalWrite(CH_2_OT_LED, LOW);
  digitalWrite(CH_1_OC_LED, HIGH);
  digitalWrite(CH_2_OC_LED, HIGH);

  /* All outputs are active low */
  pinMode(CH_1_OT_LED, OUTPUT);
  pinMode(CH_2_OT_LED, OUTPUT);
  pinMode(CH_1_OC_LED, OUTPUT);
  pinMode(CH_2_OC_LED, OUTPUT);
  pinMode(CH_1_CTL, OUTPUT);
  pinMode(CH_2_CTL, OUTPUT);

  pinMode(CH_1_MANUAL_SW, INPUT_PULLUP);
  pinMode(CH_2_MANUAL_SW, INPUT_PULLUP);
  pinMode(7, INPUT);
  pinMode(TS_1, INPUT);
  pinMode(TS_2, INPUT);
  pinMode(CS_1, INPUT);
  pinMode(CS_2, INPUT);
  pinMode(AMP_SENSE_POS, INPUT);
  pinMode(AMP_SENSE_NEG, INPUT);
  pinMode(VCC_HALF_SENSE, INPUT);
}

void setup() {
  io_setup();
  slave.begin( 115200 );
  #ifdef DEBUG_ENABLE
  Serial.begin( 115200 );
  #endif

  analogReference(INTERNAL);
  //Turn on adc (needed to init internal analogReference)
  analogRead(A0);

  // INTERNAL_REFERENCE should be replaced with AIN+
  // AIN+ -> PE6 pin
  analogComparator.setOn(AIN0, INTERNAL_REFERENCE);

  //     Turn on ADC again after comparator setup
  ADCSRA |= 1 << 7;
  timer_1_setup();
  timer_3_setup();

  analogComparator.enableInterrupt(zero_crossing_handler, CHANGE);
  delay(40);
  analogComparator.disableInterrupt();
}

void loop() {
#ifdef DEBUG_ENABLE
  debug_time_counter++;
  if (debug_time_counter == 200) {
    Serial.write(27);       // ESC command
    Serial.print("[2J");    // clear screen command
    Serial.write(27);
    Serial.print("[H");     // cursor to home command
  }
  #endif

  slave.poll( au16data, 15);

  #ifdef DEBUG_ENABLE
  if (debug_time_counter == 200) {
    dump_modbus_data();
  }
  #endif
  io_poll();

  #ifdef DEBUG_ENABLE
  if (debug_time_counter == 200) {
    debug_time_counter = 0;
  }
  #endif
}
