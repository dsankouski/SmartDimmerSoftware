#include "modbus_slave/ModbusRtu.h"
#include "Arduino.h"

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
#define TS_2 A8

#define MANUAL_SW_MASK 4

Modbus slave(BOARD_ID, 0, 0);
uint8_t ch_1_ctl_remote_sw;
uint8_t ch_2_ctl_remote_sw;
int16_t power_setting_ch1;
int16_t power_setting_ch2;
uint8_t debug_time_counter = 0;

/*
 * Modbus data table
 *
 * +------------------------------------------------------------------------------
 * | Register   Bit  Name                   Type      Modbus addr  Access
 * +------------------------------------------------------------------------------
 * |au16data[0]                             Discrete
 * |            0     CH_1_OT_LED                      0           Read
 * |            1     CH_1_OC_LED                      1           Read
 * |            2     CH_1_MANUAL_SW                   2           Read
 * |            3     CH_1_OUTPUT                      2           Read
 * |au16data[1]
 * |            8     CH_2_OT_LED                      8           Read
 * |            9     CH_2_OC_LED                      9           Read
 * |            10    CH_2_MANUAL_SW                   10          Read
 * |            11    CH_2_OUTPUT                      10          Read
 * |au16data[2]                             Discrete
 * |            32    power_reading_ch1
 * |au16data[3]                             Discrete
 * |            48    power_reading_ch2
 * |au16data[4]
 * |            64    CH_1_CTL                         17          Read/Write
 * |au16data[5]
 * |            80    CH_2_CTL                         25          Read/Write
 * |au16data[6]
 * |            96    soft_start_ch1        Holding    16          Read/Write
 * |au16data[7]
 * |            112   soft_start_ch2        Holding    16          Read/Write
 * |au16data[8]
 * |            128   power_setting_ch1     Holding    32          Read/Write
 * |au16data[9]
 * |            144   power_setting_ch2     Holding    32          Read/Write
 * +-----------------------------------------------------------------------------
 */
uint16_t au16data[10];

void setup() {
  io_setup();
  slave.begin( 19200 );
  Serial.begin( 19200 );

  analogReference(INTERNAL);
  //Turn on adc
  analogRead(A0);
}

void io_setup() {
  digitalWrite(CH_1_CTL, HIGH);
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
}

void loop() {
  debug_time_counter++;
  if (debug_time_counter == 200) {
    Serial.write(27);       // ESC command
    Serial.print("[2J");    // clear screen command
    Serial.write(27);
    Serial.print("[H");     // cursor to home command
  }
  slave.poll( au16data, 10);
  if (debug_time_counter == 200) {
    dump_modbus_data();
  }
  io_poll();

  if (debug_time_counter == 200) {
    debug_time_counter = 0;
  }
}

void io_poll() {
  uint8_t ch_1_ctl = 0;
  uint8_t ch_2_ctl = 0;

  uint8_t status_ch1;
  status_ch1 = digitalRead(CH_1_OT_LED);
  status_ch1 += digitalRead(CH_1_OC_LED) << 1;
  status_ch1 += digitalRead(CH_1_MANUAL_SW) << 2;
  status_ch1 += digitalRead(CH_1_CTL) << 3;

  uint8_t status_ch2;
  status_ch2 = digitalRead(CH_2_OT_LED);
  status_ch2 += digitalRead(CH_2_OC_LED) << 8 << 1;
  status_ch2 += digitalRead(CH_2_MANUAL_SW) << 8 << 2;
  status_ch2 += digitalRead(CH_2_CTL) << 8 << 3;

  ch_1_ctl = should_activate_channel(status_ch1 & MANUAL_SW_MASK, au16data[0] & MANUAL_SW_MASK, au16data[4], ch_1_ctl_remote_sw);
  ch_2_ctl = ((status_ch2 & MANUAL_SW_MASK) > (au16data[1] & MANUAL_SW_MASK)) | (au16data[5] > ch_2_ctl_remote_sw);

  au16data[0] = status_ch1;
  au16data[1] = status_ch2;
  ch_1_ctl_remote_sw = au16data[4];
  ch_2_ctl_remote_sw = au16data[5];

  digitalWrite(CH_1_CTL, ch_1_ctl);
  digitalWrite(CH_2_CTL, ch_2_ctl);
}

uint8_t should_activate_channel(uint8_t man_sw_old, uint8_t man_sw_new, uint16_t prog_sw_old, uint16_t prog_sw_new) {
	if (debug_time_counter == 200) {
        Serial.print("man_sw_old: ");
        Serial.println(man_sw_old, BIN);
        Serial.print("man_sw_new: ");
        Serial.println(man_sw_new, BIN);
        Serial.print("prog_sw_old: ");
        Serial.println(prog_sw_old, BIN);
        Serial.print("prog_sw_new: ");
        Serial.println(prog_sw_new, BIN);
    }
	return (man_sw_new != man_sw_old) & (man_sw_new) | prog_sw_new > prog_sw_old;
}

void dump_modbus_data() {
	Serial.print("Modbus data array:\n");
	for (uint8_t i = 0; i < sizeof(au16data) / 2; i++) {
		Serial.print("reg ");
		Serial.print(i);
		Serial.print(": ");
		Serial.println(au16data[i], BIN);
	}

	Serial.print("###################\n");
}