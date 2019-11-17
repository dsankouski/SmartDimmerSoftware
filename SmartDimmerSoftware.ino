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
uint16_t ch_1_ctl_remote_sw = 0;
uint16_t ch_2_ctl_remote_sw = 0;
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


#ifdef DEBUG_ENABLE
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
#endif

void io_poll() {
  uint8_t man_sw_control_ch1 = digitalRead(CH_1_MANUAL_SW);
  uint8_t man_sw_control_ch2 = digitalRead(CH_2_MANUAL_SW);
  uint8_t man_sw_control_ch1_old = (au16data[0] & MANUAL_SW_MASK) >> 2;
  uint8_t man_sw_control_ch2_old = (au16data[1] & MANUAL_SW_MASK) >> 2;

  uint8_t status_ch1;
  status_ch1 = digitalRead(CH_1_OT_LED);
  status_ch1 += digitalRead(CH_1_OC_LED) << 1;
  status_ch1 += man_sw_control_ch1 << 2;
  status_ch1 += digitalRead(CH_1_CTL) << 3;

  uint8_t status_ch2;
  status_ch2 = digitalRead(CH_2_OT_LED);
  status_ch2 += digitalRead(CH_2_OC_LED) << 1;
  status_ch2 += man_sw_control_ch2 << 2;
  status_ch2 += digitalRead(CH_2_CTL) << 3;

  au16data[6] = analogRead(TS_1);
  au16data[7] = analogRead(TS_2);
  au16data[8] = analogRead(CS_1);

  if (man_sw_control_ch1 != man_sw_control_ch1_old) {
    #ifdef DEBUG_ENABLE
    Serial.print("channel 1 set by manual switch to: ");
    Serial.println(man_sw_control_ch1, BIN);
    #endif

    digitalWrite(CH_1_CTL, man_sw_control_ch1);
  }
  if (man_sw_control_ch2 != man_sw_control_ch2_old) {
    #ifdef DEBUG_ENABLE
    Serial.print("channel 2 set by manual switch to: ");
    Serial.println(man_sw_control_ch2, BIN);
    #endif

    digitalWrite(CH_2_CTL, man_sw_control_ch2);
  }
  if (ch_1_ctl_remote_sw != au16data[4]) {
    #ifdef DEBUG_ENABLE
    Serial.print("channel 1 set by modbus to: ");
    Serial.println(au16data[4] != 0, BIN);
    #endif

    digitalWrite(CH_1_CTL, au16data[4] == 0);
  }
  if (ch_2_ctl_remote_sw != au16data[5]) {
    #ifdef DEBUG_ENABLE
    Serial.print("channel 2 set by modbus to: ");
    Serial.println(au16data[5], BIN);
    #endif

    digitalWrite(CH_2_CTL, au16data[5] == 0);
  }

  ch_1_ctl_remote_sw = au16data[4];
  ch_2_ctl_remote_sw = au16data[5];
  au16data[0] = status_ch1;
  au16data[1] = status_ch2;
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
  slave.begin( 19200 );
  #ifdef DEBUG_ENABLE
  Serial.begin( 19200 );
  #endif

  analogReference(INTERNAL);
  //Turn on adc
  analogRead(A0);
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

  slave.poll( au16data, 10);

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