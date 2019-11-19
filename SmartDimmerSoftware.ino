#include "modbus_slave/ModbusRtu.h"
#include "Arduino.h"

#define DEBUG_ENABLE
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
	Serial.print("Modbus data array:\n");
	Serial.println();
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
  uint8_t man_sw_control_ch1_old = (au16data[2] & MANUAL_SW_MASK) >> 2;
  uint8_t man_sw_control_ch2_old = (au16data[3] & MANUAL_SW_MASK) >> 2;

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

  au16data[5] = analogRead(CS_1);
  au16data[6] = analogRead(CS_2);
  au16data[7] = analogRead(TS_1);
  au16data[8] = analogRead(TS_2);

  if (man_sw_control_ch1 != man_sw_control_ch1_old) {
    digitalWrite(CH_1_CTL, man_sw_control_ch1);
  }
  if (man_sw_control_ch2 != man_sw_control_ch2_old) {
    digitalWrite(CH_2_CTL, man_sw_control_ch2);
  }
  if (ch_1_ctl_remote_sw != au16data[0]) {
    digitalWrite(CH_1_CTL, au16data[0] == 0);
  }
  if (ch_2_ctl_remote_sw != au16data[1]) {
    digitalWrite(CH_2_CTL, au16data[1] == 0);
  }

  ch_1_ctl_remote_sw = au16data[0];
  ch_2_ctl_remote_sw = au16data[1];
  au16data[2] = status_ch1;
  au16data[3] = status_ch2;
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
  //Turn on adc (needed to init internal analogReference)
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