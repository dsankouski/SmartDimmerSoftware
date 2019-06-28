#include "modbus_slave/ModbusRtu.h"
#include "Arduino.h"

#define BOARD_ID 1

#define CH_1_OT_LED 13
#define CH_1_OC_LED 6
// Channel 1 thyristor control output
#define CH_1_CTL 5
#define CH_1_MANUAL_SW 10
#define CH_2_OT_LED 12
#define CH_2_OC_LED 8
// Channel 2 thyristor control output
#define CH_2_CTL 9
#define CH_2_MANUAL_SW 11
#define AMP_SENSE_POS A0
#define AMP_SENSE_NEG A4
#define VCC_HALF_SENSE A5
#define CS_1 A2
#define TS_1 A3
#define CS_2 A1
#define TS_2 A8

Modbus slave(BOARD_ID, 0, 0);
boolean soft_start_ch1;
boolean soft_start_ch2;
boolean is_ch1_manual_on = false;
boolean is_ch2_manual_on = false;
boolean is_ch1_auto_on = false;
boolean is_ch2_auto_on = false;
int8_t power_setting_ch1;
int8_t power_setting_ch2;

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
 * |            8     CH_2_OT_LED                      8           Read
 * |            9     CH_2_OC_LED                      9           Read
 * |            10    CH_2_MANUAL_SW                   10          Read
 * +-----------------------------------------------------------------------------
 * |au16data[1] 16    soft_start_ch1        Coil       16          Read/Write
 * |            17    CH_1_CTL              Coil       17          Read/Write
 * |            24    soft_start_ch2        Coil       24          Read/Write
 * |            25    CH_2_CTL              Coil       25          Read/Write
 * +-----------------------------------------------------------------------------
 * |au16data[2]       power_setting_ch1     Holding    32          Read/Write
 * |au16data[3]       power_setting_ch1     Holding    48          Read/Write
 */
uint16_t au16data[3];

void setup() {
  io_setup();
  slave.begin( 19200 );

  analogReference(INTERNAL);
  //Turn on adc
  analogRead(A0);
}

void io_setup() {
  digitalWrite(CH_1_CTL, HIGH); 
  digitalWrite(CH_2_CTL, HIGH);
  digitalWrite(CH_2_OT_LED, LOW);
  digitalWrite(CH_1_OT_LED, LOW);
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
  slave.poll( au16data, 3);
  io_poll();
}

void io_poll() {
  is_ch1_manual_on = digitalRead(CH_1_MANUAL_SW);
  is_ch2_manual_on = digitalRead(CH_2_MANUAL_SW);

  uint8_t status_ch1;
  status_ch1 = digitalRead(CH_1_OT_LED);
  status_ch1 += digitalRead(CH_1_OC_LED) << 1;
  status_ch1 += is_ch1_manual_on << 2;

  uint8_t status_ch2;
  status_ch2 = digitalRead(CH_2_OT_LED);
  status_ch2 += digitalRead(CH_2_OC_LED) << 1;
  status_ch2 += is_ch2_manual_on << 2;

  au16data[0] = status_ch1 + (status_ch2 << 8);

  is_ch1_auto_on = (au16data[1] & ( 1 << 1 )) >> 1;
  is_ch2_auto_on = (au16data[1] & ( 1 << 9 )) >> 9;

  digitalWrite(CH_1_CTL, is_ch1_manual_on & is_ch1_auto_on);
  digitalWrite(CH_2_CTL, is_ch2_manual_on & is_ch2_auto_on);
}