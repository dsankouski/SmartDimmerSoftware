#include <analogComp.h>
#include "modbus_slave/ModbusRtu.h"
#include "Arduino.h"
#include <PinChangeInterrupt.h>

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
#define stlPin  13  // номер выхода индикатора работы
                    // расположен на плате Arduino

//Задаём ведомому адрес, последовательный порт, выход управления TX
//uint8_t BOARD_ID = 1;
Modbus slave(BOARD_ID, 0, 0); 
boolean phase;
boolean led;
boolean soft_start_ch1;
boolean soft_start_ch2;
int8_t power_setting_ch1;
int8_t power_setting_ch2;
int8_t state = 0;
int8_t zero_cross_count = 0;
unsigned long tempus;
volatile boolean is_ch2_manual_on = false;

// массив данных modbus
/*
 * +------------------------------------------------------------------------------
 * | Register   Bit  Name                   Type      Modbus addr  Access
 * +------------------------------------------------------------------------------
 * |au16data[0]                             Discrete               Read
 * |            0     CH_1_OT_LED                      0           Read
 * |            1     CH_1_OC_LED                       1          Read
 * |            2     CH_1_MANUAL_SW                     2         Read
 * |            3     CH_2_OT_LED                         3        Read
 * |            4     CH_2_OC_LED                          4       Read
 * |            5     CH_2_MANUAL_SW                        5      Read
 * +-----------------------------------------------------------------------------
 * |au16data[1] 0     soft_start_ch1        Coil            8
 * |            1     soft_start_ch2        Coil            9
 * |            2     CH_1_CTL        Coil            10
 * |            3     CH_2_CTL        Coil            11
 * +-----------------------------------------------------------------------------
 * |au16data[2]       power_setting_ch1     Holding         6
 * |au16data[3]       power_setting_ch1     Holding         7
 */
volatile uint16_t au16data[3];

void setup() {
//  slave.setID(BOARD_ID);
  // настраиваем входы и выходы
  io_setup();
  // настраиваем последовательный порт ведомого
  slave.begin( 19200 ); 
  // зажигаем светодиод на 100 мс
  tempus = millis() + 100; 
  digitalWrite(stlPin, HIGH );

  // INTERNAL_REFERENCE should be replaced with AIN+
  analogComparator.setOn(INTERNAL_REFERENCE, INTERNAL_REFERENCE);
  analogComparator.enableInterrupt(zero_crossing_handler, CHANGE);
}

void io_setup() {
  digitalWrite(stlPin, HIGH );
  digitalWrite(CH_1_CTL, HIGH); 
  digitalWrite(CH_2_CTL, HIGH);
  digitalWrite(CH_2_OT_LED, LOW);
  digitalWrite(CH_1_OT_LED, LOW);
  digitalWrite(CH_1_OC_LED, HIGH);
  digitalWrite(CH_2_OC_LED, HIGH);
  pinMode(CH_1_OT_LED, OUTPUT);
  pinMode(CH_2_OT_LED, OUTPUT);
  pinMode(CH_1_OC_LED, OUTPUT);
  pinMode(CH_2_OC_LED, OUTPUT);
  pinMode(stlPin, OUTPUT);
  pinMode(CH_1_MANUAL_SW, INPUT_PULLUP);
  pinMode(CH_2_MANUAL_SW, INPUT_PULLUP);
  pinMode(CH_1_CTL, OUTPUT);
  pinMode(CH_2_CTL, OUTPUT);
  pinMode(7, INPUT);

  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(CH_1_MANUAL_SW), manual_switch_down_1_ISR, FALLING);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(CH_1_MANUAL_SW), manual_switch_up_1_ISR, RISING);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(CH_2_MANUAL_SW), manual_switch_down_2_ISR, FALLING);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(CH_2_MANUAL_SW), manual_switch_up_2_ISR, RISING);

  analogReference(INTERNAL);
  //Turn on adc
  analogRead(A0);
}

void loop() {
  // обработка сообщений
  state = slave.poll( au16data, 3);
  // если получили пакет без ошибок - зажигаем светодиод на 50 мс 
  if (state > 4) {
    tempus = millis() + 50;
    digitalWrite(stlPin, HIGH);
  }
  if (millis() > tempus) digitalWrite(stlPin, LOW );
  //обновляем данные в регистрах Modbus и в пользовательской программе
  io_poll();

  digitalWrite( CH_1_CTL, is_ch2_manual_on);
}

void io_poll() {
  uint16_t status = digitalRead(CH_1_OT_LED   );
  status += digitalRead(CH_1_OC_LED   ) << 1;
  status += is_ch2_manual_on << 2;
  status += digitalRead(CH_2_OT_LED   ) << 3;
  status += digitalRead(CH_2_OC_LED   ) << 4;

  au16data[0] = status;
}

void zero_crossing_handler() {
  zero_cross_count++;
}

void manual_switch_down_1_ISR() {
  is_ch2_manual_on = false;
  digitalWrite(CH_1_OC_LED, HIGH);
}
void manual_switch_up_1_ISR() {
  is_ch2_manual_on = true;
  digitalWrite(CH_1_OC_LED, LOW);
}
void manual_switch_down_2_ISR() {
  //au16data[0] |= 1 << 5;
  //digitalWrite(CH_1_OC_LED, LOW);
}
void manual_switch_up_2_ISR() {
//  au16data[0] &= ~(1 << 5);
  //digitalWrite(CH_1_OC_LED, LOW);
}

void manual_switch_1_ISR() {
}

void manual_switch_2_ISR() {
}
