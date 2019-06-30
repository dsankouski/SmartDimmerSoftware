
#include "modbus_slave/ModbusRtu.h"
#include "Arduino.h"
#define BOARD_ID 1

#define CH_1_OT_LED 13
#define CH_1_OC_LED 6
#define CH_1_CTL 5
#define CH_1_MANUAL_SW 10
#define CH_2_OT_LED 8
#define CH_2_OC_LED 12
#define CH_2_CTL 9
#define CH_2_MANUAL_SW 11
#define AMP_SENSE_POS A0
#define AMP_SENSE_NEG A4
#define VCC_HALF_SENSE A5
#define CS_1 A2
#define TS_1 A3
#define CS_2 A1
#define TS_2 A8
#define btnPin  2   // номер входа, подключенный к кнопке
#define stlPin  13  // номер выхода индикатора работы
                    // расположен на плате Arduino
#define ledPin  12  // номер выхода светодиода

//Задаём ведомому адрес, последовательный порт, выход управления TX
//uint8_t BOARD_ID = 1;
Modbus slave(BOARD_ID, 0, 0); 
boolean phase;
boolean led;
int8_t state = 0;
unsigned long tempus;

// массив данных modbus
uint16_t au16data[11];

void setup() {
  //slave.setID(BOARD_ID);
  // настраиваем входы и выходы
  io_setup();
  // настраиваем последовательный порт ведомого
  slave.begin( 19200 ); 
  // зажигаем светодиод на 100 мс
  tempus = millis() + 100; 
  digitalWrite(stlPin, HIGH );
}

void io_setup() {
  digitalWrite(stlPin, HIGH ); 
  digitalWrite(ledPin, LOW ); 
  pinMode(stlPin, OUTPUT); 
  pinMode(ledPin, OUTPUT);   
  pinMode(btnPin, INPUT);  
}

void loop() {
  // обработка сообщений
  state = slave.poll( au16data, 11);  
  // если получили пакет без ошибок - зажигаем светодиод на 50 мс 
  if (state > 4) {
    tempus = millis() + 50;
    digitalWrite(stlPin, HIGH);
  }
  if (millis() > tempus) digitalWrite(stlPin, LOW );
  //обновляем данные в регистрах Modbus и в пользовательской программе
  io_poll();
} 

void io_poll() {
  //Копируем Coil[1] в Discrete[0]
  au16data[0] = au16data[1];
  //Выводим значение регистра 1.3 на светодиод 
  digitalWrite( ledPin, bitRead( au16data[1], 3 ));
  //Сохраняем состояние кнопки в регистр 0.3
  bitWrite( au16data[0], 3, digitalRead( btnPin ));
  //Копируем Holding[5,6,7] в Input[2,3,4]
  au16data[2] = au16data[5];
  au16data[3] = au16data[6];
  au16data[4] = au16data[7];
  //Сохраняем в регистры отладочную информацию
  au16data[8] = slave.getInCnt();
  au16data[9] = slave.getOutCnt();
  au16data[10] = slave.getErrCnt();
}
