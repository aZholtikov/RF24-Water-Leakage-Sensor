#include "Arduino.h"
#include "Arduino_FreeRTOS.h"
#include "nRF24L01.h"
#include "RF24.h"
#include "avr/interrupt.h"
#include "semphr.h"
#include "ZHRF24SensorProtocol.h"

#define ID 1 // Уникальный идентификатор устройства RF24 в сети.
#define PIPE 0xAABBCC
#define CHANNEL 120
#define SLEEP_TIME 3600 // В секундах.

RF24 radio(9, 10);
TaskHandle_t xSendBatteryLevelCharge;

void sendBatteryLevelCharge(void *pvParameters);
void sendAlarmStatus(void *pvParameters);
float getBatteryLevelCharge(void);

void setup()
{
  EICRA |= (1 << ISC01) | (1 << ISC00);
  EIMSK |= (1 << INT0);

  ADCSRA &= ~(1 << ADEN);

  radio.begin();
  radio.setChannel(CHANNEL);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MAX);
  radio.setPayloadSize(14);
  radio.setAddressWidth(3);
  radio.setCRCLength(RF24_CRC_8);
  radio.enableDynamicAck();
  radio.setAutoAck(false);
  radio.openWritingPipe(PIPE);
  radio.powerDown();

  xTaskCreate(sendBatteryLevelCharge, "Send Battery Level Charge", configMINIMAL_STACK_SIZE, NULL, 1, &xSendBatteryLevelCharge);
}

void loop()
{
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  portENTER_CRITICAL();
  sleep_enable();
  portEXIT_CRITICAL();
  sleep_cpu();
  sleep_reset();
}

void sendBatteryLevelCharge(void *pvParameters)
{
  (void)pvParameters;
  for (;;)
  {
    TransmittedData sensor{ID, WATER_LEAKAGE};
    sensor.value_1 = getBatteryLevelCharge() * 100; // *100 для передачи значения float в int. На стороне получателя оно преобразуется обратно в float.
    sensor.value_2 = DRY;
    radio.powerUp();
    radio.flush_tx();
    for (int i{0}; i < 3; ++i)
      radio.writeFast(&sensor, sizeof(struct TransmittedData), true);
    radio.txStandBy();
    radio.powerDown();
    vTaskDelay(SLEEP_TIME);
  }
  vTaskDelete(NULL);
}

void sendAlarmStatus(void *pvParameters)
{
  (void)pvParameters;
  for (;;)
  {
    TransmittedData sensor{ID, WATER_LEAKAGE};
    sensor.value_2 = ALARM;
    radio.powerUp();
    radio.flush_tx();
    for (int i{0}; i < 3; ++i)
      radio.writeFast(&sensor, sizeof(struct TransmittedData), true);
    radio.txStandBy();
    radio.powerDown();
    vTaskDelay(60);
  }
  vTaskDelete(NULL);
}

float getBatteryLevelCharge()
{
  ADMUX = (1 << REFS0) | (1 << MUX3) | (1 << MUX2) | (1 << MUX1) | (0 << MUX0);
  ADCSRA |= (1 << ADEN);
  delay(10);
  ADCSRA |= (1 << ADSC);
  while (bit_is_set(ADCSRA, ADSC))
    ;
  ADCSRA &= ~(1 << ADEN);
  float value = ((1024 * 1.1) / (ADCL + ADCH * 256));
  return value;
}

ISR(INT0_vect)
{
  cli();
  vTaskDelete(xSendBatteryLevelCharge);
  xTaskCreate(sendAlarmStatus, "Send Alarm Status", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
}