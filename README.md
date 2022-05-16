# RF24 датчик протечки воды
Датчик протечки воды на ATmega168/328 + RF24.

## Функции:

1. Потребление энергии в спящем режиме около 10 мкА. До 4 лет работы на одной батарейке CR2450 (ориентировочно).
2. Передает уровень заряда батареи и состояние датчика (DRY) каждый час (можно изменить до 65535 секунд).
3. При срабатывании датчик передает сигнал тревоги (ALARM) каждую минуту. Для прекращения передачи сигнала тревоги необходимо перезагрузить датчик.
4. Автоматический перезапуск в случае зависания.

## Примечание:

Подробности см. в папке "hardware".