#include <Servo.h>

// Переменные для двигателя
int thrustInPin = 4;  // Пин входящего сигнала ШИМ
int thrustOutPin = 3; // Пин исходящего сигнала ШИМ
int minThrust = 1500; // Минимальный ШИМ газа
int minThrustCar = minThrust;
int minThrustGraf = 1000;        // Минимальный ШИМ газа
int maxThrust = 2000;            // Максимальный ШИМ газа
float cruiseCoeff = 0.1;         // Коэффициент ШИМ газа на круизе
float minOutThrustVoltage = 0.9; // Напряжение холостого хода для вывода на двигатель
float maxOutThrustVoltage = 4.8; // Максимальное напряжение для вывода на двигатель
int currentThrust;               // Текуший входящий ШИМ газа
int currentThrustVoltage;        // Текущее напряжение газа

// Переменные для сервопривода
Servo servo1;            // Объект сервопривода
int servoControlPin = 8; // Пин сигнала, включающего управление сервоприводом
int servoInPin = 5;      // Пин входящего сигнала ШИМ
int servoOutPin = 6;     // Пин исходящего сигнала ШИМ
int minServo = 1150;     // Минимальный ШИМ сервопривода
int zeroPositionServo;   // ШИМ нулевого положения сервопривода
int maxServo = 1930;     // Максимальный ШИМ сервопривода
int currentServo;        // Текущий входящий ШИМ сервопривода
int currentOutServo;     // Текущий исходящий ШИМ сервопривода
float scaleServo = 1.2;  // Коэффициент масштабирования входящего ШИМ

// Общие установки
bool activateLogs = false;
int box_arm = 7;
int change_control = 8;

void setup()
{
  // Проверка и запуск последовательного порта
  if (activateLogs)
    Serial.begin(9600);

  delay(2000); // Ожидание загрузки приемника

  zeroPositionServo = pulseIn(servoInPin, HIGH); // Запись нулевого положения ШИМ сервопривода

  // Установка режима работы пинов
  pinMode(thrustOutPin, OUTPUT);
  pinMode(servoControlPin, INPUT);
  pinMode(box_arm, INPUT);
  pinMode(change_control, OUTPUT);

  digitalWrite(change_control, HIGH); // Устанавливаем первоначально управление по внутреннему приемнику

  configure_reciever(); // Переключаем приемник по управляющему сигналу

  servo1.attach(servoOutPin);
}

void loop()
{
  configureThrust(); // Запуск обработки ШИМ двигателя

  configureServo(); // Запуск обработки ШИМ сервопривода

  configure_reciever(); // Запуск проверки на переключение приемника

  delay(5); // Задержка 0.005 секунду
}

// Функция управления переключением приемников
void configure_reciever()
{
  if (pulseIn(box_arm, HIGH) > 1200)
  {
    digitalWrite(change_control, LOW);
  }
  else
  {
    digitalWrite(change_control, HIGH);
  }
}

// Функция управления двигателем
void configureThrust()
{

  currentThrust = pulseIn(thrustInPin, HIGH); // Получение шим в микросекундах

  if (pulseIn(box_arm, HIGH) > 1200)
  {
    minThrustCar = minThrustGraf;
  }
  else
  {
    minThrustCar = minThrust;
    if (currentThrust < 1000)                           // Проверка на отклонение стика газа назад
      currentThrust = minThrustCar * (1 + cruiseCoeff); // Установка ШИМ круиза
  }

  currentThrustVoltage = getThrustVoltage(currentThrust); // Расчет напряжения круиза

  analogWrite(thrustOutPin, currentThrustVoltage); // Вывод напряжения на двигатель

  // Проверка и запуск логгера функции управления двигателем
  if (activateLogs)
    thrustLogger(currentThrust, currentThrustVoltage);
}

// Функция расчета дискретной величины, определяющей напряжение на выводе, от напряжения переданного в вольтах
int getOutSignalFromVoltage(float voltage, bool withoutProcceed)
{
  if (withoutProcceed)
  {
    return (255 / 5) * voltage;
  }
  else
  {
    return (255 / 5) * voltage * maxOutThrustVoltage / 5;
  }
}

// Функция обработки ШИМ двигателя
int getThrustVoltage(int currentThrust)
{
  if (pulseIn(box_arm, HIGH) > 1200)
  {
    minThrustCar = minThrustGraf;
  }
  else
  {
    minThrustCar = minThrust;
    if (currentThrust < minThrustCar && currentThrust >= 1500)   // Проверка ШИМ на выход за минимальное значение
      minThrustCar = currentThrust;                              // Корректировка минимального положения
    if (currentThrust < minThrustCar + 80)                       // Проверка на холостой ход с мертвой зоной
      return getOutSignalFromVoltage(minOutThrustVoltage, true); // Установка холостого хода
    currentThrust = currentThrust - 80;                          // Корректировка входящего ШИМ для избавления от мервой зоны
  }

  if (currentThrust > maxThrust) // Проверка ШИМ на выход за максимальное значение
    currentThrust = maxThrust;   // Корректировка максимального значения

  float percentage = (float)(currentThrust - minThrustCar) / (maxThrust - minThrustCar); // Расчет процента установленного газа

  return getOutSignalFromVoltage(minOutThrustVoltage, true) * (1 - percentage) + (255 * percentage * maxOutThrustVoltage / 5); // Расчет дискретной величины, определяющей напряжение, и возврат этого показателя
}

// Функция логгер для функции управления двигателем
void thrustLogger(int currentThrust, int currentThrustVoltage)
{
  Serial.print("Thrust: Current duration: ");
  Serial.print(currentThrust);
  Serial.print("; Out signal: ");
  Serial.print(currentThrustVoltage);
  Serial.print("; Out voltage: ");
  Serial.println(((float)currentThrustVoltage / 255) * 5);
}

// Функция управления сервоприводом
void configureServo()
{
  currentServo = pulseIn(servoInPin, HIGH); // Получение ШИМ в микросекундах

  if (currentServo == 0 || (currentServo <= zeroPositionServo + 40 && currentServo >= zeroPositionServo - 40)) // Проверка на отсутствие сигнала ШИМ либо на мертвую зону
    currentOutServo = 1500;                                                                                    // Установка нулевого положения ШИМ сервопривода

  if (currentServo <= zeroPositionServo - 40 || currentServo >= zeroPositionServo + 40) // Проверка на выход из мертвой зоны ШИМ сервопривода
    // Расчет исходящего ШИМ сервопривода, масштабирование сигнала, ограничение угла в зависимости от оборотов двигателя
    currentOutServo = zeroPositionServo - (currentServo - zeroPositionServo) * scaleServo * (1 - max(0, 0.5 * ((currentThrust - minThrust * (1 + cruiseCoeff)) / (maxThrust - minThrust * (1 + cruiseCoeff)))));

  servo1.writeMicroseconds(currentOutServo); // Вывод обработанного ШИМ на сервопривод

  // Проверка и запуск логгера функции управления сервоприводом
  if (activateLogs)
    servoLogger();

  currentOutServo = 0; // Обнуление переменной ШИМ для случая отказа приемника, на следующей итерации ШИМ встранет в нулевое положение - 1500 микросекунд
}

// Функция логгер для функции управления сервоприводом
void servoLogger()
{
  Serial.print("Servo: Current In duration: ");
  Serial.print(currentServo);
  Serial.print("; Current Out duration: ");
  Serial.println(currentOutServo);
}
