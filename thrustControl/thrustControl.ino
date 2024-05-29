#include <Servo.h>

// Переменные для двигателя
uint8_t thrustInPin_A = 5;      // Пин входящего сигнала ШИМ - ARDUPILOT
uint8_t thrustInPin_B = 6;      // Пин входящего сигнала ШИМ
uint8_t thrustOutPin = 11;      // Пин исходящего сигнала ШИМ
int minThrust = 1500; // Минимальный принимаемый ШИМ газа
int minThrustCar = minThrust;// Минимальный принимаемый ШИМ газа
int minThrustGraf = 1000; // Минимальный принимаемый ШИМ газа ARDUPILOT
int maxThrust = 2000;     // Максимальный ШИМ газа
float cruiseCoeff = 0.1;            // Коэффициент ШИМ газа на круизе
float minOutThrustVoltage = 0.9;    // Напряжение холостого хода для вывода на двигатель
float maxOutThrustVoltage = 4.95;    // Максимальное напряжение для вывода на двигатель
int currentThrust;        // Текуший входящий ШИМ газа
int currentThrustIN;        // Текуший входящий ШИМ газа исходное для танкового разворота
int currentThrustVoltage; // Текущее напряжение газа
int diff;                        // Разница сигналов газа и поворота для танкового пульта

// Переменные для сервопривода
Servo servo1; // Объект сервопривода
// int servoControlPin = 8; // БОЛЬШЕ НЕ НУЖЕН Пин сигнала, включающего управление сервоприводом
uint8_t servoInPin_A = 3; // Пин входящего сигнала ШИМ - ARDUPILOT
uint8_t servoInPin_B = 4; // Пин входящего сигнала ШИМ
uint8_t servoOutPin = 9; // Пин исходящего сигнала ШИМ
int  minServo = 1150;      // Минимальный ШИМ сервопривода
int  zeroPositionServo;    // ШИМ нулевого положения сервопривода
int  maxServo = 1930;      // Максимальный ШИМ сервопривода
int  currentServoIN;         // Текущий входящий ШИМ сервопривода исходное для танкового разворота
int  currentServo;         // Текущий входящий ШИМ сервопривода
int currentOutServo;      // Текущий исходящий ШИМ сервопривода
float scaleServo = 1.2;   // Коэффициент масштабирования входящего ШИМ

unsigned long swich = 1100;

// Общие установки
unsigned long timer;
unsigned long timerLog1;
unsigned long timerLog2;
unsigned long timelog = 50; //итерация лога в миллисекундах
bool activateLogs = true;
uint8_t box_arm = 2;
// int change_control = 8; БОЛЬШЕ НЕ НУЖЕН

void setup()
{
  // Проверка и запуск последовательного порта
  if (activateLogs)
    Serial.begin(9600);
  
  delay(2000); // Ожидание загрузки приемника
  
  currentServo = 1500;
  // Установка режима работы пинов
  pinMode(thrustOutPin, OUTPUT);
  pinMode(servoOutPin, OUTPUT); 
  pinMode(box_arm, INPUT);
  pinMode(thrustInPin_A, INPUT);
  pinMode(thrustInPin_B, INPUT);
  pinMode(servoInPin_A, INPUT);
  pinMode(servoInPin_B, INPUT);


  zeroPositionServo = pulseIn(servoInPin_B, HIGH); // Запись нулевого положения ШИМ сервопривода

  //digitalWrite(change_control, HIGH); // Устанавливаем первоначально управление по внутреннему приемнику

  //configure_reciever(); // Переключаем приемник по управляющему сигналу
  timer = millis();
  timerLog1 = millis();
  timerLog2 = millis();
  servo1.attach(servoOutPin);
}

void loop()
{
  if (millis() - timer > 5)// повторить через 0.005 секунды
  {
    swich = pulseIn(box_arm, HIGH, 20000);
   // analogWrite(thrustOutPin, 200);
    configureThrust();    // Запуск обработки ШИМ двигателя
    configureServo();     // Запуск обработки ШИМ сервопривода
    //configure_reciever(); // Запуск проверки на переключение приемника
    timer = millis(); // обнуляю таймер ожидания

  }
}

// Функция управления переключением приемников
/*
void configure_reciever()
{
  if (pulseIn(box_arm, HIGH) > 1200)
  {
    // digitalWrite(change_control, LOW);
  }
  else
  {
    // digitalWrite(change_control, HIGH);
  }
}
*/

// Функция управления двигателем
void configureThrust()
{
  if (swich > 1200)
  {
    minThrustCar = minThrustGraf;
    currentThrust = pulseIn(thrustInPin_A, HIGH); // Получение шим в микросекундах
  }
  else
  {
    minThrustCar = minThrust;
    currentThrustIN = pulseIn(thrustInPin_B, HIGH);       // Получение шим в микросекундах
    currentServoIN = pulseIn(servoInPin_B, HIGH); // Получение шим в микросекундах для танкового пульта
    diff = abs(currentThrustIN-currentServoIN); // для танкового пульта
Serial.println(diff);
    currentThrust = currentThrustIN;
    if (diff > 30 || currentThrust <= 1400)                           // Проверка на отклонение стика газа назад или в бок для танкового пульта
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
 // if (pulseIn(box_arm, HIGH) > 1200) // Проверка активации Ардупилота
  if (swich > 1200)            // Проверка активации Ардупилота
  {
    minThrustCar = minThrustGraf;
  }
  else
  {
    minThrustCar = minThrust;
    //if (currentThrust < minThrustCar && currentThrust >= 1500)   // Проверка ШИМ на выход за минимальное значение
    //  minThrustCar = currentThrust;                              // Корректировка минимального положения
    if (currentThrust < minThrustCar + 40)                       // Проверка на холостой ход с мертвой зоной
      return getOutSignalFromVoltage(minOutThrustVoltage, true); // Установка холостого хода
    currentThrust = currentThrust - 40;                          // Корректировка входящего ШИМ для избавления от мервой зоны
  }

  if (currentThrust > maxThrust) // Проверка ШИМ на выход за максимальное значение
    currentThrust = maxThrust;   // Корректировка максимального значения
  float percentage = (float)(currentThrust - minThrustCar) / (maxThrust - minThrustCar); // Расчет процента установленного газа
  return getOutSignalFromVoltage(minOutThrustVoltage, true) * (1 - percentage) + (255 * percentage * maxOutThrustVoltage / 5); // Расчет дискретной величины, определяющей напряжение, и возврат этого показателя
}

// Функция логгер для функции управления двигателем
void thrustLogger(int currentThrust, int currentThrustVoltage)
{
  if (millis() - timerLog1 > timelog)// Итерация вывода в миллисекундах
  {
    Serial.print("Thrust: Current duration: ");
    Serial.print(currentThrust);
    Serial.print("; Out signal: ");
    Serial.print(currentThrustVoltage);
    Serial.print("; Out voltage: ");
    Serial.println(((float)currentThrustVoltage / 255) * 5);
    timerLog1 = millis();// обнулить таймер итерации
  }
}

// Функция управления сервоприводом
void configureServo()
{
  //if (pulseIn(box_arm, HIGH) > 1200)            // Проверка активации Ардупилота
  if (swich > 1200)            // Проверка активации Ардупилота
  {                                             // тут работает Ардупилот
    currentServo = pulseIn(servoInPin_A, HIGH); // Получение ШИМ в микросекундах
    currentOutServo = currentServo;
  }
  else
  {                                             // тут работает маленький пульт
    //currentServo = pulseIn(servoInPin_B, HIGH); // Получение ШИМ в микросекундах

    //if (currentServo == 0 || (currentServo <= zeroPositionServo + 40 && currentServo >= zeroPositionServo - 40)) // Проверка на отсутствие сигнала ШИМ либо на мертвую зону
    //  currentOutServo = 1500;                                                                                    // Установка нулевого положения ШИМ сервопривода

    //if (currentServo <= zeroPositionServo - 40 || currentServo >= zeroPositionServo + 40) // Проверка на выход из мертвой зоны ШИМ сервопривода
    //{
      // Расчет исходящего ШИМ сервопривода, масштабирование сигнала, ограничение угла в зависимости от оборотов двигателя для локального управления
    //  currentOutServo = zeroPositionServo - (currentServo - zeroPositionServo) * scaleServo * (1 - max(0, 0.5 * ((currentThrust - minThrust * (1 + cruiseCoeff)) / (maxThrust - minThrust * (1 + cruiseCoeff)))));
    //}
    // для танкового пульта:
   // diff = abs(currentServo - pulseIn(thrustInPin_B, HIGH)); 
   currentServo = currentServoIN;
    if (diff > 30)
    {
      currentOutServo = currentThrustIN;
    }
    else
    {
      currentOutServo = zeroPositionServo;
    }
  }

  // обрезать концы что бы руль не упирался в случае передозы
  if (currentOutServo > maxServo)
    currentOutServo = maxServo;
  if (currentOutServo < minServo)
    currentOutServo = minServo;

  servo1.writeMicroseconds(currentOutServo); // Вывод обработанного ШИМ на сервопривод

  // Проверка и запуск логгера функции управления сервоприводом
  if (activateLogs)
    servoLogger();

  currentOutServo = 1500; // Обнуление переменной ШИМ для случая отказа приемника, на следующей итерации ШИМ встранет в нулевое положение - 1500 микросекунд
}

// Функция логгер для функции управления сервоприводом
void servoLogger()
{
  if(millis() - timerLog2 > timelog)// Итерация вывода в миллисекундах
  {
    Serial.print("Servo: Current In duration: ");
    Serial.print(currentServo);
    Serial.print("; Current Out duration: ");
    Serial.println(currentOutServo);
    timerLog2 = millis();// обнулить таймер итерации
  }
}
