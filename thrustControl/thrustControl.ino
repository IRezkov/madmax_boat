#include <Servo.h>

// Переменные для двигателя
uint8_t thrustInPin_Ardu = 5;			// Пин входящего сигнала ШИМ - ARDUPILOT
uint8_t thrustInPin_RC = 6;			// Пин входящего сигнала ШИМ
uint8_t thrustOutPin = 10;			// Пин исходящего сигнала ШИМ 490 Гц на VOTOL
int minThrust = 1500; // Минимальный принимаемый ШИМ газа
int minThrustCar = minThrust;// Минимальный принимаемый ШИМ газа
int minThrustGraf = 1000; // Минимальный принимаемый ШИМ газа ARDUPILOT
int maxThrust = 2000;		 // Максимальный ШИМ газа
float cruiseCoeff = 0.1;						// Коэффициент ШИМ газа на круизе
float minOutThrustVoltage = 0.9;		// Напряжение холостого хода для вывода на двигатель
float maxOutThrustVoltage = 4.95;		// Максимальное напряжение для вывода на двигатель
int currentThrotleInPulse;				// Текуший входящий ШИМ газа
int currentThrotleInPulseIN;			// Текуший входящий ШИМ газа исходное для танкового разворота
int currentThrottleOutVoltage; // Текущее напряжение газа
int diff;												// Разница сигналов газа и поворота для танкового пульта

// Переменные для сервопривода
Servo servo1; // Объект сервопривода
uint8_t servoInPin_Ardu = 3; // Пин входящего сигнала ШИМ - ARDUPILOT
uint8_t servoInPin_RC = 4; // Пин входящего сигнала ШИМ
uint8_t servoOutPin = 9; // Пин исходящего сигнала ШИМ
int	minServo = 1150;			// Минимальный ШИМ сервопривода
int	zeroPositionServo;		// ШИМ нулевого положения сервопривода
int	maxServo = 1930;			// Максимальный ШИМ сервопривода
int	currentServoIN;				 // Текущий входящий ШИМ сервопривода исходное для танкового разворота
int	currentServo;				 // Текущий входящий ШИМ сервопривода
int currentOutServo;			// Текущий исходящий ШИМ сервопривода
int currentOutServoLog; //для лога
float scaleServo = 1.2;	 // Коэффициент масштабирования входящего ШИМ

unsigned long arm_value = 1100;

// Общие установки
unsigned long timer;
unsigned long timerLog1;
unsigned long timerLog2;
unsigned long timelog = 50; //итерация лога в миллисекундах
bool activateLogs = true;
uint8_t box_arm = 2;


void setup()
{
	// Проверка и запуск последовательного порта
	if (activateLogs)
		Serial.begin(115200);
	
	delay(2000); // Ожидание загрузки приемника
	
	currentServo = 1500;
	// Установка режима работы пинов
	pinMode(thrustOutPin, OUTPUT);
	pinMode(servoOutPin, OUTPUT); 
	pinMode(box_arm, INPUT);
	pinMode(thrustInPin_Ardu, INPUT);
	pinMode(thrustInPin_RC, INPUT);
	pinMode(servoInPin_Ardu, INPUT);
	pinMode(servoInPin_RC, INPUT);


	zeroPositionServo = pulseIn(servoInPin_RC, HIGH); // Запись нулевого положения ШИМ сервопривода

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
		arm_value = pulseIn(box_arm, HIGH, 20000);

		configureThrust();		// Запуск обработки ШИМ двигателя
		configureServo();		 // Запуск обработки ШИМ сервопривода
		
		Log();


		timer = millis(); // обнуляю таймер ожидания
	}
}


// Функция управления двигателем
void configureThrust()
{
	if (arm_value > 1200)
	{
		minThrustCar = minThrustGraf;
		currentThrotleInPulse = pulseIn(thrustInPin_Ardu, HIGH); // Получение шим в микросекундах
	}
	else
	{
		minThrustCar = minThrust;
		currentThrotleInPulseIN = pulseIn(thrustInPin_RC, HIGH);			 // Получение шим в микросекундах
		currentServoIN = pulseIn(servoInPin_RC, HIGH); // Получение шим в микросекундах для танкового пульта
		diff = abs(currentThrotleInPulseIN-currentServoIN); // для танкового пульта


		
		currentThrotleInPulse = currentThrotleInPulseIN;
		if (diff > 30 || currentThrotleInPulse <= 1400)													 // Проверка на отклонение стика газа назад или в бок для танкового пульта
			currentThrotleInPulse = minThrustCar * (1 + cruiseCoeff); // Установка ШИМ круиза
		
	}

	currentThrottleOutVoltage = getThrustVoltage(currentThrotleInPulse); // Расчет напряжения круиза

	analogWrite(thrustOutPin, currentThrottleOutVoltage); // Вывод напряжения на двигатель


}

void Log()
{
	if (activateLogs)
	{		
		if (millis() - timerLog1 > timelog)// Итерация вывода в миллисекундах
		{
			Serial.print(arm_value);
			Serial.print(" ");	
			Serial.print(currentThrotleInPulse);
			Serial.print(" ");
			Serial.print(currentServoIN);
			Serial.print(" ");
			
			Serial.print(diff);
			Serial.print(" ");
			
			Serial.print(currentThrottleOutVoltage);
			Serial.print(" ");
			
			Serial.println(currentOutServoLog);
			
			timerLog1 = millis();// обнулить таймер итерации
		}
	}
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
int getThrustVoltage(int currentThrotleInPulse)
{
 // if (pulseIn(box_arm, HIGH) > 1200) // Проверка активации Ардупилота
	if (arm_value > 1200)						// Проверка активации Ардупилота
	{
		minThrustCar = minThrustGraf;
	}
	else
	{
		minThrustCar = minThrust;
		//if (currentThrotleInPulse < minThrustCar && currentThrotleInPulse >= 1500)	 // Проверка ШИМ на выход за минимальное значение
		//	minThrustCar = currentThrotleInPulse;															// Корректировка минимального положения
		if (currentThrotleInPulse < minThrustCar + 40)											 // Проверка на холостой ход с мертвой зоной
			return getOutSignalFromVoltage(minOutThrustVoltage, true); // Установка холостого хода
		currentThrotleInPulse = currentThrotleInPulse - 40;													// Корректировка входящего ШИМ для избавления от мервой зоны
	}

	if (currentThrotleInPulse > maxThrust) // Проверка ШИМ на выход за максимальное значение
		currentThrotleInPulse = maxThrust;	 // Корректировка максимального значения
	float percentage = (float)(currentThrotleInPulse - minThrustCar) / (maxThrust - minThrustCar); // Расчет процента установленного газа
	return getOutSignalFromVoltage(minOutThrustVoltage, true) * (1 - percentage) + (255 * percentage * maxOutThrustVoltage / 5); // Расчет дискретной величины, определяющей напряжение, и возврат этого показателя
}


// Функция управления сервоприводом
void configureServo()
{
	//if (pulseIn(box_arm, HIGH) > 1200)						// Проверка активации Ардупилота
	if (arm_value > 1200)						// Проверка активации Ардупилота
	{																						 // тут работает Ардупилот
		currentServo = pulseIn(servoInPin_Ardu, HIGH); // Получение ШИМ в микросекундах
		currentOutServo = currentServo;
	}
	else
	{																						 // тут работает маленький пульт
		//currentServo = pulseIn(servoInPin_RC, HIGH); // Получение ШИМ в микросекундах

		//if (currentServo == 0 || (currentServo <= zeroPositionServo + 40 && currentServo >= zeroPositionServo - 40)) // Проверка на отсутствие сигнала ШИМ либо на мертвую зону
		//	currentOutServo = 1500;																																										// Установка нулевого положения ШИМ сервопривода

		//if (currentServo <= zeroPositionServo - 40 || currentServo >= zeroPositionServo + 40) // Проверка на выход из мертвой зоны ШИМ сервопривода
		//{
			// Расчет исходящего ШИМ сервопривода, масштабирование сигнала, ограничение угла в зависимости от оборотов двигателя для локального управления
		//	currentOutServo = zeroPositionServo - (currentServo - zeroPositionServo) * scaleServo * (1 - max(0, 0.5 * ((currentThrotleInPulse - minThrust * (1 + cruiseCoeff)) / (maxThrust - minThrust * (1 + cruiseCoeff)))));
		//}
		// для танкового пульта:
		// diff = abs(currentServo - pulseIn(thrustInPin_RC, HIGH)); 
		
		currentServo = currentServoIN;
		if (diff > 30)
		{
			currentOutServo = currentThrotleInPulseIN;
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
	currentOutServoLog = currentOutServo;

	currentOutServo = 1500; // Обнуление переменной ШИМ для случая отказа приемника, на следующей итерации ШИМ встранет в нулевое положение - 1500 микросекунд
}
