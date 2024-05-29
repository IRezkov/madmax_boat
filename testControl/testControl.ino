#include <Servo.h>

int InputServo2;

Servo servo9; // Объект сервопривода
Servo servo10;

// Общие установки
unsigned long timerLog2;

uint8_t inputservoport3 = 3;


void setup()
{
	// Проверка и запуск последовательного порта
	Serial.begin(115200);
	
	pinMode(inputservoport3, INPUT);

	pinMode(9, OUTPUT);
    pinMode(10, OUTPUT);
	servo9.attach(9);
    servo10.attach(10);
}


void loop()
{
	InputServo2 = pulseIn(inputservoport3, HIGH);
	servo9.writeMicroseconds(InputServo2); // Вывод обработанного ШИМ на сервопривод
    servo10.writeMicroseconds(InputServo2); // Вывод обработанного ШИМ на сервопривод

	//Logger();
    delay(100);
}


// Функция логгер
void Logger()
{
	if(millis() - timerLog2 > 50)// Итерация вывода в миллисекундах
	{
		Serial.println(InputServo2);
		timerLog2 = millis();// обнулить таймер итерации
	}
}
