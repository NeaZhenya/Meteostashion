#include <Wire.h>                 // библиотека для протокола IIC 
#include <LiquidCrystal_I2C.h>    // подключаем библиотеку LCD IIC//KY-037
#include "DHT.h"   // подключаем библиотеку для DHT11
#include "Adafruit_NeoPixel.h" // подключаем библиотеку
#include <GyverBME280.h>
GyverBME280 bme;
#define PIN  10              // указываем пин для подключения ленты
#define NUMPIXELS 5  // указываем количество светодиодов в ленте
int zv = A2;  // Пин Arduino к которому подключен пин D0 датчика KY037
LiquidCrystal_I2C lcd(0x27, 20, 2); // присваиваем имя lcd для дисплея
DHT dht(4, DHT11); // к какому порту подключаем датчик
byte gradus[8] = {0b01100, 0b10010, 0b10010, 0b01100, 0b00000, 0b00000, 0b00000, 0b00000};
int pinD0 = A1;                    // Пин к которому подключен D0
// создаем объект strip с нужными характеристиками
Adafruit_NeoPixel strip (NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  lcd.init();         // инициализация LCD дисплея
  lcd.backlight();    // включение подсветки дисплея
  lcd.createChar(1, gradus);
  pinMode (pinD0, INPUT);          // Установим вывод A1 как вход
  pinMode (zv, INPUT);          // Установим вывод A2 как вход
  strip.begin();                     // инициализируем ленту
  strip.setBrightness(30);  // указываем яркость светодиодов (максимум 255)
  dht.begin();
  delay(2000);
}

void loop() {
  // put your main code here, to run repeatedly:
  byte h = dht.readHumidity();    // считываем значение температуры
  byte t = dht.readTemperature(); // считываем значение влажности
  byte s = analogRead(A1);
  float paskal = bme.readPressure();
  float d = paskal / 133.322;
  float svet = digitalRead (pinD0);        // считываем значение с порта pinD0
  Serial.print("Temp: ");
  Serial.println(t);   // отправляем значение температуры на монитор
  Serial.print("Vlazh: ");
  Serial.println(h);   // отправляем значение температуры на монитор
  Serial.print ("Svet: "); // выводим значение датчика на монитор
  Serial.println (s); // выводим значение датчика на монитор
  Serial.print ("Davl: "); // выводим значение датчика на монитор
  Serial.println(d, 0); // выводим значение датчика на монитоp
  Serial.println ("  "); // выводим значение датчика на монитор

  lcd.setCursor(0, 0);
  lcd.print("TP:");    // используем латинские буквы
  lcd.print(t);        // выводим значение температуры на LCD
  lcd.print(char(1));  // выводим знак градуса
  lcd.print("  ");

  lcd.setCursor(0, 1);
  lcd.print("VL:");    // используем латинские буквы
  lcd.print(h);        // выводим значение влажности на LCD
  lcd.print("%  ");      // выводим знак процент
  lcd.setCursor(8, 0);
  lcd.print("DAVL:");
  lcd.print(d, 0);

  if (!t)
  {
    Serial.println("Датчик DHT11 (температуры и влажности) неисправен/неподключен");
    lcd.setCursor(0, 0);
    strip.setPixelColor(0, strip.Color(0, 0, 255));   // включаем синий цвет на 1 светодиоде
    strip.show();   // отправляем сигнал на лентy
    lcd.print("TP:");      // используем латинские буквы
    lcd.print("nan");
    lcd.setCursor(0, 1);
    lcd.print("VL:");      // используем латинские буквы
    lcd.print("nan");
    strip.setPixelColor(1, strip.Color(0, 0, 255));   // включаем синий цвет на 1 светодиоде
    strip.show();   // отправляем сигнал на лентy
  }


  if (s < 100)
  {
    Serial.println("Датчик света неисправен/неподключен");
    lcd.setCursor(13, 1);
    lcd.print("nan");
    strip.setPixelColor(4, strip.Color(0, 0, 255));   // включаем синий цвет на 1 светодиоде
    strip.show();   // отправляем сигнал на ленту
  }

  if (!d)
  {
    Serial.println("Датчик давления неисправен/неподключен");
    lcd.setCursor(13, 0);
    lcd.print("nan");
    strip.setPixelColor(2, strip.Color(0, 0, 255));   // включаем синий цвет на 1 светодиоде
    strip.show();   // отправляем сигнал на ленту
  }
  if (t > 26)    // Обработка значений температуры с датчика DHT11
  {
    strip.setPixelColor(0, strip.Color(255, 0, 0));   // включаем красный цвет на 1 светодиоде
    strip.show();   // отправляем сигнал на ленту
  }
  if (t <= 26 & t != 0)  // Обработка значений температуры с датчика DHT11
  {
    strip.setPixelColor(0, strip.Color(0, 255, 0));   // включаем зеленый цвет на 1 светодиоде
    strip.show();   // отправляем сигнал на ленту
  }

  if ( h > 48 )   // Обработка значений влажности с датчика DHT11
  {
    strip.setPixelColor(1, strip.Color(255, 0, 0));   // включаем красный цвет на 2 светодиоде
    strip.show();   // отправляем сигнал на ленту
  }
  if (h <= 48 & h != 0)   // Обработка значений влажности с датчика DHT11
  {
    strip.setPixelColor(1, strip.Color(0, 255, 0));   // включаем зеленый цвет на 2 светодиоде
    strip.show();   // отправляем сигнал на ленту
  }
  if (d > 740 )   // Обработка значений давления с датчика BMP280
  {
    strip.setPixelColor(2, strip.Color(255, 0, 0));   // включаем красный цвет на 3 светодиоде
    strip.show();   // отправляем сигнал на ленту
  }
  if (d <= 740 & d != !bme.begin(0x76))   // Обработка значений давления с датчика BMP280
  {
    strip.setPixelColor(2, strip.Color(0, 255, 0));   // включаем зеленый цвет на 3 светодиоде
    strip.show();   // отправляем сигнал на ленту
    lcd.setCursor(8, 1);
    lcd.print("SVET:");
  }
  if (s > 100 && s < 170)   // Обработка значений сопротивления на фоторезисторе с датчика LM393
  {
    lcd.print("ON ");
    strip.setPixelColor(4, strip.Color(255, 0, 0));   // включаем красный цвет на 5 светодиоде
    strip.show();   // отправляем сигнал на ленту
  }
  if (s > 170)      // Обработка значений сопротивления на фоторезисторе с датчика LM393
  {
    lcd.print("OFF");
    strip.setPixelColor(4, strip.Color(0, 255, 0));   // включаем зеленый цвет на 5 светодиоде
    strip.show();   // отправляем сигнал на ленту
  }
  delay(2000);
}
