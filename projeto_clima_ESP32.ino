#include <Arduino.h>
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_I2CDevice.h>
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include <SPI.h>             // Arduino SPI library

#include <DHT.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

// Color definitions
#define BLACK    0x0000
#define BLUE     0x001F
#define RED      0xF800
#define GREEN    0x07E0
#define CYAN     0x07FF
#define MAGENTA  0xF81F
#define YELLOW   0xFFE0 
#define WHITE    0xFFFF

// SENSOR DE LUZ
#define BH1750_ADDR 0x23  // Endereço padrão do sensor BH1750 (GY-30)
// DHT
#define DHTPIN 13           // Pino digital conectado ao sensor DHT22 no ESP32
#define DHTTYPE DHT22      // Definindo o tipo de sensor como DHT22

DHT dht(DHTPIN, DHTTYPE);
Adafruit_BMP280 bmp; //OBJETO DO TIPO Adafruit_BMP280 (I2C)
const int pinSensorUV = 12;

// DISPLAY
#define TFT_MOSI 23  // SDA Pin on ESP32
#define TFT_SCLK 18  // SCL Pin on ESP32
#define TFT_CS   15  // Chip select control pin
#define TFT_DC    2  // Data Command control pin
#define TFT_RST   4  // Reset pin (could connect to RST pin)

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

void TCA9548A(uint8_t id) {
Wire.beginTransmission(0x70); // A0= LOW; A1= LOW; A2= LOW
Wire.write(1 << id);
Wire.endTransmission();
}

void setup() {
  Serial.println(F("Leitura do Sensor DHT22:"));
  Wire.begin();
  dht.begin();
  TCA9548A(1);
  if(!bmp.begin(0x76)){ // Inicializa o BMP280 sem especificar o endereço I2C para o ESP32
    Serial.println(F("Sensor BMP280 não foi identificado! Verifique as conexões.")); 
  }
  // put your setup code here, to run once:
  Serial.begin(115200);
  tft.init(240, 240, SPI_MODE2);    
  tft.setRotation(3);
  tft.setTextSize(2);
  tft.cp437(true);
  tft.fillScreen(BLACK);
}

int tempo = 0;

void loop() {
  tempo++;
  int pagina = 1;

  if(tempo < 5){
    pagina = 1;
  } else if (tempo >= 5){
    pagina = 2;
    if(tempo > 6) tempo = 0;
  }


  /* if(hallRead() > 20 && pagina == 1) {
    pagina = 2;
  } else if(hallRead() > 20 && pagina == 2) {
    pagina = 1;
  } */
  retornoSerial();

  // DISPLAY
  limpaResultado();
  if(pagina == 1) primeiraPagina();
  if(pagina == 2) segundaPagina();
  

  delay(1000);
}  

void retornoSerial() {
  float humidity = dht.readHumidity();     // Leia a umidade
  float temperature = dht.readTemperature(); // Leia a temperatura em Celsius
  int valorAnalogico = analogRead(pinSensorUV);  // Lê o valor analógico do sensor
  int percentualUV = map(valorAnalogico, 0, 1023, 0, 100); // Lê e molda o UV

  //UV
  Serial.print("Sensor UV | ");
  Serial.print("Valor UV: ");
  Serial.print(valorAnalogico);
  Serial.print(" | Percentual UV: ");
  Serial.println(percentualUV);

  /* // GY-30 - deve ter o CI para pôr tudo certinho
  if (luminosidade >= 0) {
    Serial.print("Luminosidade: ");
    Serial.print(luminosidade);
    Serial.println(" lux");
  } */

  // DTH22
  if (isnan(humidity) || isnan(temperature)) {
    Serial.println(F("Erro ao ler do sensor DHT22!"));
  }
  Serial.print(F("DTH22 | "));
  Serial.print(F("Umidade: "));
  Serial.print(humidity);
  Serial.print(F("% | Temperatura: "));
  Serial.print(temperature);
  Serial.println(F("°C"));

  //BMP-280
  Serial.print(F("BMP280 | ")); 
  Serial.print(F("Pressão: ")); 
  Serial.print(bmp.readPressure()); 
  Serial.println(" Pa (Pascal)"); 
  Serial.print(F(" Temperatura: "));
  Serial.print(bmp.readTemperature());
  Serial.println(F("°C"));
  Serial.print(F("Altitude aprox.: ")); 
  Serial.print(bmp.readAltitude(1013.25),0); 
  Serial.println(" m (Metros)"); 
}

void limpaResultado() {
  tft.fillRoundRect(11, 11, 103, 103, 10, BLACK);
  tft.fillRoundRect(126, 11, 103, 103, 10, BLACK);
  tft.fillRoundRect(126, 126, 103, 103, 10, BLACK);
  tft.fillRoundRect(11, 125, 103, 103, 10, BLACK);
}

void primeiraPagina() {
  float humidity = dht.readHumidity();     // Leia a umidade
  float temperature1 = dht.readTemperature(); // Leia a temperatura em Celsius 
  float temperature2 = bmp.readTemperature();
  float preasure = bmp.readPressure();
  int altitude = (bmp.readAltitude(1013.25),0);

  tft.drawRoundRect(10, 10, 105, 105, 10, temperature2 > 100 ? (isnan(temperature1) ? RED : YELLOW) : CYAN);
  tft.setCursor(20, 20);
  tft.println("Temp.:");
  tft.setCursor(20, 60);
  tft.print(temperature2 > 100 ? temperature1 : temperature2);
  tft.print("C");

  tft.drawRoundRect(125, 10, 105, 105, 10, isnan(humidity) ? RED : CYAN);
  tft.setCursor(135, 20);
  tft.println("Hum.:");
  tft.setCursor(135, 60);
  tft.print(humidity);
  tft.print(" %");
  
  tft.drawRoundRect(125, 125, 105, 105, 10, isnan(bmp.readAltitude()) ? RED : CYAN);
  tft.setCursor(135, 135);
  tft.println("Alt.:");
  tft.setCursor(135, 175);
  tft.print(bmp.readAltitude(1013.25), 0);
  tft.println(" m.");

  tft.drawRoundRect(10, 125, 105, 105, 10, isnan(preasure) ? RED : CYAN);
  tft.setCursor(20, 135);
  tft.println("Press.:");
  tft.setCursor(15, 175);
  tft.print(preasure, 1);
  tft.setCursor(40, 195);
  tft.print("Pa");
}

void segundaPagina() {
  int valorAnalogico = analogRead(pinSensorUV);  // Lê o valor analógico do sensor
  int percentualUV = map(valorAnalogico, 0, 1023, 0, 100); // Lê e molda o UV

  tft.drawRoundRect(10, 10, 105, 105, 10, valorAnalogico == 0 ? YELLOW : CYAN);
  tft.setCursor(20, 20);
  tft.println("UV:");
  tft.setCursor(20, 60);
  tft.print(valorAnalogico);
  tft.setCursor(20, 80);
  tft.print(percentualUV);
  tft.print("%");
}