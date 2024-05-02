#include <Arduino.h>
#include <Adafruit_GFX.h>    // Biblioteca gráfica básica
#include <Adafruit_I2CDevice.h>
#include <Adafruit_ST7789.h> // Biblioteca específica para o display ST7789
#include <SPI.h>             // Biblioteca SPI do Arduino

#include <DHT.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

// Definições de cores
#define BLACK    0x0000
#define BLUE     0x001F
#define RED      0xF800
#define GREEN    0x07E0
#define CYAN     0x07FF
#define MAGENTA  0xF81F
#define YELLOW   0xFFE0 
#define WHITE    0xFFFF
// Pinos do display
#define TFT_CS   15  
#define TFT_DC    2  
#define TFT_RST   4  
// Objeto do display ST7789
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

// Endereço do sensor de luz
#define BH1750_ADDR 0x23  

// Pino do sensor de UV
#define pinSensorUV 13

// Pinos do sensor DHT22
#define DHTPIN 14       
#define DHTTYPE DHT22      
// Objeto do sensor DHT22
DHT dht(DHTPIN, DHTTYPE);

// BMP-280
Adafruit_BMP280 bmp; //OBJETO DO TIPO Adafruit_BMP280 (I2C)

void TCA9548A(uint8_t id) {
    Wire.beginTransmission(0x70); // A0= LOW; A1= LOW; A2= LOW
    Wire.write(1 << id);
    Wire.endTransmission();
}

void setup() {
  Serial.begin(115200);
  
  Wire.begin();
  // Inicializa o sensor DHT22
  dht.begin();

// Inicializa o display
    tft.init(240, 240, SPI_MODE2);    
    tft.setRotation(3);
    tft.setTextSize(2);
    tft.cp437(true);
    tft.fillScreen(BLACK);

  TCA9548A(3);
  if (!bmp.begin(0x76)) { // Inicializa o BMP280 com o endereço I2C 0x76
    Serial.println(F("Sensor BMP280 não foi identificado! Verifique as conexões."));
  }

  // Inicializa o sensor de luz BH1750
  TCA9548A(1);
  Wire.beginTransmission(BH1750_ADDR);
  Wire.write(0x10); // Inicia medição com modo de alta resolução
  Wire.endTransmission();
}

void loop() {
  float humidityDht = dht.readHumidity();     
  float temperatureDht = dht.readTemperature(); 
  int valorAnalogico = analogRead(pinSensorUV);
  int percentualUV = map(valorAnalogico, 0, 1023, 0, 100);
  float luminosidade = lerLuminosidade();
  TCA9548A(3);
  float temperatureBmp = bmp.readTemperature();
  float pressureBmp = bmp.readPressure();
  float altitudeBmp = bmp.readAltitude(1013.25);

  DHT22Serial(humidityDht, temperatureDht);
  UVSerial(valorAnalogico, percentualUV);
  luminosidadeSerial(luminosidade);
  BMP280Serial(temperatureBmp, pressureBmp, altitudeBmp);

  limpaResultado();
  desenhaPrimeiraPagina(humidityDht, temperatureDht, temperatureBmp, pressureBmp, altitudeBmp);

  delay(1000); // Aguarda 2 segundos antes da próxima leitura
}

void DHT22Serial(float humidity, float temperature){
  
  if (isnan(humidity) || isnan(temperature)) {
    Serial.println("Erro ao ler o sensor DHT22!");
  }
  else {
    Serial.print("DHT22 | ");
    Serial.print("Umidade: ");
    Serial.print(humidity);
    Serial.print("% | Temperatura: ");
    Serial.print(temperature);
    Serial.println("°C");
  }
}

void UVSerial(int valorAnalogico, int percentualUV){
  Serial.print("Sensor UV | ");
  Serial.print("Valor UV: ");
  Serial.print(valorAnalogico);
  Serial.print(" | Percentual UV: ");
  Serial.println(percentualUV);
}

void luminosidadeSerial(float luminosidade) {
  if (luminosidade >= 0) {
    Serial.print("Luminosidade: ");
    Serial.print(luminosidade);
    Serial.println(" lux");
  }
}

void BMP280Serial(float temperature, float pressure, float altitude){
  Serial.print(F("Temp: "));
  Serial.print(temperature);
  Serial.println("C");

  Serial.print(F("Pressão: "));
  Serial.print(pressure);
  Serial.println(" Pa (Pascal)");

  Serial.print(F("Altitude aprox.: "));
  Serial.print(altitude,0);
  Serial.println(" m (Metros)");
}

float lerLuminosidade() {
    TCA9548A(1);
    Wire.requestFrom(BH1750_ADDR, 2);
    if (Wire.available() >= 2) {
        uint16_t valor = Wire.read() << 8 | Wire.read();
        float luminosidade = valor / 1.2; 
        return luminosidade;
    } else {
        Serial.println("Erro ao ler o sensor BH1750");
        return -1.0; 
    }
}

void limpaResultado() {
    tft.fillRoundRect(11, 11, 103, 103, 10, BLACK);
    tft.fillRoundRect(126, 11, 103, 103, 10, BLACK);
    tft.fillRoundRect(126, 126, 103, 103, 10, BLACK);
    tft.fillRoundRect(11, 125, 103, 103, 10, BLACK);
}

void desenhaPrimeiraPagina(float temperature1, float temperature2, float humidity, float pressure, float altitude) {
    desenhaTemperatura(temperature1, temperature2);
    desenhaUmidade(humidity);
    desenhaAltitude(altitude);
    desenhaPressao(pressure);
}

void desenhaTemperatura(float temperature1, float temperature2) {
    tft.drawRoundRect(10, 10, 105, 105, 10, temperature2 > 100 ? (isnan(temperature1) ? RED : YELLOW) : CYAN);
    tft.setCursor(20, 20);
    tft.println("Temp.:");
    tft.setCursor(20, 60);
    tft.print(temperature2 > 100 ? temperature1 : temperature2);
    tft.print("C");
}

void desenhaUmidade(float humidity) {
    tft.drawRoundRect(125, 10, 105, 105, 10, isnan(humidity) ? RED : CYAN);
    tft.setCursor(135, 20);
    tft.println("Hum.:");
    tft.setCursor(135, 60);
    tft.print(humidity);
    tft.print(" %");
}

void desenhaAltitude(int altitude) {
    tft.drawRoundRect(125, 125, 105, 105, 10, isnan(altitude) ? RED : CYAN);
    tft.setCursor(135, 135);
    tft.println("Alt.:");
    tft.setCursor(135, 175);
    tft.print(altitude);
    tft.println(" m.");
}

void desenhaPressao(float pressure) {
    tft.drawRoundRect(10, 125, 105, 105, 10, isnan(pressure) ? RED : CYAN);
    tft.setCursor(20, 135);
    tft.println("Press.:");
    tft.setCursor(15, 175);
    tft.print(pressure, 1);
    tft.setCursor(40, 195);
    tft.print("Pa");
}

