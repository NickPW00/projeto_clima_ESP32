#include <Arduino.h>
#include <Adafruit_GFX.h>    // Biblioteca gráfica básica
#include <Adafruit_I2CDevice.h>
#include <Adafruit_ST7789.h> // Biblioteca específica para o display ST7789
#include <SPI.h>             // Biblioteca SPI do Arduino
#include <DHT.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <string> // Inclui a biblioteca string

// Definições de cores
#define BLACK    0x0000
#define BLUE     0x001F
#define RED      0xF800
#define GREEN    0x07E0
#define CYAN     0x07FF
#define MAGENTA  0xF81F
#define YELLOW   0xFFE0 
#define WHITE    0xFFFF

// Configurações display ST7789
#define TFT_CS   15  
#define TFT_DC    2  
#define TFT_RST   4  
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

// Endereço do sensor de luz
#define BH1750_ADDR 0x23  

// Pino do sensor de UV
#define pinSensorUV 13

// Configurações DHT22
#define DHTPIN 14       
#define DHTTYPE DHT22      
DHT dht(DHTPIN, DHTTYPE);

// BMP-280
Adafruit_BMP280 bmp; //OBJETO DO TIPO Adafruit_BMP280 (I2C)

//Configurações do Pluviometro
#define PIN_PLUVIOMETRO 26 
#define MEDIDA_BASCULA 7.54 // 7.54ml each time it's activated, 9cm diameter, 4.5cm radius, 63,585 cm2 , 0,118521 L/m2
#define INTERVALO_LEITURA_PLUVIOMETRO 1000 // 1000 milisegundos que são 1 segundo
#define RAIO_PLUVIOMETRO 4.5 
#define tamanhoLista 60 // Define o tamanho da lista de contagens por sec do Pluviometro
int listaDeContagemSec[tamanhoLista]; // Lista de Contagem de quantas medições por sec do Pluviometro
int indiceInsercao = 0; // Qual espaço do array de contagem que será preenchida e lida consecutiva

// Complexador
void TCA9548A(uint8_t id) {
    Wire.beginTransmission(0x70); // A0= LOW; A1= LOW; A2= LOW
    Wire.write(1 << id);
    Wire.endTransmission();
}

void setup() {
  Serial.begin(115200);

  // Inicialização de todos os sensores que utilizam Wire
  Wire.begin();

  // Inicializa o sensor DHT22
  dht.begin();
  
  // Inicializa o display
  tft.init(240, 240, SPI_MODE2);    
  tft.setRotation(3);
  tft.setTextSize(2);
  tft.cp437(true);
  tft.fillScreen(BLACK);

  // Iniciação Pluviometro
  pinMode(PIN_PLUVIOMETRO, INPUT);

  // Inicialização BMP
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

unsigned long ultimoIntervalo = 0; 

// Pluviometro
int contagemGeral = 0; // A soma de quantas vezes foi lida a passagem do Basculante
int ultimaContagemGeral = 0;
int contagemSec = 0; 
bool reedSwitchAtivado = false;


void loop() {
  int valorDigital = digitalRead(PIN_PLUVIOMETRO); // Ele sempre será 1, e só será 0 quando o imã se aproximar no Reed Switch
  modificarContagem(valorDigital); // Sempre verifica caso o pluviometro for acionado

  if(millis() - ultimoIntervalo > INTERVALO_LEITURA_PLUVIOMETRO ){
    float mediaContagemMin = contarMlPorMinuto();

    float humidityDht = dht.readHumidity();     
    float temperatureDht = dht.readTemperature(); 
    int valorAnalogico = analogRead(pinSensorUV);
    int percentualUV = map(valorAnalogico, 0, 1023, 0, 100);
    float luminosidade = lerLuminosidade();
    
    TCA9548A(3);
    float temperatureBmp = bmp.readTemperature();
    float pressureBmp = bmp.readPressure();
    float altitudeBmp = bmp.readAltitude(1013.25);

    Serial.println("------------------------------");
    DHT22Serial(humidityDht, temperatureDht);
    UVSerial(valorAnalogico, percentualUV);
    luminosidadeSerial(luminosidade);
    BMP280Serial(temperatureBmp, pressureBmp, altitudeBmp);
    pluviometroSerial(valorDigital, mediaContagemMin);

    limpaResultado();
    desenhaPrimeiraPagina(temperatureDht, temperatureBmp, humidityDht, pressureBmp, altitudeBmp);
    contagemSec = 0;
    ultimoIntervalo = millis();
  }
}

void DHT22Serial(float humidity, float temperature){
  if (isnan(humidity) || isnan(temperature)) 
    Serial.println("Erro ao ler o sensor DHT22!");
  else {
    comumSerial("DHT22", "Umidade", humidity, "%", true, true, false);
    comumSerial("", "Temperature", temperature, "°C", true, false, true);
  }
}

void UVSerial(int valorAnalogico, int percentualUV){
  comumSerial("Sensor UV", "Valor UV", valorAnalogico, "", false, true, false);
  comumSerial("", "Valor UV", percentualUV, "%", true, false, true);
}

void luminosidadeSerial(float luminosidade) {
  if (luminosidade >= 0) {
    comumSerial("BH1750", "Luminosidade", luminosidade, "lux", true, true, true);
  }
}

void BMP280Serial(float temperature, float pressure, float altitude){
  comumSerial("BMP280", "Temp", temperature, "°C", true, true, false);
  comumSerial("", "Pressão", pressure, "Pa (Pascal)", true, false, false);
  comumSerial("", "Altitude aprox.", altitude, "m (Metros)", true, false, true);
}

void comumSerial(std::string nomeSensor, std::string nomeMedida, float medida, std::string sinalMedida, bool temSinal, bool ehPrimeiro, bool ehUltimo){
  if(ehPrimeiro){
    Serial.print(F(nomeSensor.c_str()));
    Serial.print(F(" -> "));
  }
  Serial.print(F(nomeMedida.c_str()));
  Serial.print(F(": "));
  Serial.print(medida);
  if (ehUltimo) {
    Serial.println(temSinal ? sinalMedida.c_str() : "");
  } else {
    Serial.print(temSinal ? (sinalMedida + " | ").c_str() : " | ");
  }
}

void pluviometroSerial(int valorDigital, float mediaContagemMin) {
  double mlTotais = contagemGeral * MEDIDA_BASCULA;
  double mlMedioPorMin = mediaContagemMin * MEDIDA_BASCULA;

  Serial.print("Pluviometro -> ");
  Serial.print("Basculadas: ");
  Serial.print(contagemGeral);
  Serial.print(" | mL Totais: ");
  Serial.print(mlTotais);
  Serial.print(" | ml Por Min: ");
  Serial.print(mlMedioPorMin);
  Serial.print(" | L/m2 Totais: ");
  Serial.print(medidaLporM2());
  Serial.print(" | ContagemSec ");
  Serial.println(contagemSec);
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

// Pluviometro
void modificarContagem(int valorDigital){
  if (valorDigital == LOW && !reedSwitchAtivado) contagemSegura();
  else if (valorDigital == HIGH) reedSwitchAtivado = false;
}

void contagemSegura(){
  if(contagemGeral == ultimaContagemGeral) somasContagem(); // Caso seja posta mais parametros para serem somados. Seguro pois não haverá 2 contagens ao mesmo tempo
  else ultimaContagemGeral = contagemGeral;
  reedSwitchAtivado = true;
}

void somasContagem(){
  contagemGeral++;
  contagemSec++;
}

float contarMlPorMinuto(){
  float soma = 0;
  float mediaTotal;
  listaDeContagemSec[indiceInsercao] = contagemSec; 
  indiceInsercao = (indiceInsercao + 1) % tamanhoLista;
  for (int i = 0; i < tamanhoLista; i++) soma += listaDeContagemSec[i];
  return mediaTotal = soma / tamanhoLista;
}

float medidaLporM2() {
  float areaPluviometroM2 = (PI * RAIO_PLUVIOMETRO * RAIO_PLUVIOMETRO) / 10000.0; // convertendo cm² para m²
  float medidaBasculaemM3 = MEDIDA_BASCULA * 1e-6; // transformando mL em m³
  float quantidadeDeAguaPorM2 = medidaBasculaemM3 / areaPluviometroM2;
  return quantidadeDeAguaPorM2;
}
///////////////////////////////////////////////////////////////////////////////////////////////

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
