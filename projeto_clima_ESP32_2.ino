#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <ArduinoJson.h>

// WiFi Configuration
const char* ssid = "";
const char* password = "";
const char* serverName = "";

// Client NTP Configuration
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", -10800, 60000);

// Definições de cores
#define BLACK    0x0000
#define RED      0xF800
#define GREEN    0x07E0
#define CYAN     0x07FF
#define YELLOW   0xFFE0 
#define WHITE    0xFFFF

// Configurações display ST7789
#define TFT_CS   15  
#define TFT_DC    2  
#define TFT_RST   4  

// Sensor Pins
#define BH1750_ADDR 0x23   // Light Sensor Address
#define pinSensorUV 13     // UV Sensor Pin
#define PIN_PLUVIOMETRO 26 // Rain Gauge Pin

// Rain Gauge Configuration
#define MEDIDA_BASCULA 7.54   // 7.54ml each time it's activated, 9cm diameter, 4.5cm radius, 63,585 cm2 , 0,118521 L/m2
#define RAIO_PLUVIOMETRO 4.4
#define TAMANHO_LISTA 60      // Define o tamanho da lista de contagens por sec do Pluviometro
#define AREA_PLUVIOMETRO (RAIO_PLUVIOMETRO * RAIO_PLUVIOMETRO * PI)

// Deep Sleep Configuration
#define uS_TO_S_FACTOR 1000000 /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  510       /* Time ESP32 will go to sleep (in seconds) */

// Object Declarations
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST); // Display 
Adafruit_BME280 bme;                                            // BME280 Sensor

// Valiables Declarations
int listaDeContagemSec[TAMANHO_LISTA]; // Lista de Contagem de quantas medições por sec do Pluviometro
int indiceInsercao = 0;                // Qual espaço do array de contagem que será preenchida e lida consecutiva
RTC_DATA_ATTR int bootCount = 0;

// Complexador 
void TCA9548A(uint8_t id) {
    Wire.beginTransmission(0x70); // A0= LOW; A1= LOW; A2= LOW
    Wire.write(1 << id);
    Wire.endTransmission();
}

struct SensorReadings {
  float luminosidade;
  float temperatureBme;
  float humidityBme;
  float pressureBme;
  float altitudeBme;
  float mediaContagemMin;
  int valorAnalogico;
  int percentualUV;
};

// Function Declarations;
void setupWiFi();
void setupDisplay();
void setupSensors();
void print_wakeup_reason();
void serialOutputs(SensorReadings readings);
void displayOutputs(SensorReadings readings);
void sendReadingsToServer(SensorReadings readings);
String createJsonPayload(SensorReadings readings);
SensorReadings readSensors();

// Botão para troca das telas
int buttonPin = 25;
int botaoAcionado = 0;      // Variavel para declarar qual 
int buttonState;

void setup() {
  Serial.begin(115200);
  delay(1000); // Deixa a estação acordar direitinho :D
  setupDeepSleep();
  pinMode(buttonPin, INPUT); // botão
  setupWiFi();
  setupDisplay();
  setupSensors();
  timeClient.begin();
}

void setupWiFi(){
  WiFi.begin(ssid, password);                 // Conecte-se à rede Wi-Fi
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Conectando ao WiFi...");  // Esperando se conectar com a internet
  }  
  Serial.println("Conectado ao WiFi!");      
  Serial.print("Endereço IP: ");
  Serial.println(WiFi.localIP());             // Imprima o endereço IP quando conectado
}

void setupDisplay(){
  tft.init(240, 240, SPI_MODE2); // Inicializa o display
  tft.setRotation(3);
  tft.setTextSize(2);
  tft.cp437(true);
  tft.fillScreen(BLACK);
}

void setupSensors() {
  Wire.begin();                     // Inicialização de todos os sensores que utilizam Wire
  pinMode(PIN_PLUVIOMETRO, INPUT);  // Iniciação Pluviometro

  // Inicializa o BME280 com o endereço I2C 0x76
  TCA9548A(3);
  if (!bme.begin(0x76)) Serial.println(F("Sensor BME280 não foi identificado! Verifique as conexões.")); 
  
  TCA9548A(2);                          // Inicializa o sensor de luz BH1750
  Wire.beginTransmission(BH1750_ADDR);
  Wire.write(0x10);                     // Inicia medição com modo de alta resolução
  Wire.endTransmission();
}

// Deep Sleep Mode //////////////////////////////////////////////////

void setupDeepSleep() {
  ++bootCount;
  Serial.println("Boot number: " + String(bootCount));
  print_wakeup_reason();
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) + " Seconds");
}

void print_wakeup_reason() {
  esp_sleep_wakeup_cause_t wakeup_reason;
  wakeup_reason = esp_sleep_get_wakeup_cause();
  switch (wakeup_reason) {
    case ESP_SLEEP_WAKEUP_EXT0:     Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1:     Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER:    Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD: Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP:      Serial.println("Wakeup caused by ULP program"); break;
    default:                        Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason); break;
  }
}

// Loop /////////////////////////////////////////////////////// 

unsigned long ultimoIntervaloGeral = 0; 
unsigned long ultimoIntervaloRequisicao = 0; 

int contagemGeralBasculas = 0; // A soma de quantas vezes foi lida a passagem do Basculante
int ultimaContagemGeral = 0;
int contagemSec = 0; 

void loop() {
  buttonState = digitalRead(buttonPin);  // Lê o estado do botão
  int valorDigitalPluviometro = digitalRead(PIN_PLUVIOMETRO); // Ele sempre será 1, e só será 0 quando o imã se aproximar no Reed Switch
  modificarContagem(valorDigitalPluviometro); // Sempre verifica caso o pluviometro for acionado

  if(buttonState == HIGH) botaoAcionado++;
  if(millis() - ultimoIntervaloGeral > 1000){
    SensorReadings readings = readSensors();
    serialOutputs(readings);
    displayOutputs(readings);
    
    if(millis() - ultimoIntervaloRequisicao > 90000){
      sendReadingsToServer(readings);
      ultimoIntervaloRequisicao = millis();
      Serial.println("Going to sleep now");
      Serial.flush();
      esp_deep_sleep_start();
    }
    contagemSec = 0;
    ultimoIntervaloGeral = millis();
  }
}

// Mandando para a API /////////////////////////////////////////////////////// 

SensorReadings readSensors() {
  SensorReadings readings;
  readings.valorAnalogico = analogRead(pinSensorUV);
  readings.percentualUV = map(readings.valorAnalogico, 0, 1023, 0, 100);
  readings.luminosidade = lerLuminosidade();
  readings.mediaContagemMin = contarMlPorMinuto();
  TCA9548A(3);
  readings.temperatureBme = bme.readTemperature();
  readings.humidityBme = bme.readHumidity();
  readings.pressureBme = bme.readPressure();
  readings.altitudeBme = bme.readAltitude(1013.25);
  return readings;
}

String getFormattedTimestamp() {
  timeClient.update();
  unsigned long epochTime = timeClient.getEpochTime();
  struct tm *ptm = gmtime((time_t *)&epochTime);
  char timestamp[25];
  snprintf(timestamp, sizeof(timestamp), "%04d-%02d-%02dT%02d:%02d:%02dZ",
           ptm->tm_year + 1900, ptm->tm_mon + 1, ptm->tm_mday,
           ptm->tm_hour, ptm->tm_min, ptm->tm_sec);
  return String(timestamp);
}

String createJsonPayload(SensorReadings readings) {
  // Criando um objeto JSON com capacidade suficiente para os dados
  StaticJsonDocument<256> doc;

  // Preenchendo o objeto JSON com os dados
  doc["idEstacao"] = "Teste Principal";
  doc["timestamp"] = getFormattedTimestamp();
  doc["numeroMedicao"] = 1;
  doc["temperatura"] = readings.temperatureBme;
  doc["umidade"] = readings.humidityBme;
  doc["percentualUV"] = readings.percentualUV;
  doc["nivelUV"] = readings.valorAnalogico;
  doc["pressao"] = readings.pressureBme;
  doc["luminosidade"] = readings.luminosidade;
  doc["mlChuva"] = readings.mediaContagemMin;

  // Convertendo o objeto JSON em uma string
  String jsonString;
  serializeJson(doc, jsonString);
  return jsonString;
}

void sendReadingsToServer(SensorReadings readings) {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(serverName);
    http.addHeader("Content-Type", "application/json");
    String httpRequestData = createJsonPayload(readings);
    int httpResponseCode = http.POST(httpRequestData);
    serialSendReadingsToServer(httpResponseCode, http);
    http.end();
  } else {
    Serial.println("Desconectado do WiFi");
  }
}

void serialSendReadingsToServer(int httpResponseCode, HTTPClient &http){
  if (httpResponseCode > 0) {
      Serial.print("Código de Resposta HTTP: ");
      Serial.println(httpResponseCode);
      String payload = http.getString();
      Serial.println("Resposta do servidor:");
      Serial.println(payload);
    } else {
      Serial.print("Erro na solicitação HTTP: ");
      Serial.println(httpResponseCode);
    }
}

// PreSets /////////////////////////////////////////////////////// 

float lerLuminosidade() {
  TCA9548A(2);
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

// Seriais /////////////////////////////////////////////////////// 

void serialOutputs(SensorReadings readings) {
  Serial.println("------------------------------");
  luminosidadeSerial(readings.luminosidade);
  UVSerial(readings.valorAnalogico, readings.percentualUV);
  pluviometroSerial(readings.mediaContagemMin);
  BME280Serial(readings.temperatureBme, readings.humidityBme, readings.pressureBme, readings.altitudeBme);
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

void BME280Serial(float temperature, float humidity, float pressure, float altitude){
  comumSerial("BME280", "Temp", temperature, "°C", true, true, false);
  comumSerial("", "Umidade", humidity, "%", true, false, false);
  comumSerial("", "Pressão", pressure, "Pa (Pascal)", true, false, false);
  comumSerial("", "Altitude aprox.", altitude, "m (Metros)", true, false, true);
}

void pluviometroSerial(float mediaContagemMin) {
  double mlTotais = contagemGeralBasculas * MEDIDA_BASCULA;
  double mlMedioPorMin = mediaContagemMin * MEDIDA_BASCULA;
  float mmTotais = mmPorM2();

  comumSerial("Pluviometro", "Basculas", contagemGeralBasculas, "", false, true, false);
  comumSerial("", "ml Totais", mlTotais, "ml", true, false, false);
  comumSerial("", "L/m2 Totais", mmTotais, "L/m2", true, false, false);
  comumSerial("", "Basculas por sec", contagemSec, "", false, false, true);
}

void comumSerial(const char* nomeSensor, const char* nomeMedida, float medida, const char* sinalMedida, bool temSinal, bool ehPrimeiro, bool ehUltimo) {
  if (ehPrimeiro) {
    Serial.print(nomeSensor);
    Serial.print(F(" -> "));
  }

  Serial.print(nomeMedida);
  Serial.print(F(": "));
  Serial.print(medida);

  if (temSinal) Serial.print(sinalMedida);
  if (ehUltimo) Serial.println();
  else          Serial.print(F(" | "));
}

// Pluviometro /////////////////////////////////////////////////////// 

bool reedSwitchAtivado = true;
unsigned long intervalo2 = 0;

void modificarContagem(int valorDigital) {
  if (valorDigital == HIGH && millis() - intervalo2 > 500){
    contagemGeralBasculas++;
    contagemSec++;
    intervalo2 = millis();
  }
}

// Função para calcular a média de ml por minuto
float contarMlPorMinuto() {
  float soma = 0;
  listaDeContagemSec[indiceInsercao] = contagemSec;
  indiceInsercao = (indiceInsercao + 1) % TAMANHO_LISTA;
  for (int i = 0; i < TAMANHO_LISTA; i++) {
    soma += listaDeContagemSec[i];
  }
  contagemSec = 0; // Reset contagemSec depois de calcular a média
  return soma / TAMANHO_LISTA;
}

// Função para calcular a quantidade de chuva em mm por m²
float mmPorM2() {
  float mmPorM2 = MEDIDA_BASCULA / AREA_PLUVIOMETRO;
  return contagemGeralBasculas * mmPorM2;
}

// Display - Opcional /////////////////////////////////////////////////////////////////////////////////////////////

void displayOutputs(SensorReadings readings) {
  bool condicaoBotao = botaoAcionado % 2;
  limpaResultado();
  if(!condicaoBotao) desenhaPrimeiraPagina(readings.temperatureBme, readings.humidityBme, readings.pressureBme, readings.altitudeBme);
  if(condicaoBotao) desenhaSegundaPagina(readings.luminosidade, readings.valorAnalogico, readings.percentualUV, readings.mediaContagemMin);
}

void limpaResultado() {
    tft.fillRoundRect(11, 11, 103, 103, 10, BLACK);
    tft.fillRoundRect(126, 11, 103, 103, 10, BLACK);
    tft.fillRoundRect(126, 126, 103, 103, 10, BLACK);
    tft.fillRoundRect(11, 125, 103, 103, 10, BLACK);
}

void desenhaPrimeiraPagina(float temperature, float humidity, float pressure, float altitude) {
  drawSensorData("Temp.:", temperature, "C", 1, false);
  drawSensorData("Hum.:", humidity, "%", 2, false);
  drawSensorData("Alt.:", altitude, "m", 3, false);
  drawSensorData("Press.:", pressure, "Pa", 4, true);
}

void desenhaSegundaPagina(float luminosidade, float valorAnalogico, float percentualUV, float mediaContagemMin) {
  drawSensorData("Lum.:", luminosidade, "lux", 1, true);
  drawSensorData("UV.:", percentualUV, "%", 2, false);
  drawSensorData("Bas.:", contagemGeralBasculas * MEDIDA_BASCULA, "ml", 3, false);
  drawSensorData("ML/Min.:", mediaContagemMin, "ml/min", 4, true);
}

void drawSensorData(const char* sensorName, float sensorValue, const char* unit, int tela, bool ultrapassa) {
    int x; int y;
    if(tela == 1){x = 10; y = 10;}
    if(tela == 2){x = 125; y = 10;}
    if(tela == 3){x = 10; y = 125;}
    if(tela == 4){x = 125; y = 125;}
    tft.drawRoundRect(x, y, 105, 105, 10, isnan(sensorValue) ? RED : CYAN);
    tft.setCursor(x + 10, y + 10);
    tft.print(sensorName);
    tft.setCursor(x + 10, y + 60);
    tft.print(sensorValue);
    if(ultrapassa){
      tft.setCursor(x + 10, y + 80);
    }
    tft.print(unit);
}
