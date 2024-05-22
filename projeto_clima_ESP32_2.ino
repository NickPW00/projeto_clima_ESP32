#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>
#include <DHT.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <TimeLib.h>

// WiFi Configuration
const char* ssid = "Hermes";
const char* password = "Dionisio401";
const char* serverName = "http://192.168.0.202:8080/medicao";

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

// Sensor Pins
#define BH1750_ADDR 0x23   // Light Sensor Address
#define pinSensorUV 13     // UV Sensor Pin
#define DHTPIN 14          // DHT Sensor Pin
#define PIN_PLUVIOMETRO 26 // Rain Gauge Pin

//Sensor Types
#define DHTTYPE DHT22   

// Rain Gauge Configuration
#define MEDIDA_BASCULA 7.54   // 7.54ml each time it's activated, 9cm diameter, 4.5cm radius, 63,585 cm2 , 0,118521 L/m2
#define RAIO_PLUVIOMETRO 4.5 
#define TAMANHO_LISTA 60      // Define o tamanho da lista de contagens por sec do Pluviometro

// Object Declarations
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST); // Display 
DHT dht(DHTPIN, DHTTYPE);                                       // DHT22 Sensor
Adafruit_BMP280 bmp;                                            // BMP280 Sensor

// Valiables Declarations
int listaDeContagemSec[TAMANHO_LISTA]; // Lista de Contagem de quantas medições por sec do Pluviometro
int indiceInsercao = 0;                // Qual espaço do array de contagem que será preenchida e lida consecutiva

// Complexador 
void TCA9548A(uint8_t id) {
    Wire.beginTransmission(0x70); // A0= LOW; A1= LOW; A2= LOW
    Wire.write(1 << id);
    Wire.endTransmission();
}

struct SensorReadings {
  float humidityDht;
  float temperatureDht;
  float luminosidade;
  float temperatureBmp;
  float pressureBmp;
  float altitudeBmp;
  float mediaContagemMin;
  int valorAnalogico;
  int percentualUV;
};


// Function Declarations;
void setupWiFi();
void setupDisplay();
void setupSensors();
void displayReadings(SensorReadings readings);
void sendReadingsToServer(SensorReadings readings);
SensorReadings readSensors();
String createJsonPayload(SensorReadings readings);

void setup() {
  Serial.begin(115200);

  setupWiFi();
  setupDisplay();
  setupSensors();
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
  dht.begin();                      // Inicializa o sensor DHT22
  pinMode(PIN_PLUVIOMETRO, INPUT);  // Iniciação Pluviometro

  // Inicializa o BMP280 com o endereço I2C 0x76
  TCA9548A(3);
  if (!bmp.begin(0x76)) Serial.println(F("Sensor BMP280 não foi identificado! Verifique as conexões.")); 
  
  TCA9548A(1);                          // Inicializa o sensor de luz BH1750
  Wire.beginTransmission(BH1750_ADDR);
  Wire.write(0x10);                     // Inicia medição com modo de alta resolução
  Wire.endTransmission();
}

unsigned long ultimoIntervalo = 0; 

// Pluviometro
int contagemGeral = 0; // A soma de quantas vezes foi lida a passagem do Basculante
int ultimaContagemGeral = 0;
int contagemSec = 0; 
bool reedSwitchAtivado = false;



SensorReadings readSensors() {
  SensorReadings readings;

  readings.humidityDht = dht.readHumidity();
  readings.temperatureDht = dht.readTemperature();
  readings.valorAnalogico = analogRead(pinSensorUV);
  readings.percentualUV = map(readings.valorAnalogico, 0, 1023, 0, 100);
  readings.luminosidade = lerLuminosidade();

  TCA9548A(3);
  readings.temperatureBmp = bmp.readTemperature();
  readings.pressureBmp = bmp.readPressure();
  readings.altitudeBmp = bmp.readAltitude(1013.25);

  readings.mediaContagemMin = contarMlPorMinuto();

  return readings;
}

void displayReadings(SensorReadings readings) {
  Serial.println("------------------------------");
  DHT22Serial(readings.humidityDht, readings.temperatureDht);
  UVSerial(readings.valorAnalogico, readings.percentualUV);
  luminosidadeSerial(readings.luminosidade);
  BMP280Serial(readings.temperatureBmp, readings.pressureBmp, readings.altitudeBmp);
  pluviometroSerial(digitalRead(PIN_PLUVIOMETRO), readings.mediaContagemMin);

  limpaResultado();
  desenhaPrimeiraPagina(readings.temperatureDht, readings.temperatureBmp, readings.humidityDht, readings.pressureBmp, readings.altitudeBmp);
}

void sendReadingsToServer(SensorReadings readings) {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(serverName);
    http.addHeader("Content-Type", "application/json");

    String httpRequestData = createJsonPayload(readings);

    int httpResponseCode = http.POST(httpRequestData);

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
    http.end();
  } else {
    Serial.println("Desconectado do WiFi");
  }
}

String createJsonPayload(SensorReadings readings) {
  // Formatando o timestamp no formato ISO 8601
  String timestamp = String(year()) + "-" + String(month()) + "-" + String(day()) +
                     "T" + String(hour()) + ":" + String(minute()) + ":" + String(second());

  // Construindo o JSON com os dados da leitura dos sensores
  String json = "{";
  json += "\"idEstacao\": \"XYZ123\",";
  json += "\"timestamp\": \"" + timestamp + "\",";
  json += "\"numeroMedicao\": 1,"; // Número sequencial da medição
  json += "\"temperatura\": " + String(readings.temperatureDht, 2) + ",";
  json += "\"umidade\": " + String(readings.humidityDht, 2) + ",";
  json += "\"percentualUV\": " + String(readings.percentualUV) + ",";
  json += "\"nivelUV\": " + String(readings.valorAnalogico) + ",";
  json += "\"pressao\": " + String(readings.pressureBmp, 2) + ",";
  json += "\"luminosidade\": " + String(readings.luminosidade, 2) + ",";
  json += "\"mlChuva\": " + String(readings.mediaContagemMin, 2); // Quantidade de chuva em milímetros por minuto
  json += "}";
  return json;
}

void loop() {
  int valorDigital = digitalRead(PIN_PLUVIOMETRO); // Ele sempre será 1, e só será 0 quando o imã se aproximar no Reed Switch
  modificarContagem(valorDigital); // Sempre verifica caso o pluviometro for acionado

  if(millis() - ultimoIntervalo > 1000){
    SensorReadings readings = readSensors();
    displayReadings(readings);
    sendReadingsToServer(readings);
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

void comumSerial(char* nomeSensor, char* nomeMedida, float medida, char* sinalMedida, bool temSinal, bool ehPrimeiro, bool ehUltimo) {
  if(ehPrimeiro) {
    Serial.print(nomeSensor);
    Serial.print(F(" -> "));
  }
  Serial.print(nomeMedida);
  Serial.print(F(": "));
  Serial.print(medida);
  if (ehUltimo) {
    if (temSinal) {
      Serial.println(sinalMedida);
    } else {
      Serial.println();
    }
  } else {
    if (temSinal) {
      Serial.print(sinalMedida);
      Serial.print(F(" | "));
    } else {
      Serial.print(F(" | "));
    }
  }
}

void pluviometroSerial(int valorDigital, float mediaContagemMin) {
  double mlTotais = contagemGeral * MEDIDA_BASCULA;
  double mlMedioPorMin = mediaContagemMin * MEDIDA_BASCULA;

  comumSerial("Pluviometro", "Basculas", contagemGeral, "", false, true, false);
  comumSerial("", "ml Totais", mlTotais, "ml", true, false, false);
  comumSerial("", "L/m2 Totais", medidaLporM2(), "L/m2", true, false, false);
  comumSerial("", "Basculas por sec", contagemSec, "", false, false, true);
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

// Pluviometro /////////////////////////////////////////////////////////////////
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
  indiceInsercao = (indiceInsercao + 1) % TAMANHO_LISTA;
  for (int i = 0; i < TAMANHO_LISTA; i++) soma += listaDeContagemSec[i];
  return mediaTotal = soma / TAMANHO_LISTA;
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
  drawSensorData("Temp.:", temperature2 > 100 ? temperature1 : temperature2, "C", 1, false);
  drawSensorData("Hum.:", humidity, "%", 2, false);
  drawSensorData("Alt.:", altitude, "m", 3, false);
  drawSensorData("Press.:", pressure, "Pa", 4, true);
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
