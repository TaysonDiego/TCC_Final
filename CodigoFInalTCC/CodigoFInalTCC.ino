//==================================================
// BIBLIOTECAS
//==================================================
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_GC9A01A.h>

#include "Protocentral_MAX30205.h"
#include "MAX30105.h"
#include "heartRate.h"
#include "spo2_algorithm.h"

#include <Adafruit_MPU6050.h>

#include <HTTPClient.h>

#include <WiFi.h>
#include <time.h>
#include <math.h>

//==================================================
// DEFINIÇÃO DOS PINOS
//==================================================
#define TFT_CS   10
#define TFT_DC    2
#define TFT_RST   3

#define BUZZER    6
#define BOTAO     1

//==================================================
// OBJETOS
//==================================================
Adafruit_GC9A01A tft(TFT_CS, TFT_DC, TFT_RST);

MAX30105 Sensor_Batimento_e_Spo2;
MAX30205 Sensor_Temperatura;
Adafruit_MPU6050 Sensor_Queda;

//==================================================
// CONFIGURAÇÕES DE WIFI
//==================================================
const char* Nome = "Thaysla casa";
const char* Senha = "Taysoncasa.#";

//==================================================
// CONFIGURAÇÕES DA HORA
//==================================================
const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = -10800;
const int daylightOffset_sec = 0;

//==================================================
// API
//==================================================
char* url = "https://api-monitoramento-pvcc.onrender.com/dados";
//==================================================
// CONFIGURAÇÕES DA TELA
//==================================================
int alturaGradiente = 120;

int wifix = 130;
int wifiy = 220;

int coracaox = 40;
int coracaoy = 100;
bool TamanhoCoracao = false;


int gotax = 40;
int gotay = 160;

int termometrox = 140;
int termometroy = 130;

//==================================================
// VARIÁVEIS GERAIS
//==================================================
bool sensorBPMeSPO2Conectado = false;
bool sensorTempConectado = false;
bool sensorQuedaConectado = false;

unsigned long intervaloCoracao = 0;
unsigned long ultimaAnimacao = 0;
unsigned long ultimaAtualizacaoHora = 0;
unsigned long ultimoEnvioAPI = 0;

int ContatoSensor = 0;

//==================================================
// SENSOR MAX30102
//==================================================
long RED;
long IR;

//---------------- BPM ----------------//
float BatimentosPorMinuto = 0;
int MediaBatimento = 0;

long UltimoBatimento = 0;
long Delta = 0;

const byte Frequencia_Tamanho = 4;
byte Frequencia[Frequencia_Tamanho];
byte FrequenciaSpot = 0;

//---------------- SPO2 ----------------//
const int BufferSize = 50;

uint32_t BufferIR[BufferSize];
uint32_t BufferRed[BufferSize];

int32_t Spo2;
int8_t Spo2Valido;

int32_t BPM;
int8_t BPMvalido;

int BufferIndex = 0;

//==================================================
// SENSOR DE TEMPERATURA
//==================================================
float tempC = 0;

//==================================================
// ALERTA
//==================================================
bool alertaAtivo = false;
unsigned long inicioAlerta = 0;

//==================================================
// SENSOR DE QUEDA
//==================================================
sensors_event_t aceleracao;
sensors_event_t giroscopio;
sensors_event_t temperaturadosensor;

float AccMagnitude;
float WccMagnitude;

float movimento = 0;

bool PossivelQueda = false;
bool QuedaVerdadeira = false;
unsigned long tempoQueda = 0;

//==================================================
// SERIAL
//==================================================
unsigned long ultimoPrint = 0;

//==================================================
// FUNÇÕES
//==================================================
void fundo(uint16_t corTopo);
void circulo(uint16_t cor);

void hora();
void AtualizarHora();

void wifi (uint16_t corWifi);
void arco(int arcox, int arcoy, int raio, uint16_t corWifi);

void coracao(int tamanhoCoracao);
void animacaoCoracao();

void inicializacaoMAX30102();
void LerMAX30102();
void DetectarBatimento();
void AtualizarSPO2();
void gota();

void termometro();
void LerMAX30205();

void LerMPU6050();
void DetectarQueda();

void alerta();
void alertaVisual();
void desligarAlerta();

void MostrarDados();

//==================================================
// SETUP
//==================================================
void setup()
{
  /* Configura os Pinos e o Serial*/
  Serial.begin(115200);
  delay(1000);
  SPI.begin(5, -1, 4);
  Wire.begin(8, 9);
  pinMode(BUZZER, OUTPUT);
  pinMode(BOTAO, INPUT);
  /* Inicializa o Display */
  tft.begin();
  tft.setRotation(0);
  fundo(tft.color565(138, 0, 196));
  circulo(tft.color565(138, 0, 196));
  /* Inicializa o Wi-Fi */
  WiFi.begin(Nome, Senha);
  /* Checa se o Nome e Senha estão corretos */
  while (WiFi.status() != WL_CONNECTED) {
    wifi(tft.color565(255, 255, 255));
    delay(500);
    wifi(tft.color565(0, 0, 0));
    delay(500);
  }
  /* Configura a hora */
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  /* Caso o Wi-Fi conectar o simbolo fica verde */
  wifi(tft.color565(0, 255, 0));
  /* Mostra a hora e data */
  hora();
  /* Inicialização dos sensores */
  Serial.println("Iniciando sensores...");
  // ============================
  // Sensor de Batimento e SpO2
  // ============================
  if (Sensor_Batimento_e_Spo2.begin(Wire, I2C_SPEED_STANDARD)) {
    Serial.println("MAX30102 conectado");
    sensorBPMeSPO2Conectado = true;
    inicializacaoMAX30102();
  } else {
    Serial.println("MAX30102 nao encontrado");
  }
  // ============================
  // Sensor de Temperatura
  // ============================
  if(!Sensor_Temperatura.scanAvailableSensors())
  {
    Serial.println("MAX30205 nao encontrado!");
  }else{
    Serial.println("MAX30205 encontrado!");
    Sensor_Temperatura.begin();
    sensorTempConectado = true;
  }
  // =========================
  // MPU6050
  // =========================
  if (!Sensor_Queda.begin(0x68, &Wire)) {
    Serial.println("MPU6050 nao encontrado");
  }else{
    Serial.println("MPU6050 encontrado!");
    sensorQuedaConectado = true;
    Sensor_Queda.setAccelerometerRange(MPU6050_RANGE_8_G);
    Sensor_Queda.setGyroRange(MPU6050_RANGE_500_DEG);
    Sensor_Queda.setFilterBandwidth(MPU6050_BAND_21_HZ);
  }
  gota();
  termometro();
}

//==================================================
// LOOP
//==================================================
void loop()
{
  AtualizarHora();
  LerMAX30102();
  LerMAX30205();
  LerMPU6050();
  desligarAlerta();
  DetectarQueda();
  animacaoCoracao();
  MostrarDados();
}

//==================================================
// FUNÇÕES
//==================================================
//---------------- Desenhar Fundo ----------------//
void fundo(uint16_t corTopo){
   // Fundo base
  tft.fillScreen(GC9A01A_BLACK);

    // Extrai RGB da cor recebida
  uint8_t r1 = ((corTopo >> 11) & 0x1F) << 3;
  uint8_t g1 = ((corTopo >> 5) & 0x3F) << 2;
  uint8_t b1 = (corTopo & 0x1F) << 3;

  // Cor final (preto)
  float r2 = 0, g2 = 0, b2 = 0;

  for (int y = 0; y < alturaGradiente; y++) {

    float p = (float)y / (alturaGradiente - 1);

    // suavização (ease-out)
    p = 1 - (1 - p) * (1 - p);

    uint8_t r = r1 + (r2 - r1) * p;
    uint8_t g = g1 + (g2 - g1) * p;
    uint8_t b = b1 + (b2 - b1) * p;

    uint16_t cor = tft.color565(r, g, b);

    tft.drawFastHLine(0, y, 240, cor);
  }
}
//---------------- HORA ----------------//
void hora() {

  struct tm timeinfo;

  if (getLocalTime(&timeinfo)) {

    // Data DD/MM
    char data[6];

    sprintf(
      data,
      "%02d/%02d",
      timeinfo.tm_mday,
      timeinfo.tm_mon + 1
    );

    // Dias da semana
    const char* dias[] = {
      "DOM",
      "SEG",
      "TER",
      "QUA",
      "QUI",
      "SEX",
      "SAB"
    };

    // Data
    tft.setTextSize(2);
    tft.setTextColor(GC9A01A_WHITE, GC9A01A_BLACK);
    tft.setCursor(70, 20);
    tft.print(data);

    // Separador
    tft.fillRect(133, 20, 2, 14, tft.color565(2, 225, 230));

    // Dia da semana
    tft.setCursor(140, 20);
    tft.print(dias[timeinfo.tm_wday]);

    // Hora
    tft.setTextSize(4);

    // Horas
    tft.setTextColor(GC9A01A_WHITE, GC9A01A_BLACK);
    tft.setCursor(70, 50);
    tft.printf("%02d", timeinfo.tm_hour);

    // :
    tft.setTextColor(tft.color565(138, 0, 196));
    tft.setCursor(112, 50);
    tft.print(":");

    // Minutos
    tft.setTextColor(GC9A01A_WHITE, GC9A01A_BLACK);
    tft.setCursor(130, 50);
    tft.printf("%02d", timeinfo.tm_min);
  }
}
//---------------- ATUALIZAR HORA ----------------//
void AtualizarHora(){
  // Atualiza a hora a cada segundo
  if (millis() - ultimaAtualizacaoHora >= 1000) {
    
    hora();

    ultimaAtualizacaoHora = millis();
  }
}
//---------------- HORA PARA API ----------------//
String dataAtual() {

  struct tm timeinfo;

  if (!getLocalTime(&timeinfo)) {
    return "";
  }

  char buffer[20];

  strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", &timeinfo);

  return String(buffer);
}

//---------------- Símbolo Wi-Fi ----------------//
void wifi (uint16_t corWifi){

  // apaga símbolo anterior
  arco(wifix, wifiy, 8, GC9A01A_BLACK);
  arco(wifix, wifiy, 14, GC9A01A_BLACK);
  arco(wifix, wifiy, 20, GC9A01A_BLACK);
  tft.fillCircle(wifix, wifiy, 2, GC9A01A_BLACK);
  //desenha
  arco(wifix, wifiy, 8, corWifi);
  arco(wifix, wifiy, 14, corWifi);
  arco(wifix, wifiy, 20, corWifi);

  tft.fillCircle(wifix, wifiy, 2, corWifi);
}
//---------------- Os risco do sinal ----------------//
void arco(int arcox, int arcoy, int raio, uint16_t corWifi) {
  for (int ang = 220; ang <= 320; ang++) {
    float rad = ang * PI / 180.0;

    int px = arcox + raio * cos(rad);
    int py = arcoy + raio * sin(rad);

    tft.fillCircle(px, py, 1, corWifi);
  }
}
//---------------- Desenhar Coração ----------------//
void coracao(int tamanhoCoracao){
  tft.fillCircle(coracaox - tamanhoCoracao, coracaoy, tamanhoCoracao, GC9A01A_RED);
  tft.fillCircle(coracaox + tamanhoCoracao, coracaoy, tamanhoCoracao, GC9A01A_RED);
  tft.fillTriangle(
    coracaox - 2 * tamanhoCoracao, coracaoy + 2,
    coracaox + 2 * tamanhoCoracao, coracaoy + 2,
    coracaox, coracaoy + 3 * tamanhoCoracao,
    GC9A01A_RED
  );
}

//---------------- Animar Coração ----------------//
void animacaoCoracao(){
  if (MediaBatimento > 0){
    intervaloCoracao = 60000 / MediaBatimento;
    // 60 BPM - 1 vez por segundo
    // 90 BPM - 1,5 vezes por segundo
    // 120 BPM - 2 vezes por segundo
    if (millis() - ultimaAnimacao >= intervaloCoracao){
      ultimaAnimacao = millis();
      /* Apagar o antigo */
      tft.fillRect(coracaox - 25, coracaoy - 10, 50, 45, GC9A01A_BLACK);
      TamanhoCoracao = !TamanhoCoracao;
      if(TamanhoCoracao){
        coracao(10);
      }else{
        coracao(8);
      }
    }
  }
}
//---------------- Trazer o SpO2 da propria biblioteca ----------------//
void inicializacaoMAX30102(){
  byte brilhoLED = 60;
  byte media = 4;
  byte modoLED = 2;
  int taxaAmostra = 200;
  int larguraPulso = 411;
  int faixaADC = 4096;
  // Configuração do sensor
  Sensor_Batimento_e_Spo2.setup(
    brilhoLED,
    media,
    modoLED,
    taxaAmostra,
    larguraPulso,
    faixaADC
  );
  // Buffer inicial do SpO2
  Serial.println("Coletando amostras iniciais...");

  for (int i = 0; i < BufferSize; i++) {

    while (!Sensor_Batimento_e_Spo2.available()) {

      Sensor_Batimento_e_Spo2.check();
    }

    BufferRed[i] = Sensor_Batimento_e_Spo2.getRed();
    BufferIR[i] = Sensor_Batimento_e_Spo2.getIR();

    Sensor_Batimento_e_Spo2.nextSample();
  }
}
//---------------- Detectar se esta em contato com a pele ----------------//
void LerMAX30102() {

  Sensor_Batimento_e_Spo2.check();

  if (Sensor_Batimento_e_Spo2.available()) {

    IR = Sensor_Batimento_e_Spo2.getIR();
    RED = Sensor_Batimento_e_Spo2.getRed();

    Sensor_Batimento_e_Spo2.nextSample();

    // DETECTADO
    if (IR > 50000) {

      DetectarBatimento();

      AtualizarSPO2();
    }
    // CONTATO FRACO
    else if (IR >= 10000 && IR < 50000) {
      
      BatimentosPorMinuto = 0;
      MediaBatimento = 0;
      BufferIndex = 0;
      Spo2=0;
    }

    // SEM CONTATO
    else {

      BatimentosPorMinuto = 0;
      MediaBatimento = 0;
      BufferIndex = 0;
      Spo2=0;
    }
  }
}

//---------------- Detectar o Batimento ----------------//
void DetectarBatimento() {

  if (checkForBeat(IR)) {

    Delta = millis() - UltimoBatimento;

    UltimoBatimento = millis();

    BatimentosPorMinuto =
      60.0 / (Delta / 1000.0);

    if (BatimentosPorMinuto > 20 &&
        BatimentosPorMinuto < 255) {

      Frequencia[FrequenciaSpot++] =
        (byte)BatimentosPorMinuto;

      FrequenciaSpot %= Frequencia_Tamanho;

      MediaBatimento = 0;

      for (byte x = 0;
           x < Frequencia_Tamanho;
           x++) {

        MediaBatimento += Frequencia[x];
      }

      MediaBatimento /= Frequencia_Tamanho;
    }
  }
}

//---------------- Trazer novos dados do SpO2 ----------------//
void AtualizarSPO2() {

  BufferRed[BufferIndex] = RED;
  BufferIR[BufferIndex] = IR;

  BufferIndex++;

  if (BufferIndex >= BufferSize) {

    maxim_heart_rate_and_oxygen_saturation(
      BufferIR,
      BufferSize,
      BufferRed,
      &Spo2,
      &Spo2Valido,
      &BPM,
      &BPMvalido
    );

    BufferIndex = 0;
  }
}
void gota() {

  // Parte redonda da gota
  tft.fillCircle(gotax, gotay, 10, tft.color565(0,191,255));

  // Ponta da gota
  tft.fillTriangle(
    gotax - 10, gotay,
    gotax + 10, gotay,
    gotax, gotay - 18,
    tft.color565(0,191,255)
  );
 
  tft.setTextColor(GC9A01A_WHITE);
  tft.setTextSize(2);
  tft.setCursor(45, 155);
  tft.print("O");

  tft.setTextSize(1);
  tft.setCursor(58, 165);
  tft.print("2");
}
//---------------- Desenhar Termometro ----------------//
void termometro(){

  uint16_t vermelho = GC9A01A_RED;
  uint16_t amarelo = tft.color565(255, 220, 0);

  // Bulbo (contorno)
  tft.fillCircle(termometrox, termometroy + 30, 10, vermelho);

  // Tubo (contorno)
  tft.fillRoundRect(termometrox - 4, termometroy, 8, 30, 4, vermelho);

  // Interior do tubo
  tft.fillRoundRect(termometrox - 2, termometroy + 2, 4, 26, 2, amarelo);

  // Interior do bulbo
  tft.fillCircle(termometrox, termometroy + 30, 7, amarelo);
}

//---------------- Ler a Temperatura Corporal ----------------//
void LerMAX30205(){
  if (sensorTempConectado) {

    float leitura = Sensor_Temperatura.getTemperature();

    // filtro de segurança
    if (leitura > 10 && leitura < 45) {
      tempC = leitura;
    }

  } else {
    tempC = 0;
  }
}
//---------------- Configura Sensor de Queda  ----------------//
void LerMPU6050() {

  Sensor_Queda.getEvent(
    &aceleracao,
    &giroscopio,
    &temperaturadosensor
  );

  AccMagnitude = sqrt(

    aceleracao.acceleration.x *
    aceleracao.acceleration.x +

    aceleracao.acceleration.y *
    aceleracao.acceleration.y +

    aceleracao.acceleration.z *
    aceleracao.acceleration.z
  );

  WccMagnitude = sqrt(

    giroscopio.gyro.x *
    giroscopio.gyro.x +

    giroscopio.gyro.y *
    giroscopio.gyro.y +

    giroscopio.gyro.z *
    giroscopio.gyro.z
  );

  movimento = abs(AccMagnitude - 9.81);
}

//---------------- Detectar q Queda ----------------//
void DetectarQueda() {

  if (movimento >= 10 &&
      WccMagnitude > 2.0) {

    PossivelQueda = true;

    tempoQueda = millis();

    alertaAtivo = true;
    inicioAlerta = millis();
  }

  if (PossivelQueda) {

    if (millis() - tempoQueda > 3000) {

      if (movimento < 1.5 &&
          WccMagnitude < 0.2) {
        QuedaVerdadeira = true;
        Serial.println("\nQUEDA DETECTADA\n");
        circulo(tft.color565(255, 0, 0));
        tone(BUZZER, 2000);
      }
      QuedaVerdadeira = false;
      PossivelQueda = false;
    }
  }
}
//---------------- Desenhar o Circulo em volta do display ----------------//
void circulo(uint16_t cor){
  for(int i=0;i<6;i++){
    tft.drawCircle(120,118,118-i,cor);
  }
}
//---------------- Som de alerta ----------------//
void alerta(){

  if (!alertaAtivo) {
    noTone(BUZZER);
    return;
  }

  if (millis() - inicioAlerta < 3000) {

    if ((millis() / 500) % 2)
      tone(BUZZER, 1500);
    else
      noTone(BUZZER);

  } else {

    noTone(BUZZER);
    alertaAtivo = false;
  }
}
//---------------- Alerta Visual ----------------//
void alertaVisual() {

  static unsigned long tempoPisca = 0;
  static bool estadoPisca = false;

  if (millis() - tempoPisca >= 300) {

    tempoPisca = millis();

    estadoPisca = !estadoPisca;

    if (estadoPisca) {
      // Amarelo
      circulo(tft.color565(247, 239, 5));
    }
    else {
      // Roxo
      circulo(tft.color565(138, 0, 196));
    }
  }
}
//---------------- Desligar o Alerta ----------------//
void desligarAlerta(){
  if(digitalRead(BOTAO) == HIGH){
    alertaAtivo = false;
    PossivelQueda = false;

    noTone(BUZZER);

    circulo(tft.color565(138,0,196));

    Serial.println("ALARME CANCELADO");
  }
}
//---------------- ENVIO DA API ----------------//
void EnviarAPI() {
  /* Envia a cada 5 segundos */ 
  if (millis() - ultimoEnvioAPI < 5000) {
    return; // Não envia nada
  }
  ultimoEnvioAPI = millis();
  /* Se o Wi-Fi estiver funcionando envia a API */
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(url);
    http.addHeader(
      "Content-Type",
      "application/json"
    );
    http.addHeader(
      "Authorization",
      "X%BNY&9@1s8!A" /* TOKEN */
    );
    // DEFINE MOVIMENTO
    int nivelMovimento = 0;
    if (movimento < 1.5 && WccMagnitude < 0.2) {
      /* Parado */
      nivelMovimento = 0;
    } else if (movimento < 4) {
      /* Movimento Leve */
      nivelMovimento = 1;
    } else if (movimento < 10){
      /* Andando */
      nivelMovimento = 2;
    }
    // JSON
    String json = "{";
    /* dados para envio */
    // Cardiaco
    json += "\"Batimento_Atual\":"  + String(BatimentosPorMinuto) + ",";
    json += "\"Media_Batimento\":"  + String(MediaBatimento) + ",";
    // Spo2
    json += "\"Oxigenacao\":"      + String(Spo2) + ",";
    // Temperatura
    json += "\"Temperatura\":"     + String(tempC) + ",";
    // ContatoSensorBPM
    json += "\"Contato_Sensor\":"     + String(ContatoSensor) + ","; // 0 sem contato, 1 contato fraco e 2 encostado
    // Aceleracao
    json += "\"Aceleracao_X\":"    + String(aceleracao.acceleration.x) + ",";
    json += "\"Aceleracao_Y\":"    + String(aceleracao.acceleration.y) + ",";
    json += "\"Aceleracao_Z\":"    + String(aceleracao.acceleration.z) + ",";
    // Giroscopio
    json += "\"Giroscopio_X\":"    + String(giroscopio.gyro.x) + ",";
    json += "\"Giroscopio_Y\":"    + String(giroscopio.gyro.y) + ",";
    json += "\"Giroscopio_Z\":"    + String(giroscopio.gyro.z) + ",";
    // Intensidade da acelerção total
    json += "\"Acelerometro\":"    + String(AccMagnitude) + ",";
    // Velocidade angular
    json += "\"Giroscopio\":"      + String(WccMagnitude) + ",";
    // movimento
    json += "\"Movimento\":"       + String(movimento) + ",";
    json += "\"NivelMovimento\":"  + String(nivelMovimento) + ",";
    // Detectar queda
    json += "\"Possivel_Queda\":"  + String(PossivelQueda) + ","; // 1 = true ou 0 = false
    json += "\"Queda_Detectada\":"  + String(QuedaVerdadeira) + ","; // 1 = true ou 0 = false
    json += "\"Alerta\":"          + String(alertaAtivo) + ","; // 1 = true ou 0 = false
    // BPM e SpO2 valido ?
    json += "\"Spo2_Valido\":" + String(Spo2Valido) + ",";
    json += "\"BPM_Valido\":" + String(BPMvalido) + ",";
    // Sensores
    json += "\"SensorMAX30102_Coracao\":" + String(sensorBPMeSPO2Conectado) + ","; // 1 = true ou 0 = false
    json += "\"SensorMAX30205_Temperatura\":" + String(sensorTempConectado) + ","; // 1 = true ou 0 = false
    json += "\"SensorMPU6050_Queda\":" + String(sensorQuedaConectado) + ","; // 1 = true ou 0 = false
    // Outros
    json += "\"Data\":\""          + dataAtual() + "\",";
    json += "\"WiFiRSSI\":"        + String(WiFi.RSSI() + ",");
    json += "\"Tempo_Ligado\":" + String(millis()/1000);
    json += "}";
    // =========================
    // ENVIA
    // =========================
    int codigo = http.POST(json);
    Serial.println("\n========== API ==========");
    Serial.print("HTTP: ");
    Serial.println(codigo);
    Serial.println("JSON enviado:");
    Serial.println(json);
    Serial.println("=========================\n");
    http.end();
  } else {
    Serial.println("WiFi desconectado");
    wifi(tft.color565(255, 255, 255));
  }
}
//---------------- Mostrar dados no Serial e no Display ----------------//
void MostrarDados() {
  if (millis() - ultimoPrint > 500) {
    ultimoPrint = millis();
    Serial.println("===================================");
    // DEDO DETECTADO
    if (IR > 50000) {
      Serial.println("Pele encostada\n");
      ContatoSensor = 2;
    }
    // CONTATO FRACO
    else if (IR >= 10000 && IR < 50000) {
      Serial.println("Contato fraco com a pele\n");
      ContatoSensor = 1;
    }
    // SEM CONTATO
    else {
      Serial.println("Sem contato com a pele\n");
      ContatoSensor = 0;
    }
    // CARDIACO
    Serial.println("CARDIACO");

    Serial.print("BPM Atual: ");
    Serial.println(BatimentosPorMinuto);

    Serial.print("Media BPM: ");
    Serial.println(MediaBatimento);

    tft.setTextSize(3);
    tft.setTextColor(GC9A01A_WHITE, GC9A01A_BLACK);
    tft.setCursor(70, 100);
    tft.printf("%d   ", MediaBatimento);

    tft.setTextSize(2);
    tft.setTextColor(tft.color565(138, 0, 196));
    tft.setCursor(125, 100);
    tft.print("BPM");

    Serial.print("SPO2: ");

    if (Spo2Valido && BPMvalido) {

      Serial.print(Spo2);
      Serial.println("%");

      
      tft.setTextSize(2);
      tft.setTextColor(GC9A01A_WHITE, GC9A01A_BLACK);
      tft.setCursor(70, 150);
      tft.printf("%d  ", Spo2);

    } else {

      Serial.println("Invalido");
      
      tft.setTextSize(2);
      tft.setTextColor(GC9A01A_WHITE, GC9A01A_BLACK);
      tft.setCursor(65, 150);
      tft.print(" 0 ");
    }
    
    tft.setTextSize(2);
    tft.setTextColor(tft.color565(138, 0, 196));
    tft.setCursor(106, 147);
    tft.print("%");

    Serial.println("------------------------------");
    Serial.print("Temperatura: ");
    Serial.print(tempC, 2);
    Serial.println(" °C");

    tft.setTextSize(2);
    tft.setTextColor(GC9A01A_WHITE, GC9A01A_BLACK);
    tft.setCursor(160, 150);
    tft.printf("%.1f", tempC);

    tft.setTextSize(1);
    tft.setTextColor(tft.color565(138, 0, 196));
    tft.setCursor(210, 150);
    tft.print((char)247); 
    tft.print("C");

    // ACELERACAO
    Serial.println("\nACELERACAO");

    Serial.print("X: ");
    Serial.println(aceleracao.acceleration.x);

    Serial.print("Y: ");
    Serial.println(aceleracao.acceleration.y);

    Serial.print("Z: ");
    Serial.println(aceleracao.acceleration.z);

    // GIROSCOPIO
    Serial.println("\nGIROSCOPIO");

    Serial.print("X: ");
    Serial.println(giroscopio.gyro.x);

    Serial.print("Y: ");
    Serial.println(giroscopio.gyro.y);

    Serial.print("Z: ");
    Serial.println(giroscopio.gyro.z);

    // ESTADO DE MOVIMENTO
    Serial.println("\nESTADO");

    Serial.print("Movimento: ");
    Serial.println(movimento);

    Serial.print("Vel Angular: ");
    Serial.println(WccMagnitude);

    if (movimento < 1.5 &&
        WccMagnitude < 0.2) {

      Serial.println("Status: PARADO");
      
    } else if (movimento < 4) {

      Serial.println("Status: MOVIMENTO LEVE");   

    } else if (movimento < 10) {

      Serial.println("Status: ANDANDO");
      circulo(tft.color565(138, 0, 196));
    }

    if (PossivelQueda) {

      Serial.println("ALERTA: POSSIVEL QUEDA");
      alertaVisual();
      alerta();
    }

    Serial.println("===================================\n");
    
  }
}
