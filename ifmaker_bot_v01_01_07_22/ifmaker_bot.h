#include "ifmaker_bot_variaveis.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WiFi.h>
#include <WebSocketsServer.h>
#include <SPI.h>
#include <Wire.h>

#include <ArduinoJson.h> //https://arduinojson.org/v6/example/ (JSON generator)





#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
#define SDA_2 33
#define SCL_2 32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire1, OLED_RESET);




const char* ssid     = "IF-MAKERBOT";
const char* password = "123456788";

IPAddress local_IP(192, 168, 0, 110);
IPAddress gateway(192, 168, 0, 1);
IPAddress subnet(255, 255, 255, 0);


WebSocketsServer webSocket = WebSocketsServer(81);

StaticJsonDocument<1024> doc_tx;
StaticJsonDocument<1024> doc_rx;//Reserva 1Kb da RAM para JSON

//////////////////////////////////////////////////////////////////////////////////////////////////////
// Filtro passa-baixa genérico (1ª ordem)
// Exige uma memória de estado anterior
//////////////////////////////////////////////////////////////////////////////////////////////////////

float filtro_pb(float y, float y_ant) {
  float a = 0.1;
  return a * y + (1 - a) * y_ant;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
// Interrupções dos Encoders
//////////////////////////////////////////////////////////////////////////////////////////////////////

//Interrupção dos Encoders
void IRAM_ATTR Encoder_E_C1_ISR() {
  if (digitalRead(Encoder_E_C2)) {
    pulsosE_ODM++;
    pulsosE_PID++;
  }
  else {
    pulsosE_ODM--;
    pulsosE_PID--;
  }
}

void IRAM_ATTR Encoder_D_C1_ISR() {
  if (digitalRead(Encoder_D_C2)) {
    pulsosD_ODM++;
    pulsosD_PID++;
  }
  else {
    pulsosD_ODM--;
    pulsosD_PID--;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
// Evento Websocket
//////////////////////////////////////////////////////////////////////////////////////////////////////

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t lenght) {
  //payload
  switch (type) {
    case WStype_TEXT:
      String json_rx = (char *) payload;
      //Serial.println(txt);
      deserializeJson(doc_rx, json_rx);
      String cmd_2 =  doc_rx["cmd"];
      cmd = cmd_2;
      cmd_2 = " ";
      par1 = doc_rx["par1"];
      par2 = doc_rx["par2"];
      par3 = doc_rx["par3"];
      doc_rx = " ";
      json_rx = " ";
      /*Serial.print("CMD: ");
        Serial.print(cmd);
        Serial.print(" PAR1: ");
        Serial.print(par1);
        Serial.print(" PAR2: ");
        Serial.print(par2);
        Serial.print(" PAR3: ");
        Serial.println(par3);*/
      // ws_msg = String((char *) &payload[0]);

      //TODO
      break;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
// Odometria
//////////////////////////////////////////////////////////////////////////////////////////////////////

void atualizarOdometria() {
  float Qe_ODM = (float) (pulsosE_ODM - lastPulsosE_ODM) / dt_ODM;
  we_ODM = (float) Qe_ODM * 2.0 * M_PI / N;

  float Qd_ODM = (float) (pulsosD_ODM - lastPulsosD_ODM) / dt_ODM;
  wd_ODM = (float) Qd_ODM * 2.0 * M_PI / N;

  lastPulsosE_ODM = pulsosE_ODM;
  lastPulsosD_ODM = pulsosD_ODM;

  phi_ODM += R / L * (wd_ODM - we_ODM) * dt_ODM;
  x_ODM += R / 2 * (wd_ODM + we_ODM) * cos(phi_ODM) * dt_ODM;
  y_ODM += R / 2 * (wd_ODM + we_ODM) * sin(phi_ODM) * dt_ODM;

  phi_ODM = atan2(sin(phi_ODM), cos(phi_ODM));
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
// Display
//////////////////////////////////////////////////////////////////////////////////////////////////////

void iniciarDISPLAY() {
  display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
  display.display();
}
//
void printDisplay(String msg) {
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.println("IFMAKERBOT");  // Display static text
  display.setCursor(0, 20);
  // Display static text
  display.println(msg);
  display.display();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
// Inicialização I2C
//////////////////////////////////////////////////////////////////////////////////////////////////////

void iniciarI2C() {
  Wire1.begin(SDA_2, SCL_2);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
// Inicialização I/Os
//////////////////////////////////////////////////////////////////////////////////////////////////////

void iniciarIO() {
  pinMode(ECHO, INPUT);
  pinMode(TRIG, OUTPUT);
  digitalWrite(TRIG, LOW);

  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);
  pinMode(BUZZER, OUTPUT);

  pinMode(LED_BOARD_PIN, OUTPUT);

  pinMode(Motor_E_C1, OUTPUT);
  pinMode(Motor_E_C2, OUTPUT);
  pinMode(Motor_D_C1, OUTPUT);
  pinMode(Motor_D_C2, OUTPUT);

  int freq = 25000; //Frequência PWM
  int res = 10;     // Resolução PWM - 10 bits

  ledcSetup(0, freq, res);
  ledcSetup(1, freq, res);
  ledcSetup(2, freq, res);
  ledcSetup(3, freq, res);

  ledcAttachPin(Motor_E_C1, 0);
  ledcAttachPin(Motor_E_C2, 1);
  ledcAttachPin(Motor_D_C1, 2);
  ledcAttachPin(Motor_D_C2, 3);

  pinMode(Encoder_E_C1, INPUT);
  pinMode(Encoder_E_C2, INPUT);

  pinMode(Encoder_D_C1, INPUT);
  pinMode(Encoder_D_C2, INPUT);

  attachInterrupt(Encoder_E_C1, Encoder_E_C1_ISR, RISING);
  attachInterrupt(Encoder_D_C1, Encoder_D_C1_ISR, RISING);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
// Iniciar WiFi
//////////////////////////////////////////////////////////////////////////////////////////////////////

void iniciarWIFI() {
  // Conexões wi-fi

  WiFi.begin(ssid, password);

  WiFi.config(local_IP, gateway, subnet);

  for (int k = 0; k <= 20; k++) { //Animação obrigatória do LED
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(50);
  }

  while (WiFi.status() != WL_CONNECTED) {
    delay(50);
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }

  digitalWrite(LED_BOARD_PIN, 1); //Led indicador da placa aceso após conexão

  //Websocket
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
// Ultrassônico
//////////////////////////////////////////////////////////////////////////////////////////////////////

void lerDistUltrassom() {
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(ECHO, HIGH);
  // Calculate the distance
  distanceCm = duration * SOUND_SPEED / 2;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
// Comandos de alto nível para os controladores do robô
//////////////////////////////////////////////////////////////////////////////////////////////////////
void reiniciar_odometria() {
  x_ODM = 0.0;
  y_ODM = 0.0;
  phi_ODM = 0.0;

  pulsosE_ODM = 0;
  pulsosD_ODM = 0;

  lastPulsosE_ODM = 0;
  lastPulsosD_ODM = 0;
}

void reiniciar_controladores_pid() {
  ref_we = 0.0;
  ref_wd = 0.0;

  setpoint_E = 0.0; //setpoint
  setpoint_D = 0.0;

  erro_D = 0.0; //erro
  erro_E = 0.0;

  erro_D_int = 0.0; //integral do erro
  erro_E_int = 0.0;

  last_erro_D = 0.0;//derivada do erro
  last_erro_E = 0.0;

  acaoD = 0.0; //ações de controle
  acaoE = 0.0;

  pulsosD_PID = 0.0;
  pulsosE_PID = 0.0;

  lastPulsosE_PID = 0;
  lastPulsosD_PID = 0;
}

void pararMotor(char M); //Protótipo

void parar_robo() {
  pararMotor('e');
  pararMotor('d');
  delay(500);
  reiniciar_odometria();
  reiniciar_controladores_pid();
  isBusy = 0;
  cmd = " ";
  par1 = 0;
  par2 = 0;
  par3 = 0;
  pararMotor('e');
  pararMotor('d');
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
// Controladores de alto nível
//////////////////////////////////////////////////////////////////////////////////////////////////////

void controladorMotores(); //Protótipo

void controladorLinear(int sentido, int pos_x, int vel) {
  float u_max = 50; //Limitado a 60 cm/s

  float u = 0.0;

  if (sentido == 0) { //Frente - sentido positivo
    u = vel / 100.0 * u_max;
  }
  else { //Trás - sentido negativo
    u = -vel / 100.0 * u_max;
  }

  int pos_y = 0;

  float phid = atan2((pos_y - y_ODM), (pos_x - x_ODM));
  float erro = phid - phi_ODM;
  erro = atan2(sin(erro), cos(erro));
  float rho = sqrt(((pos_y - y_ODM) * (pos_y - y_ODM)) + ((pos_x - x_ODM) * (pos_x - x_ODM)));
  Serial.println(rho);
  if (rho >= 5.0) {
    float w = 1.5 * erro;
    ref_wd = (2 * u + w * L) / (2 * R);
    ref_we = (2 * u - w * L) / (2 * R);
  }
  else {
    parar_robo();
  }
}

void controladorGoalToGoal(int pos_x, int pos_y) {

  float u = 10.0; //Limite de 60 cm/s

  float phid = atan2((pos_y - y_ODM), (pos_x - x_ODM));
  float erro = phid - phi_ODM;
  erro = atan2(sin(erro), cos(erro));
  float rho = sqrt(((pos_y - y_ODM) * (pos_y - y_ODM)) + ((pos_x - x_ODM) * (pos_x - x_ODM)));

  if (rho >= 1.0) {
    float w = 1.5 * erro;
    ref_wd = (2 * u + w * L) / (2 * R);
    ref_we = (2 * u - w * L) / (2 * R);
  }
  else {
    parar_robo();
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
// Atuação nos motores
//////////////////////////////////////////////////////////////////////////////////////////////////////

void comandarMotor(char M, int duty) {
  if (M == 0 || M == 'E' || M == 'e') {
    if (duty == 0) {
      ledcWrite(0, duty);
      ledcWrite(1, duty);
    }
    else if (duty < 0) {
      ledcWrite(0, -duty);
      ledcWrite(1, 0);
    }
    else if (duty > 0) {
      ledcWrite(0, 0);
      ledcWrite(1, duty);
    }
  }
  else if (M == 1 || M == 'D' || M == 'd') {
    if (duty == 0) {
      ledcWrite(2, duty);
      ledcWrite(3, duty);
    }
    else if (duty < 0) {
      ledcWrite(2, -duty);
      ledcWrite(3, 0);
    }
    else if (duty > 0) {
      ledcWrite(2, 0);
      ledcWrite(3, duty);
    }
  }
}

void pararMotor(char M) {
  comandarMotor(M, 0);
}


//////////////////////////////////////////////////////////////////////////////////////////////////////
// Controlador de baixo nível (motores)
//////////////////////////////////////////////////////////////////////////////////////////////////////

void controladorMotores() {
  float Qe_PID = (float) (pulsosE_PID - lastPulsosE_PID) / dt_PID;
  we_PID = (float) Qe_PID * 2.0 * M_PI / N;
  we_PID = filtro_pb(we_PID, we_PID_ant);
  we_PID_ant = we_PID;

  float Qd_PID = (float) (pulsosD_PID - lastPulsosD_PID) / dt_PID;
  wd_PID = (float) Qd_PID * 2.0 * M_PI / N;
  wd_PID = filtro_pb(wd_PID, wd_PID_ant);
  wd_PID_ant = wd_PID;

  lastPulsosD_PID = pulsosD_PID;
  lastPulsosE_PID = pulsosE_PID;

  erro_D = ref_wd - wd_PID;
  erro_E = ref_we - we_PID;

  acaoD = erro_D * kp + erro_D_int * dt_PID * ki + (erro_D - last_erro_D) / dt_PID * kd;
  acaoE = erro_E * kp + erro_E_int * dt_PID * ki + (erro_E - last_erro_E) / dt_PID * kd;

  last_erro_D = erro_D;
  last_erro_E = erro_E;

  erro_D_int += erro_D;
  erro_E_int += erro_E;

  if (erro_D_int > 2000)  {
    erro_D_int = 2000;
  }
  else if (erro_D_int < -2000)  {
    erro_D_int = -2000;
  }

  if (erro_E_int > 2000)  {
    erro_E_int = 2000;
  }
  else if (erro_E_int < -2000)  {
    erro_E_int = -2000;
  }

  if (acaoE > 1023) {
    acaoE = 1023;
  }
  else if (acaoE < -1023) {
    acaoE = -1023;
  }

  if (acaoD > 1023) {
    acaoD = 1023;
  }
  else if (acaoD < -1023) {
    acaoD = -1023;
  }

  comandarMotor('e', acaoE);
  comandarMotor('d', acaoD);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
// Interpreta comandos recebidos via Websocket
//////////////////////////////////////////////////////////////////////////////////////////////////////

void comandosWebsocket() {
  if (cmd == "a") {
    parar_robo();
    reiniciar_odometria();
    reiniciar_controladores_pid();

    printDisplay("Robo parado!");
  }
  else if (cmd == "b") { //Determinar w_max
    comandarMotor('e', 1023);
    comandarMotor('d', 1023);
    printDisplay("Frente 100%");
  }
  else if (cmd == "c") { //Determinar -w_max
    comandarMotor('e', -1023);
    comandarMotor('d', -1023);
    printDisplay("Tras 100%");
  }
  else if (cmd == "g") {
    //    controladorLinear(par1, par2, par3) ;
    isBusy = 1;
  }
  else if (cmd == "o") {
    printDisplay("Led e Buzzer ON");
    digitalWrite(LED_R, HIGH);
    digitalWrite(LED_G, HIGH);
    digitalWrite(LED_B, HIGH);
    digitalWrite(BUZZER, HIGH);
  }
  else if (cmd == "p") {
    printDisplay("Led e Buzzer OFF");
    digitalWrite(LED_R, LOW);
    digitalWrite(LED_G, LOW);
    digitalWrite(LED_B, LOW);
    digitalWrite(BUZZER, LOW);
  }

}

//////////////////////////////////////////////////////////////////////////////////////////////////////
// Atualização de JSON para Websocket
//////////////////////////////////////////////////////////////////////////////////////////////////////

void atualizarJSON() {
  static int varCount = 0;

  doc_tx["Count"] = varCount;
  doc_tx["x"] = x_ODM;
  doc_tx["y"] = y_ODM;
  doc_tx["phi"] = phi_ODM;
  doc_tx["Dist"] = distanceCm;
  doc_tx["isBusy"] = isBusy;



  String JSONtxt;

  serializeJsonPretty(doc_tx, JSONtxt);

  webSocket.sendTXT(0, JSONtxt);

  varCount++;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////
// Função de sintonia PID das rodas
//////////////////////////////////////////////////////////////////////////////////////////////////////

void sintoniaPID() {
  static unsigned int tt = 0;

  kp = 450.0; //450
  ki = 350.0; //350
  kd = 25.0;  //25

  tt++;
  if (tt >= 0 && tt <= 1000) {
    ref_we = 5.0;
    ref_wd = 5.0;
  }
  else if (tt >= 1000 && tt <= 2000) {
    ref_we = 1.5;
    ref_wd = 1.5;
  }
  else if (tt >= 2000 && tt <= 3000) {
    ref_we = -5.5;
    ref_wd = -5.5;
  }
  else if (tt >= 3000 && tt <= 4000) {
    ref_we = 0.0;
    ref_wd = 0.0;
  }
  Serial.print((String) ref_we + " " + we_PID + "\n");
}
