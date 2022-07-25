

/*////////////////////////////////////////////////////////////////////////////////////////
  Limites de operação - Testados experimentalmente
  Velocidade limite das rodas:
  we_max = 22 rad/s
  wd_max = 21 rad/s
  ==>> 20 rad/s
  Velocidade linear máxima do robô:
  u_max = R/2 * (we_max + wd_max)
        = 3.23/2 * 40
        = 64 cm/s
  ==>> 60 cm/s
  w_max = R/L * (wd_max - we_max)
        = 3.23/17.3 * (20 - -20)
        = 7.46 rad/s
  ==>> 7 rad/s
  ///////////////////////////////////////////////////////////////////////////////////////*/

//////////////////////////////////////////////////////////////////////////////////////////////////////
// String de comando WebSocket
//////////////////////////////////////////////////////////////////////////////////////////////////////
String  cmd= " ";
int par1 = 0;
int par2 = 0;
int par3 = 0;
//////////////////////////////////////////////////////////////////////////////////////////////////////
// Configurações dos I/Os
//////////////////////////////////////////////////////////////////////////////////////////////////////

#define LED_BOARD_PIN 2 //Led da placa ESP32

#define TRIG 5 //Trigger ultrassônico
#define ECHO 18 //Echo ultrassônico

#define SDA_2 33 // Alteração de pinos do I2C da ESP32
#define SCL_2 32 // Alteração de pinos do I2C da ESP32

#define LED_R 27 //Pinos LEDs 
#define LED_G 14 //Pinos LEDs
#define LED_B 12 //Pinos LEDs 
#define BUZZER 13 //Pinos BUZZER

#define Encoder_E_C1 26 //25-Branco
#define Encoder_E_C2 25 //26-Amarelo
#define Encoder_D_C1 35 //34-Amarelo
#define Encoder_D_C2 34 //35-Branco

#define Motor_E_C1 19 // OK - sem PWM no boot
#define Motor_E_C2 21 // OK - sem PWM no boot
#define Motor_D_C1 16 // OK - sem PWM no boot
#define Motor_D_C2 17 // OK - sem PWM no boot

//////////////////////////////////////////////////////////////////////////////////////////////////////
// Variáveis de controle do fluxo do código
//////////////////////////////////////////////////////////////////////////////////////////////////////
int isBusy = 0;

//////////////////////////////////////////////////////////////////////////////////////////////////////
// Variáveis dos controladores PID das rodas
//////////////////////////////////////////////////////////////////////////////////////////////////////
float ref_wd = 0.0;
float ref_we = 0.0;

float dt_PID = 0.01;

float we_PID = 0.0;
float wd_PID = 0.0;

float we_PID_ant = 0.0;
float wd_PID_ant = 0.0;

float setpoint_E = 0.0; //setpoint
float setpoint_D = 0.0;

float erro_D = 0.0; //erro
float erro_E = 0.0;

float erro_D_int = 0.0; //integral do erro
float erro_E_int = 0.0;

float last_erro_D = 0.0;//derivada do erro
float last_erro_E = 0.0;

float acaoD = 0.0; //ações de controle
float acaoE = 0.0;

int pulsosD_PID = 0.0;
int pulsosE_PID = 0.0;

int lastPulsosE_PID = 0;
int lastPulsosD_PID = 0;

float kp = 450.0;
float ki = 350.0;
float kd = 25.0;

//////////////////////////////////////////////////////////////////////////////////////////////////////
// Configurações do sensor ultrassônico
//////////////////////////////////////////////////////////////////////////////////////////////////////
#define SOUND_SPEED 0.034
#define CM_TO_INCH 0.393701

long duration = 0;
int distanceCm = 0;

//////////////////////////////////////////////////////////////////////////////////////////////////////
// Parâmetros do robô e do encoder
// Datasheet do encoder:
// https://pt.aliexpress.com/item/32842600463.html?spm=a2g0o.store_pc_allProduct.8148356.20.58904243NpgLZF&pdp_npi=2%40dis%21BRL%21R%24%2059%2C40%21R%24%2059%2C40%21%21%21%21%21%402101e9cf16563572076107141e077f%2165151901508%21sh
//////////////////////////////////////////////////////////////////////////////////////////////////////

float N = 374.22; // Resolução encoder: 374,22 pulsos/revolução
float R = 3.23;   // Raio da roda: 3.3 cm
float L = 17.3;  // Distância entre-rodas: 18.5 cm

//////////////////////////////////////////////////////////////////////////////////////////////////////
// Variáveis da ODOMETRIA
//////////////////////////////////////////////////////////////////////////////////////////////////////
float dt_ODM = 0.1;

float we_ODM = 0.0;
float wd_ODM = 0.0;

int pulsosE_ODM = 0; //Contador encoder Esquerda
int pulsosD_ODM = 0; //Contador encoder Direita

int lastPulsosE_ODM = 0; //Memória auxiliar usada no cálculo de velocidade angular da roda esquerda
int lastPulsosD_ODM = 0; //Memória auxiliar usada no cálculo de velocidade angular da roda esquerda

float x_ODM = 0.0;
float y_ODM = 0.0;
float phi_ODM = 0.0;



/*********************************************************************
  Configuração dos GPIOS (ESP32)
  Referência:
  https://randomnerdtutorials.com/esp32-pinout-reference-gpios/
  Planilha de GPIOs livres (Fonte: https://youtu.be/LY-1DHTxRAk )
  No.  Name  Type  GPIO  True GPIO
  26  IO04  I/O GPIO04
  29  IO05  I/O GPIO05
  16  IO13  I/O GPIO13
  13  IO14  I/O GPIO14
  23  IO15  I/O GPIO15
  30  IO18  I/O GPIO18
  31  IO19  I/O GPIO19
  33  IO21  I/O GPIO21
  36  IO22  I/O GPIO22
  37  IO23  I/O GPIO23
  10  IO25  I/O GPIO25
  11  IO26  I/O GPIO26
  12  IO27  I/O GPIO27
  8  IO32  I/O GPIO32
  9  IO33  I/O GPIO33

  ----------------------------------------------------------
  ------------------REDES  LUCAS----------------------------
  ----------------------------------------------------------
  const char* ssid     = "IF-MAKERBOT";
  const char* password = "123456788";

  //ws://192.168.0.110:81
  IPAddress local_IP(192, 168, 0, 110);
  IPAddress gateway(192, 168, 0, 1);
  IPAddress subnet(255, 255, 255, 0);

  const char* ssid     = "LVS_ABR_LOGA";
  const char* password = "abcd1234";

  //ws://192.168.1.10:81
  IPAddress local_IP(192, 168, 1, 10);
  IPAddress gateway(192, 168, 1, 1);
  IPAddress subnet(255, 255, 255, 0);
*********************************************************************/
