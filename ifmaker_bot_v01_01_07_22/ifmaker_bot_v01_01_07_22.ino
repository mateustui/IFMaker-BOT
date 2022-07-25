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

#include "ifmaker_bot.h"

void setup() {
  Serial.begin(250000); //Plot gráfico e sintonia PID
  iniciarIO();
  iniciarWIFI();
  iniciarI2C();
  iniciarDISPLAY();//Parou de funcionar
  printDisplay("  ");
}

void loop() {
  static unsigned long int TimeStamp100ms = 0;
  static unsigned long int TimeStamp10ms = 0;

  webSocket.loop();

  comandosWebsocket();

  // Janela de autalização dos motores: 10ms
  if (millis() - TimeStamp10ms >= 10) {
    TimeStamp10ms = millis();

    if (isBusy) {
      //sintoniaPID();
      controladorMotores();
    }
  }

  // Janela de autalização dos sensores: 200ms
  if (millis() - TimeStamp100ms >= 100) {
    TimeStamp100ms = millis();
    lerDistUltrassom();
    atualizarOdometria();
    if (isBusy) {
      //comandosWebsocket();
      controladorLinear(par1, par2,par3) ;
//      controladorGoalToGoal(120, 60);
    }
    atualizarJSON();
  }
}
