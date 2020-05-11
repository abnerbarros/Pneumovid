/*
 * Versão patra a demonstração
 */
#include <TimerOne.h>
#include <Servo.h>
#define PINPOTFLUX 0
#define PINPOTFREQ 1
#define PINVELINS 2
#define SOLENOIDE_INS 13
#define SOLENOIDE_EXP 10 
#define ABRE 1
#define FECHA 0
#define PINPRESSAOINS 1
#define PINPRESSAOVAZAO 0
#define PINPRESSAOPEEP 2
#define CNT_TIMER_1SEG 100
#define TEMPOGUARDA 25
#define TAMANHOBUFFERPRESSAOANALISE 32
#define LIMITEBUFFERPRESSAOANALISE (TAMANHOBUFFERPRESSAOANALISE-1)
//#define LIGA_INS() ligaINS=1; 
#define LIGA_INS() medVolume = 1;\
                   tmpGuarda = TEMPOGUARDA;\
                   estadoINS = 1;\
                   digitalWrite(SOLENOIDE_INS,ABRE); 
 
#define DESLIGA_INS() medVolume = 0;\
                      tmpGuarda = TEMPOGUARDA;\
                      estadoINS = 0;\
                      digitalWrite(SOLENOIDE_INS,FECHA);
                      
#define LIGA_EXP() tmpGuarda = TEMPOGUARDA;\
                   estadoEXP = 1;\
                   digitalWrite(SOLENOIDE_EXP,ABRE);  
                   
#define DESLIGA_EXP() desligaEXP=0;\ 
                      tmpGuarda = TEMPOGUARDA;\
                      estadoEXP = 0;\
                      digitalWrite(SOLENOIDE_EXP,FECHA);
                      

     

enum COMANDOS{
  VCV = 0x01,
  PCV = 0x02,
  PSV = 0x04,
  CALIBR = 0x08,
  STOP = 0x00
};

enum PTRDATA{
  COMANDO = 0,
  VOLUME = 1,
  FREQUENCIA = 2,
  PAUSA = 3,
  FLUXO = 4,
  PIP = 5,
  PEEP = 6,
  TINS = 7,
  TSUB = 8,
  RELACAOIE = 9,
  SPRESS = 10,
  SFLUX = 11,
  PRESSAOSUPORTE =12,
  TEMPOAPNEIA = 13  
};


enum RETORNOS{
  PtrPRESSAO = 0,
  PtrVAZAO = 1,
  PtrVOLUME = 2,
  PtrTINSP = 3,
  PtrTEXP = 4,
  PtrPRESSAOPICO =  5,
  PtrVOLUMEUCICLO = 6, 
  PtrRELACAOIE = 7, 
  PtrPRESSAOPLATO = 8, 
  PtrFLAGS = 9
};

float relogio;
float bufferPressao[16];
float pressaoAnaliseInicio, pressaoAnaliseFim;
float fmediaPressaoBuffer;
float pressaoAnaliseMaxInicio, pressaoAnaliseMaxFim, pressaoAnaliseMinima, pressaoAnaliseMaxima, pressao;
float limitePressaoTesteSuperior, limitePressaoTesteInferior;
float  pressaoMedia;
float pressaoT[16];
float mediaInicio, mediaFim;
float pressaoAnalise[TAMANHOBUFFERPRESSAOANALISE];
float vazao[5];
float bufferTrabalhoPSV[25];
float dffPresao;
float pressaoSuporte, sensibilidadePressao, pressaoSuporte_prog, sensibilidadePressao_prog;
float MedVel, fluxoMedido; 
float fVazaoMedia;
float  iDP;
volatile float fVazao, fVolume, fPressao;
volatile float passoPressao, erroPressao, dffPressao, inclinacaoCurvaPressao;

int cntTmpLimiteINS, cntTmpLimitePEEP, cntTmpPressaoLimiteInspirar;
int tempoInsCiclo, tempoExpCiclo, pressoaPicoCiclo, volumeCiclo, relacaoIECiclo, pressaoPlatoCiclo, flagsCiclo;
int tempoInsUCiclo, tempoExpUCiclo, pressoaPicoUCiclo, volumeUCiclo, relacaoIEUCiclo, pressaoPlatoUCiclo, flagsUCiclo;
int receiveData[64];// = {0,0,0,0,0,0,0,0,0,0,0,0,0,0};
int sendData[] = {0,0,0,0,0,0,0,0,0,0};
int ptrPressaoMaxInicio, ptrPressaoMaxFim, ptrPressaoMinima, ptrPressaoMaxima;
int ptrRDbufferPressao, ptrWRbufferPressao, ptrRDpressaoAnalise, ptrWRpressaoAnalise;
int tmpAnaliseInspirar, estAnaliseInspirar;
int ptrWrBufferPressaoT, ptrRdBufferPressaoT;
int tamanhoCurva;
int FreqResp, Fluxo, PotFreq, PotFlux, PWM_ValvulaIns, PWM_ValvulaExp;
int Ton, Toff, TCiclo;
int tTMP;
int vazaoIntantanea;
int cntT_INS, cntT_EXP, cntT_PAUSA_INS, cntT_APNEIA;
int i_PRESS_INS_REF, i_PRESS_PEEP_REF, i_PRESS_VAZAO_REF;
int i, j, tempoExecucao, iVolumeINSP;
int cntTMP2;
int tmpCOM;
int estTestInsp, estCicloIsp, estCicloMedFluxo, estCicloTestarVazao, estCOM, estVCV, estPCV, estPSV, estTST, estCALIB, estCOMM, estPVCV;
int tempoEXP, volumeMaxINS, tempoCiclo, tempoApneia, pressaoINS, pressaoPEEP, tempoINS, rpm, tempoPausaINS, tempoEXP_prog, volumeMaxINS_prog, tempoCiclo_prog, tempoApneia_prog, pressaoINS_prog, pressaoPEEP_prog, tempoINS_prog, rpm_prog, tempoPausaINS_prog;
int cmdEmExecucao;
int nDadosBuffer, cntInclinacao;
int tmpCom2, tmpCom3, dadosDisponiveisCOM, tmpGuarda;
int est[10];
int cntResult, qtdEscritasBuffer, cntTmpPressaoSuporte, cntTmpPressaoLimiteInspirarPSV;
volatile int pressaoT1, cntTMP, cntTmpPressao, cntTmpVazao, TIMER10MS, CLOCK10MS;

char dado;
char dadoCom2;

volatile boolean ligaINS, ligaEXP;
boolean encher, parar, enchendo, testar, validar, inspirar, cmdVCV, cmdPCV, cmdPSV, cmdTST,  desligaINS, desligaEXP, cmdCALIB, medVolume, estadoINS, estadoEXP, oldEstadoINS, oldEstadoEXP;
boolean pressaoLimiteINS,pressaoLimitePEEP, pressaoLimiteINSTMP,pressaoLimitePEEPTMP,pressaoLimiteInspirar,  monitoraInspirar, pressaoLimiteSuporte, pressaoLimiteInspirarPSV, tempo;

void setup() {
  // initialize both serial ports:
  pinMode(SOLENOIDE_INS,OUTPUT);
  pinMode(SOLENOIDE_EXP,OUTPUT);
  pinMode(9,OUTPUT);
  pinMode(8,OUTPUT);
  pinMode(2,OUTPUT);
  DESLIGA_INS();
  DESLIGA_EXP();
  Timer1.initialize(10000); // uma interrupção a cada 10 ms
  Timer1.attachInterrupt(timerISR);
  Serial.begin(115200);
  Serial2.begin(115200);
  delay(1000);
  //Serial2.print("0");
  i_PRESS_INS_REF = 36;//analogRead(PINPRESSAOINS);
  i_PRESS_PEEP_REF = 460;//analogRead(PINPRESSAOPEEP);
  i_PRESS_VAZAO_REF = 205;//analogRead(PINPRESSAOVAZAO);
  encher = 0;
  enchendo = 0;
  parar=0;
  testar = 0;
  validar = 0;
  inspirar = 0;
  cntTMP = 0;
  estTestInsp = 0;
  estCOM = 0;
  estCALIB = 0;
  estCOMM = 0;
  iVolumeINSP = 500;
  ptrRDbufferPressao = 0;
  ptrWRbufferPressao = 0;
  LIGA_EXP();
  qtdEscritasBuffer= 0;
}


//void serialEvent() {
//  while (Serial.available()) {
//    // get the new byte:
//    char inChar = (char)Serial.read();
//    // add it to the inputString:
//    inputString += inChar;
//    // if the incoming character is a newline, set a flag so the main loop can
//    // do something about it:
//    if (inChar == '\n') {
//      stringComplete = true;
//    }
//  }
//}


void timerISR(){
  CLOCK10MS = 1;
}


void loop() {

      
    dadosDisponiveisCOM = Serial.available();
    switch (estCOMM){
      case 0: if (dadosDisponiveisCOM)
                 if (dadosDisponiveisCOM==28)
                   estCOMM = 3;
                 else if (dadosDisponiveisCOM < 28){
                            estCOMM = 1;
                            tmpCom3 = 100;//aguarda 1 segundo
                      }
            break;
      case 1: if (dadosDisponiveisCOM==28) //fica monitorando se chega 28 bytes 
                 estCOMM = 3;
              else if (!tmpCom3)
                      estCOMM = 2;   
            break;
      case 2: Serial.readBytes((char*)receiveData, dadosDisponiveisCOM);  // Se ´pós 1 segundo não chegarem 28 bytes, entra na rotia de erro de comunicação
              //Serial2.println("FALHA"); 
              for (i=0; i< dadosDisponiveisCOM; i++){
                 //Serial2.print(receiveData[i]);
                 //Serial2.print(" ");   
              }
              //Serial2.print(dadosDisponiveisCOM);
              //Serial2.println(" ");  
              estCOMM = 0;
              break;
       case 3:Serial.readBytes((char*)receiveData, 14*sizeof(int)); // Se chegam os 28 bytes, procede a decodificação do comando
              estCOMM = 0;
              switch(receiveData[0]) {
                  case 0x0030:// STOP
                                cmdEmExecucao = STOP;
                                parar = 1;
                                DESLIGA_INS();
                                LIGA_EXP();
                                estCicloTestarVazao = 0; 
                                estCicloMedFluxo = 0;
                                estTestInsp = 0;
                                cntTmpPressao = 0;
                                cntTmpVazao = 0;
                                cntTMP = 0;
                                validar = 0;
                                testar = 0;
                                encher = 0;
                                cmdVCV = 0;
                                estPVCV = 0;
                                estVCV = 0;
                                estPCV = 0;
                                estPSV = 0;
                                estCALIB = 0;
                                Serial2.println("STOP"); 
                                break;
                  case 0x0031: // CALIB
                                cmdEmExecucao = CALIBR;
                                cmdCALIB= 1;
                                estVCV = 0;
                                estPCV = 0;
                                estPSV = 0;
                                //Serial2.println("CALIBR:"); 
                                break;
                  case 0x0032:// VCV
                                cmdEmExecucao = VCV;
                                volumeMaxINS_prog =  receiveData[VOLUME];
                                rpm_prog = receiveData[FREQUENCIA];
                                pressaoINS_prog = receiveData[PIP];
                                pressaoPEEP_prog = receiveData[PEEP];
                                tempoINS_prog = receiveData[TINS] * CNT_TIMER_1SEG;
                                tempoPausaINS_prog = receiveData[PAUSA] * CNT_TIMER_1SEG;
                                tempoCiclo_prog = 60*CNT_TIMER_1SEG/rpm_prog;
                                tempoEXP_prog = tempoCiclo_prog-tempoINS_prog;
                                sensibilidadePressao_prog = receiveData[SPRESS];
                                if (sensibilidadePressao_prog<0)
                                   sensibilidadePressao_prog = -sensibilidadePressao_prog;
                                Serial2.println("VCV"); 
                                
                                 break;
                  case 0x0033:// PCV
                                cmdEmExecucao = PCV;
                               volumeMaxINS_prog =  receiveData[VOLUME];
                                rpm_prog = receiveData[FREQUENCIA];
                                pressaoINS_prog = receiveData[PIP];
                                pressaoPEEP_prog = receiveData[PEEP];
                                tempoINS_prog = receiveData[TINS] * CNT_TIMER_1SEG;
                                tempoPausaINS_prog = receiveData[PAUSA] * CNT_TIMER_1SEG;
                                tempoCiclo_prog = 60*CNT_TIMER_1SEG/rpm_prog;
                                tempoEXP_prog = tempoCiclo_prog-tempoINS_prog;
                                sensibilidadePressao_prog = receiveData[SPRESS];
                                if (sensibilidadePressao_prog<0)
                                   sensibilidadePressao_prog = -sensibilidadePressao_prog;
                                Serial2.println("PCV"); 
                                Serial2.print(" tempoEXP_prog:"); Serial2.println(tempoEXP_prog);
                                break;
                  case 0x0034: // PSV
                                cmdEmExecucao = PSV;
                                volumeMaxINS_prog =  receiveData[VOLUME];
                                rpm_prog = receiveData[FREQUENCIA];
                                pressaoINS_prog = receiveData[PIP];
                                pressaoPEEP_prog = receiveData[PEEP];
                                tempoINS_prog = receiveData[TINS] * CNT_TIMER_1SEG;
                                tempoPausaINS_prog= receiveData[PAUSA] * CNT_TIMER_1SEG;
                                tempoCiclo_prog = 60*CNT_TIMER_1SEG/rpm_prog;
                                tempoEXP_prog = tempoCiclo_prog-tempoINS_prog;
                                tempoApneia_prog = receiveData[TEMPOAPNEIA]*CNT_TIMER_1SEG;
                                pressaoSuporte_prog = receiveData[PRESSAOSUPORTE];
                                sensibilidadePressao_prog = receiveData[SPRESS];
                                if (sensibilidadePressao_prog<0)
                                   sensibilidadePressao_prog = -sensibilidadePressao_prog;
                                break;
                   default:
                                break;
                }
              
              break;       
    }
    

    
    switch (estCOM){
         case 0: if (cmdEmExecucao){
                    tmpCOM = 2; //  timer de 50ms
                    estCOM=1;
                }
                break;
         case 1: if (!tmpCOM){
                    sendData[PtrPRESSAO]= (int) (fPressao*10);
                    sendData[PtrVAZAO]= (int) (fVazaoMedia*10);
                    sendData[PtrVOLUME]= (int) (fVolume*10);
                    sendData[PtrTINSP]= tempoInsUCiclo;
                    sendData[PtrTEXP]= tempoExpUCiclo;
                    sendData[PtrPRESSAOPICO]= pressoaPicoUCiclo*10;
                    sendData[PtrVOLUMEUCICLO]= volumeUCiclo;
                    sendData[PtrRELACAOIE]= relacaoIEUCiclo;
                    sendData[PtrPRESSAOPLATO]= pressaoPlatoUCiclo*10;
                    sendData[PtrFLAGS]= flagsUCiclo;
                    Serial.write((char*)&sendData, 10 * sizeof(int));
                    estCOM=0;
                }
                break;
       }
//
//  if (!tmpCom2){
//    if (cmdEmExecucao){
//      Serial2.print(fPressao,3);
//      Serial2.print(", ");
//    }
//    tmpCom2=1;
//  }
//
//(cmdEmExecucao)&&
//(inspirar)||
if ((est[3]!=estPVCV)||
    (est[0]!=estVCV)||
    (est[1]!=estPCV)||
    (est[2]!=estPSV) || 
    (estadoINS!=oldEstadoINS) || 
    (estadoEXP != oldEstadoEXP)){ // monitora e relata a mudança nos estados das máquinas de estado do sistema
                           Serial2.print(" cmdEmExecucao:"); Serial2.print(cmdEmExecucao);
                           Serial2.print(" estPVCV:"); Serial2.print(estPVCV);
                           Serial2.print(" estVCV:"); Serial2.print(estVCV);  
                           Serial2.print(" estPCV:"); Serial2.print(estPCV); 
                           Serial2.print(" estPSV:"); Serial2.print(estPSV);
                           Serial2.print(" estadoINS:"); Serial2.print(estadoINS);
                           Serial2.print(" estadoEXP:"); Serial2.print(estadoEXP);
                           Serial2.print(" fPressao:"); Serial2.print(fPressao);
                           Serial2.print(" cntTMP:"); Serial2.print(cntTMP);
                           Serial2.print(" fVolume:"); Serial2.print(fVolume);
                           Serial2.print(" Relogio: "); Serial2.print(relogio, 3);
                           Serial2.print(" qtdEscritasBuffer:"); Serial2.print(qtdEscritasBuffer); 
                           Serial2.print(" inspirar:"); Serial2.print(inspirar); 
                            
                           est[0]=estVCV; est[1]=estPCV; est[2]=estPSV; est[3]=estPVCV;
                           oldEstadoINS = estadoINS;
                           oldEstadoEXP = estadoEXP; 
                           Serial2.println(" ");
                    }

      
// Testa se paciente tentou inspirar  
  if (inspirar)
    digitalWrite(2,0);
  else  digitalWrite(2,1); 
  

/*
 * Procedimento para calibração do fluxo
 */

   if (cmdCALIB){    
       cmdCALIB = 0;
       DESLIGA_EXP();
       LIGA_INS();
   }
   
/*
 * Monitora a tentativa de inpirar e gera alarme
 */
 
   if ((inspirar) && (tempo) && (((cmdEmExecucao == VCV)||(cmdEmExecucao == PCV)) && (estPSV!=41))) {
          inspirar = 0;
            //ALARME
            
      }
      
if (tempo)      
switch (estPVCV){
     case 0: if (((cmdEmExecucao == VCV)||(cmdEmExecucao == PCV))&&(estPSV == 0)){
                pressaoPlatoCiclo = 0;
                tempoInsCiclo = 0;
                tempoExpCiclo = 0;
                pressoaPicoCiclo = 0;
                volumeCiclo = 0;
                relacaoIEUCiclo = 0;
                flagsCiclo = 0;
                fVolume = 0; //
                TIMER10MS = 0;
                cntTmpLimiteINS = 0;
                pressaoLimiteINS = 0;
                
                volumeMaxINS =  volumeMaxINS_prog;
                rpm = rpm_prog;
                pressaoINS = pressaoINS_prog;
                pressaoPEEP = pressaoPEEP_prog;
                tempoINS = tempoINS_prog;
                tempoEXP = tempoEXP_prog;
                tempoApneia = tempoApneia_prog;
                tempoCiclo = tempoCiclo_prog;
                pressaoSuporte = pressaoSuporte_prog;
                sensibilidadePressao = sensibilidadePressao_prog;
                
                DESLIGA_EXP();
                LIGA_INS();
                estPVCV = 2;
                cntTMP = tempoINS;
                tmpAnaliseInspirar = 100;
                
              } 
              break;
     case 2:  if(
                 ((cmdEmExecucao == VCV)&&(fVolume >= volumeMaxINS))||
                 ((cmdEmExecucao == PCV)&&(pressaoLimiteINS))||
                 (!cntTMP))
                 { // monitora se chegou ao volume ou pressão programado ou o fim do ciclo INS
                pressaoLimiteINS = 0;
                pressoaPicoCiclo = fPressao;
                volumeCiclo = fVolume;
                DESLIGA_INS();            
                estPVCV = 3; 
                pressoaPicoUCiclo = pressoaPicoCiclo;
                volumeUCiclo = volumeCiclo;           
                } 
                
              break;
     case 3:  if (!cntTMP){ // aguarda término do ciclo INS
                estPVCV = 4;
                pressaoLimitePEEP = 0;
                cntTMP = tempoEXP;
                tempoExpCiclo = cntTMP;
                tempoInsCiclo = TIMER10MS;
                tempoInsUCiclo = tempoInsCiclo;
                relacaoIEUCiclo = 0;
                pressaoPlatoCiclo = fPressao;
                pressaoPlatoUCiclo= pressaoPlatoCiclo;
                   
                LIGA_EXP();
              }
              break;
      case 4: if ((pressaoLimitePEEP)||(!cntTMP)){ //monitora fim do ciclo EXP por pressão
                  DESLIGA_EXP();
                  estPVCV = 41;
                  pressaoLimitePEEP = 0;
                  fVolume = 0;
                  ptrRDpressaoAnalise = 0;
                  ptrWRpressaoAnalise = 0;
                  inspirar=0;
                  pressaoLimiteInspirar = 0;
                 }
                break;
       case 41: if ((!cntTMP)||(inspirar)||(pressaoLimiteInspirar)){ // Monitora fim do ciclo EXP por tempo ou pela tentativa de inspirar do paciente
                   if (inspirar) Serial2.println("inspirar");
                   if (pressaoLimiteInspirar) Serial2.println("pressaoLimiteInspirar");
                   Serial2.print(" fPressao: "); Serial2.println(fPressao);
                   Serial2.print(" sensibilidadePressao: "); Serial2.println(sensibilidadePressao); 
                   Serial2.print(" pressaoPEEP: "); Serial2.println(pressaoPEEP);
                   estPVCV = 0;
                   inspirar=0;
                   pressaoLimiteInspirar = 0;
                   tempoExpUCiclo = tempoExpCiclo; 
                   relacaoIEUCiclo = relacaoIEUCiclo;
                   flagsUCiclo = flagsCiclo;
                 }
                break;           
   }
      
/*
 * Máquina de estados do modo de desmame (PSV)
 */
//   if (((cmdEmExecucao == PCV)||(cmdEmExecucao == VCV))&& (estPSV<4)) { // Aborta o ciclo PSV caso chegue um comando PCV ou VCV e o paciente ainda não tiver inspirado
//                      DESLIGA_INS();
//                      estPSV = 0;
//                  }
  // if (tempo)
   switch (estPSV){
     case 0: if ((cmdEmExecucao == PSV)&&(estPVCV == 0)){
                cmdPSV = 0;
                estPSV = 1;
                pressaoPlatoCiclo = 0;
                tempoInsCiclo = 0;
                tempoExpCiclo = 0;
                pressoaPicoCiclo = 0;
                volumeCiclo = 0;
                relacaoIEUCiclo = 0;
                flagsCiclo = 0;
                
                volumeMaxINS = volumeMaxINS_prog;
                rpm = rpm_prog;
                pressaoINS = pressaoINS_prog;
                pressaoPEEP = pressaoPEEP_prog;
                tempoINS = tempoINS_prog;
                tempoEXP = tempoEXP_prog;
                tempoApneia = tempoApneia_prog;
                tempoCiclo = tempoCiclo_prog;
                pressaoSuporte = pressaoSuporte_prog;
                sensibilidadePressao = sensibilidadePressao_prog;

              if (fPressao < ((pressaoSuporte - 1.0)) ){
                   estPSV = 21;        
                   LIGA_INS();
                   DESLIGA_EXP();
                   cntTMP=10;
                } else 
//                if (fPressao > (pressaoSuporte + 0.2) ){
//                        pressaoLimiteSuporte = 0;
//                        estPSV = 22;
//                        LIGA_EXP();
//                        DESLIGA_INS();
//                        cntTMP=10;
//                      }else
                      {
                         cntTMP = tempoApneia;  
                         estPSV = 3;
                         DESLIGA_EXP();
                         DESLIGA_INS();
                         pressaoLimiteInspirarPSV = 0;
                         ptrRDpressaoAnalise = 0;
                         ptrWRpressaoAnalise = 0;
                         inspirar=0;
                      }
            }
            break;
                
    case 21: if(!cntTMP) // tempo para estabilizar antes de medir a pressão
               if (fPressao >= pressaoSuporte ){
                DESLIGA_INS();
                cntTMP2=20;
                cntTMP = tempoApneia;
                pressaoLimiteInspirarPSV = 0;
                inspirar=0;
                ptrRDpressaoAnalise = 0;
                ptrWRpressaoAnalise = 0;
                estPSV = 3;
             }else  if (cmdEmExecucao != PSV) { // Aborta o ciclo PSV caso chegue um comando PCV ou VCV e o paciente ainda não tiver inspirado
                      estPSV = 0;
                      DESLIGA_INS();
                  }
             break;
     case 22: if(!cntTMP) // tempo para estabilizar antes de medir a pressão
                if (pressaoLimiteSuporte ){
                DESLIGA_EXP();
                cntTMP2=20;
                cntTMP = tempoApneia;
                pressaoLimiteInspirarPSV = 0;
                ptrRDpressaoAnalise = 0;
                ptrWRpressaoAnalise = 0;
                estPSV = 3;
                
             }else  if (cmdEmExecucao != PSV) { // Aborta o ciclo PSV caso chegue um comando PCV ou VCV e o paciente ainda não tiver inspirado
                      estPSV = 0;
                      DESLIGA_INS();
                  }
             break;  
             
     case 3: if ((inspirar)||(pressaoLimiteInspirarPSV)||(!cntTMP)){ // Monitora se o paciente tentou inspirar ou se chegou ao  tempo limite de apneia
                if (pressaoLimiteInspirarPSV)
                  Serial2.print("pressaoLimiteInspirarPSV: "); Serial2.println(fPressao); 
                if (inspirar)
                  Serial2.print("inspirar: "); Serial2.println(fPressao);   
                estPSV = 5;
                fVolume = 0;
                inspirar = 0;
                pressaoLimiteInspirarPSV=0;
                DESLIGA_EXP();
                LIGA_INS();
                cntTMP = 10; // Aguarda 100 ms para começar a monitorar se chegou a presão de suporte
                TIMER10MS = 0; // timer para contar o tempo de execução de cada fase do ciclo vcv
                pressaoLimiteINS=0;
                cntTmpLimiteINS = 0;
                
              }else  if (cmdEmExecucao != PSV) { // Aborta o ciclo PSV caso chegue um comando PCV ou VCV e o paciente ainda não tiver inspirado
                      estPSV = 0;
                      DESLIGA_INS();
                  }
              break;       

     case 4:if (!cntTMP){ // Aguarda 100 ms para começar a monitorar se chegou a presão de suporte
                  estPSV = 5;
                  pressaoLimiteINS=0; 
            }          
            break;
     case 5:  if (pressaoLimiteINS){ // monitora se chegou a pressao programada/ fim do ciclo INS
                pressoaPicoCiclo = fPressao;
                volumeCiclo = fVolume;
                DESLIGA_INS();  
                LIGA_EXP();    
                pressaoLimiteSuporte=0;      
               estPSV = 6;
              }
              break;
      case 6: if (pressaoLimiteSuporte){ //monitora fim do ciclo EXP
                  DESLIGA_EXP();
                  estPSV = 0;
                  fVolume = 0;
                  tempoInsUCiclo = tempoInsCiclo;
                  tempoExpUCiclo = tempoExpCiclo; 
                  pressoaPicoUCiclo = pressoaPicoCiclo;
                  volumeUCiclo = volumeCiclo; 
                  relacaoIEUCiclo = relacaoIEUCiclo;
                  pressaoPlatoUCiclo= pressaoPlatoCiclo;
                  flagsUCiclo = flagsCiclo;
                }
                break; 
 }
   tempo = 0;
/*
 * Cotrole de tempo das  rotinas
 */
if (CLOCK10MS) {
  CLOCK10MS=0;
  digitalWrite(9,1);
  tempo = 1;
 
 /* 
  *  Comtrola os timers do sistema
  */
  
  TIMER10MS++;
  if (cntTMP)
    cntTMP--;  
  if (cntTMP2)
    cntTMP2--; 
  if (tmpCOM)  
    tmpCOM--; 
  if (tmpCom2)
    tmpCom2--;  
  if (tmpCom3)
    tmpCom3--; 
  if (tmpGuarda)
     tmpGuarda--;
  if (tmpAnaliseInspirar) 
     tmpAnaliseInspirar--; 
  relogio = relogio + 0.01;         
         
  /*  
   *    Mede a pressão das vias aéreas do paciente
   */
  
  pressaoT[ptrWrBufferPressaoT++] = analogRead(PINPRESSAOINS) - i_PRESS_INS_REF; 
  ptrWrBufferPressaoT &=15;
  if (ptrWrBufferPressaoT==ptrRdBufferPressaoT)
    ptrRdBufferPressaoT++;
  ptrRdBufferPressaoT &=15;  
   
  pressaoMedia=0;
  for (i=0; i<16; i++){
     pressaoMedia += pressaoT[(ptrRdBufferPressaoT + i) & 15];
  }
  pressaoMedia /=16;  
  fPressao = pressaoMedia/10.1977;

  /*
   * Monitora os limites de pressão para controlar mudanças de estado
   */
  
  if (fPressao >= pressaoINS)
    cntTmpLimiteINS++;
  else cntTmpLimiteINS = 0;   
  if (cntTmpLimiteINS>2)
    pressaoLimiteINS = 1;

  if (fPressao <= pressaoPEEP)
    cntTmpLimitePEEP++;
  else cntTmpLimitePEEP = 0;
   if (cntTmpLimitePEEP>3)
         pressaoLimitePEEP = 1;
    
   if ((fPressao <= (pressaoPEEP - sensibilidadePressao))&&(!estadoEXP))
      cntTmpPressaoLimiteInspirar++;
   else cntTmpPressaoLimiteInspirar = 0;   
   if (cntTmpPressaoLimiteInspirar>5)
       pressaoLimiteInspirar = 1;

   if ((fPressao <= (pressaoSuporte - sensibilidadePressao))&&(!estadoEXP))
      cntTmpPressaoLimiteInspirarPSV++;
   else cntTmpPressaoLimiteInspirarPSV = 0;   
   if (cntTmpPressaoLimiteInspirarPSV>5)
       pressaoLimiteInspirarPSV = 1;
    
    if (fPressao<=pressaoSuporte)
       cntTmpPressaoSuporte++;
    else cntTmpPressaoSuporte=0;
    if (cntTmpPressaoSuporte>1)
       pressaoLimiteSuporte = 1;   
  
  /*
   * Analise da curva de pressao para identificar tetativas de inspirar
   */
 
   pressaoAnalise[ptrWRpressaoAnalise++] = fPressao;
 
   ptrWRpressaoAnalise &=LIMITEBUFFERPRESSAOANALISE;
   
   if (ptrWRpressaoAnalise==ptrRDpressaoAnalise) 
     ptrRDpressaoAnalise++;
   ptrRDpressaoAnalise &=LIMITEBUFFERPRESSAOANALISE;  
 
   qtdEscritasBuffer = ptrWRpressaoAnalise-ptrRDpressaoAnalise;
   if (qtdEscritasBuffer<0)
      qtdEscritasBuffer += TAMANHOBUFFERPRESSAOANALISE;
   
    pressaoAnaliseMinima = 1000;
    pressaoAnaliseMaxima = 0;
    
    for (i=0; i<qtdEscritasBuffer ; i++){
      pressao = pressaoAnalise[(ptrRDpressaoAnalise + i) & LIMITEBUFFERPRESSAOANALISE];
      if (pressao >=   pressaoAnaliseMaxima){  
        pressaoAnaliseMaxima = pressao;
        ptrPressaoMaxima=i;
      }
    }
    
    for (i=ptrPressaoMaxima+1; i<qtdEscritasBuffer; i++){
      pressao = pressaoAnalise[(ptrRDpressaoAnalise + i) & LIMITEBUFFERPRESSAOANALISE];
      if (pressao < pressaoAnaliseMinima){
        pressaoAnaliseMinima = pressao;
        ptrPressaoMinima=i;
      }
    }
    
    dffPressao = pressaoAnaliseMaxima-pressaoAnaliseMinima; 
    
    tamanhoCurva =  (ptrPressaoMinima - ptrPressaoMaxima)+1;
      
    if (( tamanhoCurva > 10) && ( pressaoAnaliseMaxima-pressaoAnaliseMinima > 2) && (cmdEmExecucao)&&(!estadoEXP)){
      
          passoPressao = (dffPressao)/( tamanhoCurva );
          erroPressao = 0;
          for (i=0; i<tamanhoCurva ; i++){
            dffPressao = pressaoAnalise[(ptrPressaoMaxima + i) & LIMITEBUFFERPRESSAOANALISE] - (pressaoAnaliseMaxima+(passoPressao*i));
            erroPressao += dffPressao*dffPressao;
          }
          erroPressao = sqrt(erroPressao)/tamanhoCurva;
          
        inspirar=1;
//        Serial2.print(" tamanhoCurva:"); Serial2.print(tamanhoCurva);
//        Serial2.print(" dffPressao:"); Serial2.println(dffPresao);
//        Serial2.print(" ptrWRpressaoAnalise:"); Serial2.print(ptrWRpressaoAnalise);
//        Serial2.print(" ptrRDpressaoAnalise:"); Serial2.println(ptrRDpressaoAnalise);
//        Serial2.print(" pressaoAnaliseMaxima:"); Serial2.print(pressaoAnaliseMaxima,3);
//        Serial2.print(" pressaoAnaliseMinima:"); Serial2.println(pressaoAnaliseMinima,3);
//        Serial2.print(" ptrPressaoMaxima:"); Serial2.print(ptrPressaoMaxima);
//        Serial2.print(" ptrPressaoMinima:"); Serial2.println(ptrPressaoMinima);
//        Serial2.print(" relogio:"); Serial2.println(relogio, 3);
        Serial2.print(" erroPressao:"); Serial2.println(erroPressao, 3);
        
                           
//        for (i=0; i<tamanhoCurva; i++){
//          Serial2.println(pressaoAnalise[(ptrRDpressaoAnalise + i) & LIMITEBUFFERPRESSAOANALISE]);
//        }
//
//        Serial2.println("Buffer completo"); 
//        for (i=0; i<qtdEscritasBuffer; i++){
//          Serial2.println(pressaoAnalise[(ptrRDpressaoAnalise + i) & LIMITEBUFFERPRESSAOANALISE]);
//        }
        
        ptrRDpressaoAnalise +=ptrPressaoMinima; 
        ptrRDpressaoAnalise &=LIMITEBUFFERPRESSAOANALISE;
//        Serial2.print(" Novo ptrWRpressaoAnalise:"); Serial2.print(ptrWRpressaoAnalise);
//        Serial2.print(" Novo ptrRDpressaoAnalise:"); Serial2.println(ptrRDpressaoAnalise); 
//        Serial2.println("-----------------------------------------------------------------"); 
        
        qtdEscritasBuffer = 0;
    }

  /*   
   *    Mede o fluxo inspiratório e, com base neste fluxo, calcula volume de ar  inspirado.
   */
  vazao[0] = vazao[1];
  vazao[1] = vazao[2];
  vazao[2] = vazao[3];
  vazao[3] = vazao[4];
  
  vazao[4] = analogRead(PINPRESSAOVAZAO)-i_PRESS_VAZAO_REF;
  
  if (vazao[4] > vazao[3]+20)
     vazao[4] = vazao[3]+20;
  else if (vazao[4] < vazao[3]-20)
     vazao[4] = vazao[3]-20;
     
  iDP = (vazao[0] + vazao[1] + vazao[2] + vazao[3] + vazao[4])/5;
  fVazaoMedia = 0.90667 +(iDP*0.12044)-(0.000104468*pow(iDP,2));
  
  if (medVolume){
      fVolume += fVazaoMedia/6;
  }
  
  digitalWrite(9,0);  
  
  } 
}
