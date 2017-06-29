#include "mbed.h"
#include "MotorDC.h"
#include "mcp3208.h"

//Declaraciones con Librerias

MotorDC m_izq(PTC10,PTB3,PTB2);
MotorDC m_der(PTC11,PTB10,PTB11);
DigitalOut stby(PTB20,1);

SPI device(PTD2,PTD3,PTD1);
MCP3208 mcp(device,PTD0);

//Pines led RGB

DigitalOut led_b(LED_BLUE,1);
DigitalOut led_r(LED_RED,1);
DigitalOut led_g(LED_GREEN,1);

//Manejo de Motores

#define motores(a,b) veli = a; veld = b

//tickers

Ticker tu; //valores de motor

//valores de seguidor

#define N_IZQ 1300
#define N_DER 1200
#define N_DEL 600
#define N_TRAS 490
#define N_E_IZQ 160
#define N_E_DER 390



#define N_INT_IZQ 1380
#define N_INT_DER 1300//980
#define N_INT_DEL 700
#define N_INT_TRAS 640
#define EXT_INT_I 220//230
#define EXT_INT_D 540


#define DEL_INTERSEC_1 700
#define DEL_INTERSEC_2 600

#define MIN_IZQ 920
#define MIN_DER 900
#define MIN_TRAS 150
#define MIN_DEL 150

//canales de sensores infrarrojos conectados en SPI

#define canal_s_izq  1
#define canal_s_der  3
#define canal_s_e_izq   0
#define canal_s_e_der   4
#define canal_s_del     2
#define canal_s_tras    5

//variables globales

int s_izq;
int s_der;
int s_e_izq;
int s_e_der;
int s_del;
int s_tras;

//velocidades

#define vel_giro_menor -0.2f
#define vel_giro_mayor 0.4f
#define vel_adelante 0.3f

#define flanco_de_giro_menor 0.32f
#define flanco_de_giro_mayor 0.42f

#define vel_adelante_intersec 0.3

#define vel_giro_menor_intersec -0.45
#define vel_giro_mayor_intersec 0.5

//varibles para setear velocidad

float veli = 0;
float veld = 0;

void updateMotors(){                                      //seteador de velocidades de motores con Ticker TU
  m_der = veld;
  m_izq = veli;
}

//***************************************************************************************************************************

int main(){

  tu.attach_us(&updateMotors,100.0f);
  wait(0.5f);

  while(true){
    s_izq = mcp.iread_input(canal_s_izq);
    s_der = mcp.iread_input(canal_s_der);
    s_e_izq = mcp.iread_input(canal_s_e_izq);
    s_e_der = mcp.iread_input(canal_s_e_der);
    s_del = mcp.iread_input(canal_s_del);
    s_tras = mcp.iread_input(canal_s_tras);

    if(s_izq < N_IZQ) {
      motores(vel_giro_menor-(flanco_de_giro_menor*(s_izq < MIN_IZQ)),vel_giro_mayor+(flanco_de_giro_mayor*(s_izq < MIN_IZQ)));
    } else if(s_der < N_DER) {
      motores(vel_giro_mayor+(flanco_de_giro_mayor*(s_der < MIN_DER)),vel_giro_menor-(flanco_de_giro_menor*(s_der < MIN_DER)));
    } else {
      motores(vel_adelante,vel_adelante);
    }
  }
}
