#include "mbed.h"
#include "MotorDC.h"
#include "mcp3208.h"
#include "color.h"
#include <math.h>

//Declaraciones con Librerias

Serial pc(USBTX,USBRX);

Timer millis;

MotorDC m_izq(PTC10,PTB3,PTB2);
MotorDC m_der(PTC11,PTB10,PTB11);
DigitalOut stby(PTB20,1);

ColorSensor c_der(PTB19,PTC1,PTB18);
ColorSensor c_izq(PTC9,PTC8,PTC0);

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

#define RAZON_RGB_I (dis_i < 0.0098f && med_rgb_i.g > (med_rgb_i.b-0.006f))
#define RAZON_RGB_D (dis_d < 0.009f && med_rgb_d.g > (med_rgb_d.b-0.008f))

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

//direccion de interseccion inicial

int direccion = -1;

#define I_CALR 0.3616f//0.4733f
#define I_CALG 0.3883f//0.5691f
#define I_CALB 0.3241f//0.4308f

#define D_CALR 0.4966f//0.4733f
#define D_CALG 0.5816f//0.5691f
#define D_CALB 0.4492f//0.4308f

struct vecRGBI{
    float r;
    float g;
    float b;
    float mod;
};

typedef struct vecRGBI vecRGBI;

vecRGBI cal_rgb_i,med_rgb_i;
double dis_i;

struct vecRGBD{
    float r;
    float g;
    float b;
    float mod;
};

typedef struct vecRGBD vecRGBD;

vecRGBD cal_rgb_d,med_rgb_d;
double dis_d;

//////////////////////////////////////////////////////////////

float linearInterpolation(float x, float x1, float x2,float y1, float y2){

  float y;

  y=((y2 - y1)/(x2 - x1))
  *(x - x1) + y1;

  return y;
}

void lec_rgbi(){
  med_rgb_i.r = linearInterpolation(c_izq.getRed(), 0, 1200, 0.0, 1.0);
  med_rgb_i.g = linearInterpolation(c_izq.getGreen(), 0, 1200, 0.0, 1.0);
  med_rgb_i.b = linearInterpolation(c_izq.getBlue(), 0, 1200, 0.0, 1.0);
  dis_i = pow((cal_rgb_i.r - med_rgb_i.r), 2.0) + pow((cal_rgb_i.g - med_rgb_i.g), 2.0) + pow((cal_rgb_i.b - med_rgb_i.b), 2.0);
}

void lec_rgbd(){
  med_rgb_d.r = linearInterpolation(c_der.getRed(), 0, 1200, 0.0, 1.0);
  med_rgb_d.g = linearInterpolation(c_der.getGreen(), 0, 1200, 0.0, 1.0);
  med_rgb_d.b = linearInterpolation(c_der.getBlue(), 0, 1200, 0.0, 1.0);
  dis_d = pow((cal_rgb_d.r - med_rgb_d.r), 2.0) + pow((cal_rgb_d.g - med_rgb_d.g), 2.0) + pow((cal_rgb_d.b - med_rgb_d.b), 2.0);
}

void updateMotors(){                                      //seteador de velocidades de motores con Ticker TU
  m_der = veld;
  m_izq = veli;
}

void interseccion()                                       //en caso de interseccion se llama a este funcion
{
    //deteccion de cuadro verde.....
    if(!RAZON_RGB_I && !RAZON_RGB_D){
      lec_rgbi();
      lec_rgbd();
    }
    else if(RAZON_RGB_I){
      lec_rgbd();
    }
    else if(RAZON_RGB_D){
      lec_rgbi();
    }
    led_r = 1;
    led_g = 1;
    wait(0.1f);
    if(RAZON_RGB_I && RAZON_RGB_D){   //lectura de los RGB para encontrar un cuadro verde
      direccion = 3;
    }
    else if(RAZON_RGB_D){
      direccion = 2;
    }
    else if(RAZON_RGB_I){
      direccion = 1;
    }
    else{
      direccion = 0;
    }

    motores(0.0,0.0);
    wait_ms(100);

    if(direccion == 0) { //segui derecho
      led_g = 0;
      motores(vel_adelante_intersec,vel_adelante_intersec);
      wait_ms(150.0f);
      s_e_izq = mcp.iread_input(canal_s_e_izq);
      s_e_der = mcp.iread_input(canal_s_e_der);
        while(s_e_der < N_E_DER || s_e_izq < N_E_IZQ){
          lec_rgbd();
          lec_rgbi();
          if(RAZON_RGB_I && RAZON_RGB_D){
            motores(0.0,0.0);
            direccion = 3;
            break;
          }
          else if(RAZON_RGB_I){
            motores(0.0,0.0);
            direccion = 1;
            break;
          }
          else if(RAZON_RGB_D){
            motores(0.0,0.0);
            direccion = 2;
            break;
          }
          s_e_izq = mcp.iread_input(canal_s_e_izq);
          s_e_der = mcp.iread_input(canal_s_e_der);
        }
    }
    if(direccion == 1) {//interseccion hacia la izquierdo
      led_b = 0;
      led_r = 0;
      motores(vel_adelante_intersec,vel_adelante_intersec);
      s_izq = mcp.iread_input(canal_s_izq);
      lec_rgbd();
      while(s_izq < N_INT_IZQ+10){
        lec_rgbd();
        if(RAZON_RGB_D){
          motores(0.0,0.0);
          direccion = 3;
          break;
        }
        s_izq = mcp.iread_input(canal_s_izq);
        led_r = 1;
      }
      if(direccion == 1){
        motores(vel_giro_menor_intersec,vel_giro_mayor_intersec+0.1);
        s_del = mcp.iread_input(canal_s_del);
        while(s_del < DEL_INTERSEC_1){
          s_del = mcp.iread_input(canal_s_del);
          led_r = 1;
        }
        while(s_del > DEL_INTERSEC_2){
          s_del = mcp.iread_input(canal_s_del);
          led_r = 1;
        }
      }
    }
    if(direccion == 2) {//interseccion hacia la derecha
        led_b = 0;
        led_g = 0;
        motores(vel_adelante_intersec+0.1,vel_adelante_intersec);
        s_der = mcp.iread_input(canal_s_der);
        lec_rgbi();
        while(s_der < N_INT_DER+10){
          lec_rgbi();
          if(RAZON_RGB_I){
            motores(0.0,0.0);
            direccion = 3;
            break;
          }
          s_der = mcp.iread_input(canal_s_der);
          led_r = 1;
        }
      if(direccion == 2){
        motores(vel_giro_mayor_intersec,vel_giro_menor_intersec);
        s_del = mcp.iread_input(canal_s_del);
        while(s_del < DEL_INTERSEC_1){
          s_del = mcp.iread_input(canal_s_del);
          led_r = 1;
        }
        while(s_del > DEL_INTERSEC_2){
          s_del = mcp.iread_input(canal_s_del);
          led_r = 1;
        }
      }
    }
    if(direccion == 3) {
      led_g = 0;
      led_r = 0;
        //pegarse la vuelta
        motores(-0.4,0.4);
        s_tras = mcp.iread_input(canal_s_tras);
        while(s_tras < N_TRAS+10)
        {
          s_tras = mcp.iread_input(canal_s_tras);
          led_r = 1;
        }
        s_izq = mcp.iread_input(canal_s_izq);
        while(s_izq < N_IZQ+10)
        {
          s_izq = mcp.iread_input(canal_s_izq);
          led_r = 1;
        }
        while(s_izq > N_IZQ-5)
        {
          s_izq = mcp.iread_input(canal_s_izq);
          led_r = 1;
        }
        s_der = mcp.iread_input(canal_s_der);
        while(s_der > N_DER)
        {
          s_der = mcp.iread_input(canal_s_der);
          led_r = 1;
        }
    }
    led_r = 1;
    led_g = 1;
    led_b = 1;
    millis.reset();
}

//***************************************************************************************************************************

int main(){
  cal_rgb_i.r = I_CALR;
	cal_rgb_i.g = I_CALG;
	cal_rgb_i.b = I_CALB;

  cal_rgb_d.r = D_CALR;
	cal_rgb_d.g = D_CALG;
	cal_rgb_d.b = D_CALB;

  tu.attach_us(&updateMotors,100.0f);
  wait(0.5f);
  millis.start();
  unsigned long tiempo = 0;

  while(true){
    s_izq = mcp.iread_input(canal_s_izq);
    s_der = mcp.iread_input(canal_s_der);
    s_e_izq = mcp.iread_input(canal_s_e_izq);
    s_e_der = mcp.iread_input(canal_s_e_der);
    s_del = mcp.iread_input(canal_s_del);
    s_tras = mcp.iread_input(canal_s_tras);
    tiempo = millis.read_ms();

    lec_rgbi();
    lec_rgbd();

    if((((s_izq < N_INT_IZQ && s_der < N_INT_DER && s_tras < N_INT_TRAS && s_del < N_INT_DEL) ||
    (s_izq < N_INT_IZQ && s_del < N_INT_DEL && s_tras < N_INT_TRAS) ||
    (s_der < N_INT_DER && s_del < N_INT_DEL && s_tras < N_INT_TRAS)) &&
    (s_e_der < 950 || s_e_izq < 650)) ||
    ((RAZON_RGB_I || RAZON_RGB_D) && tiempo > 400)) {
      led_g = !RAZON_RGB_I;
      led_r = !RAZON_RGB_D;
      motores(-0.01,-0.01);
      wait(0.1f);
      interseccion();
    }
    else if(s_izq < N_IZQ) {
      motores(vel_giro_menor-(flanco_de_giro_menor*(s_izq < MIN_IZQ)),vel_giro_mayor+(flanco_de_giro_mayor*(s_izq < MIN_IZQ)));
    } else if(s_der < N_DER) {
      motores(vel_giro_mayor+(flanco_de_giro_mayor*(s_der < MIN_DER)),vel_giro_menor-(flanco_de_giro_menor*(s_der < MIN_DER)));
    } else {
      motores(vel_adelante,vel_adelante);
    }
  }
}
