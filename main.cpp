#include "mbed.h"
#include "MotorDC.h"
#include "mcp3208.h"
#include "color.h"
#include "hcsr04.h"
#include <math.h>

//M: Declaracion para comunicarse con el monitor serial:
Serial pc(USBTX,USBRX);

//M: Declaracion de los sensores ultrasonidos:
HCSR04 ultra(PTC7,PTC5);       // ultrasonido del frente 
HCSR04 ultra_der(PTC4,PTC12);  // ultrasonido del costado derecho
HCSR04 ultra_izq(PTB9,PTA1);   // ultrasonido del costado izquierdo

//M: Declaracion del objeto que permite medir el
//tiempo paso desde que empezo a correr el programa:
Timer millis;

//M: Valores recogidos por las lecturas de los sensores:
int distancia_del; // de ultra
int distancia_der; // de ultra_der
int distancia_izq; // de ultra_izq

//M: Declaracion de los motores de las ruedas del robot:
MotorDC m_izq(PTC10,PTB3,PTB2);
MotorDC m_der(PTC11,PTB10,PTB11);

//M: ?
DigitalOut stby(PTB20,1);

//M: Declaracion de los sensores de color derecho e izquierdo:
ColorSensor c_der(PTB19,PTC1,PTB18);
ColorSensor c_izq(PTC9,PTC8,PTC0);

//M: Declaracion de objetos para poder comunicarse con
//el microchip mcp:
SPI device(PTD2,PTD3,PTD1);
MCP3208 mcp(device,PTD0);

//M: Declaracion de leds RGB:
DigitalOut led_b(LED_BLUE,1);
DigitalOut led_r(LED_RED,1);
DigitalOut led_g(LED_GREEN,1);

//Manejo de Motores
//M: ?
#define motores(a,b) veli = a; veld = b

//M: Tickers:
//M: Los tickers permiten interrumpir el programa y 
//llamar a una funcion en un determinado intervalo de tiempo
Ticker tm; //muestreo de sensores
Ticker tu; //valores de motor
Ticker tmu;//muestreo ultra
Ticker trgbd;
Ticker trgbi;

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
#define RAZON_RGB_I (dis_i < 0.02f && med_rgb_i.g > med_rgb_i.b)
#define RAZON_RGB_D (dis_d < 0.02f && med_rgb_d.g > med_rgb_d.b)

//canales de sensores infrarrojos conectados en SPI
#define canal_s_izq  1
#define canal_s_der  3
#define canal_s_e_izq   0
#define canal_s_e_der   4
#define canal_s_del     2
#define canal_s_tras    5

int s_izq;
int s_der;
int s_e_izq;
int s_e_der;
int s_del;
int s_tras;

unsigned long tiempo;

//M: velocidades
#define vel_giro_menor -0.2f
#define vel_giro_mayor 0.4f
#define vel_adelante 0.3f
#define flanco_de_giro_menor 0.32f
#define flanco_de_giro_mayor 0.42f
#define vel_adelante_intersec 0.3
#define vel_giro_menor_intersec -0.45
#define vel_giro_mayor_intersec 0.5

//variables para setear velocidad:
float veli = 0; //M: velocidad izquierda
float veld = 0; //M: velocidad derecha

//direccion de interseccion inicial

int direccion = -1;

#define I_CALR 0.4225f
#define I_CALG 0.4958f
#define I_CALB 0.3633f

#define D_CALR 0.5125f
#define D_CALG 0.575f
#define D_CALB 0.4817f

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

void muestreo_del(){
  distancia_del = ultra.distance();
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

void obstaculo(){
  tm.detach();

  int ini = distancia_del;
  wait_us(500.0f);
  int esquivar = 0;
  distancia_der = ultra_der.distance();
  distancia_izq = ultra_izq.distance();
  wait_ms(60.0f);
  if(distancia_der > 28){
    esquivar = 2;
  }
  else{
    esquivar = 1;
  }
  if(esquivar == 1){
    motores(-0.3,0.3);
  }
  else if(esquivar == 2){
    motores(0.3,-0.3);
  }
  wait_ms(300.0f);
  if(esquivar == 1){
    motores(-0.3,0.3);
    while(distancia_der > ini){
      distancia_der = ultra_der.distance();
      wait_ms(60.0f);
      led_b = 0;
    }
    while(1){
      distancia_der = ultra_der.distance();
      wait_ms(60.0f);
      if(distancia_der <= ini){
        led_b= 0;
        motores(0.3,0.3);
      }
      else{
        led_b = 0;
        motores(0.5,-0.1);
      }
      s_del = mcp.iread_input(canal_s_del);
      if(s_del < N_DEL){
        break;
      }
    }
    led_r=0;
    led_g=0;
    motores(-0.3,0.3);
    wait(2.0f);
  }
  else if(esquivar == 2){
    motores(0.3,-0.3);
    while(distancia_izq > ini){
      distancia_izq = ultra_izq.distance();
      wait_ms(60.0f);
      led_b = 0;
    }
    while(1){
      distancia_izq = ultra_izq.distance();
      wait_ms(60.0f);
      led_b = 0;
      if(distancia_izq <= ini){
        led_b= 0;
        motores(0.3,0.3);
      }
      else{
        led_b = 0;
        motores(-0.1,0.5);
      }
      s_del = mcp.iread_input(canal_s_del);
      if(s_del < N_DEL){
        break;
      }
    }
    led_r=0;
    led_g=0;
    motores(0.3,-0.3);
    wait(2.0f);
  }
  motores(0,0);
  //tm.attach(&muestreo_del,0.002f);
  distancia_del = 15;
  led_b = 1;
  led_r=1;
  led_g=1;
  wait(0.5f);
  s_izq = mcp.iread_input(canal_s_izq);
  s_der = mcp.iread_input(canal_s_der);
  s_e_izq = mcp.iread_input(canal_s_e_izq);
  s_e_der = mcp.iread_input(canal_s_e_der);
  s_del = mcp.iread_input(canal_s_del);
  s_tras = mcp.iread_input(canal_s_tras);
  millis.reset();
  lec_rgbi();
  lec_rgbd();
  //c.printf("
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
  //tm.attach(&muestreo_del,0.02f);
  //pc.baud(115200);
  //bt.baud(9600);
  distancia_del = ultra.distance();
  wait(0.5f);
  millis.start();
  tiempo = 0;
  distancia_del = 15;

  while(true){

      distancia_del = ultra.distance();
      wait_ms(30);
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
    else if(distancia_del < 6 && distancia_del > 0 && distancia_del !=5){
      led_b = 0;
      obstaculo();
      tm.attach(&muestreo_del,0.02f);
    }
    else if(s_izq < N_IZQ) {
      motores(vel_giro_menor-(flanco_de_giro_menor*(s_izq < MIN_IZQ)),vel_giro_mayor+(flanco_de_giro_mayor*(s_izq < MIN_IZQ)));
    } else if(s_der < N_DER) {
      motores(vel_giro_mayor+(flanco_de_giro_mayor*(s_der < MIN_DER)),vel_giro_menor-(flanco_de_giro_menor*(s_der < MIN_DER)));
    } else {
      motores(vel_adelante,vel_adelante);
    }
    //wait(0.2f);
  }
}
