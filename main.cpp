#include "mbed.h"
#include "MotorDC.h"
#include "mcp3208.h"
#include "color.h"
#include <math.h>
#include <MPU6050.h>
#include "Servo.h"
#include "SoftwarePWM.h"
#include "hcsr04.h"
#include "QEI.h"

////////////////////////////////
//DECLARACIONES CON LIBRERIAS //
////////////////////////////////

QEI enc_izq(PTC0,PTC9,NC,334);
Servo pala(PTB19);

MPU6050 mpu(PTE25,PTE24);

DigitalIn run(PTB3,PullUp);

HCSR04 u_del_sup(PTB20,PTC10);
HCSR04 u_der(PTB10,PTC11);
long distancia_sup;
long distancia_der;

float acc[3];
float gry[3];

float accOffset[3]; //offset values
float gyroOffset[3];
float angle[3];

Serial pc(USBTX,USBRX);
//Serial bt(PTC15,PTC14);

Timer millis;
Timer lec_mpu;

MotorDC m_izq(PTC4,PTC12,PTE26);
MotorDC m_der(PTD0,PTC3,PTC7);
DigitalOut stby(PTC5,1);

ColorSensor c_der(PTB23,PTA1,PTA2);
ColorSensor c_izq(PTC17,PTB9,PTC16);

SPI device(PTD2,PTD3,PTD1);
MCP3208 mcp(device,PTC2);

////////////
//LED RGB //
////////////
DigitalOut led_b(LED_BLUE,1);
DigitalOut led_r(LED_RED,1);
DigitalOut led_g(LED_GREEN,1);

//////////////////////
//MANEJO DE MOTORES //
//////////////////////
float veli = 0;
float veld = 0;
float veld_ant = 0;
float veli_ant = 0;
int pulsos_izq = 0;
bool n = false;
#define motores(a,b) veli = a; veld = b

////////////
//TICKERS //
////////////
Ticker tu; //valores de motor
Ticker tm;

////////////////////////////
//CALIBRACION DE SEGUIDOR //
////////////////////////////
#define N_IZQ 900//1300
#define N_DER 950//1350
#define N_DEL 600
#define N_TRAS 370
#define N_E_IZQ 430
#define N_E_DER 640

#define N_INT_IZQ 760
#define N_INT_DER 730//590//980
#define N_INT_DEL 700
#define N_INT_TRAS 570
#define EXT_INT_I 1080//230
#define EXT_INT_D 980

#define DEL_INTERSEC_1 550
#define DEL_INTERSEC_2 460

#define MIN_IZQ 470
#define MIN_DER 440
#define MIN_TRAS 150
#define MIN_DEL 150

#define RAZON_RGB_I (med_rgb_i.r-rgb_promi < 0 && med_rgb_i.b-rgb_promi < 0 && med_rgb_i.g-rgb_promi>0.02f && rgb_promi < 0.7f && dis_i < 0.04f)
#define RAZON_RGB_D (med_rgb_d.r+med_rgb_d.b-2*rgb_prom < 0 && med_rgb_d.g-rgb_prom>0.02f && rgb_prom < 0.75f && dis_d < 0.04f)

///////////////////////////
//        ESTADOS        //
///////////////////////////

#define seguidor 1
#define seguidor_rampa_up 2
#define seguidor_rampa_down 3

int estado = seguidor;

////////////////////////////////////////////
//CANALES DE SENSORES INFRARROJOS EN SPI  //
////////////////////////////////////////////
#define canal_s_izq  1
#define canal_s_der  3
#define canal_s_e_izq   0
#define canal_s_e_der   4
#define canal_s_del     2
#define canal_s_tras    5

///////////////////////
//VARIABLES GLOBALES //
///////////////////////
int s_izq;
int s_der;
int s_e_izq;
int s_e_der;
int s_del;
int s_tras;

////////////////
//VELOCIDADES //
////////////////
#define vel_giro_menor -0.5f
#define vel_giro_mayor 0.7f
#define vel_adelante 0.6f

#define flanco_de_giro_menor 0.3f
#define flanco_de_giro_mayor 0.2f

#define vel_adelante_intersec 0.7

#define vel_giro_menor_intersec -0.70
#define vel_giro_mayor_intersec 0.8


float x_inicial = 0;
float angle_ant = -87.0f;
bool a_la_rampa = false;
bool en_rampa = false;
bool bajando = false;

//////////////////////////////
//DIRECCION DE INTERSECCION //
//////////////////////////////
int direccion = -1;

///////////////////////////////////////////////
//CALIBRACION DE RGB PARA DETECCION DE VERDE //
///////////////////////////////////////////////
#define I_CALR 0.4417f//0.3892f
#define I_CALG 0.5442f//0.465f
#define I_CALB 0.3933f//0.365f

#define D_CALR 0.5508f//0.4733f
#define D_CALG 0.6333f//0.5691f
#define D_CALB 0.4758f//0.4308f

//////////////////////////////////////////////
//ESTRUCTURA PARA LOS VECTORES DE MAPEO RGB //
//////////////////////////////////////////////
struct vecRGBI{
  float r;
  float g;
  float b;
  float mod;
};

typedef struct vecRGBI vecRGBI;

vecRGBI cal_rgb_i,med_rgb_i;
double dis_i;
float rgb_prom,rgb_promi;

struct vecRGBD{
  float r;
  float g;
  float b;
  float mod;
};

typedef struct vecRGBD vecRGBD;

vecRGBD cal_rgb_d,med_rgb_d;
double dis_d;

////////////////////////////////////////////////
//REMAPEO DE VALORES CON INTERPOLACION LINEAL //
////////////////////////////////////////////////
float fmap(float x, float x1, float x2,float y1, float y2){

  float y;

  y=((y2 - y1)/(x2 - x1))
  *(x - x1) + y1;

  return y;
}

///////////////////////
//LECTURA DE LOS RGB //
///////////////////////
void lec_rgbi(){
  med_rgb_i.r = fmap(c_izq.getRed(), 0, 1200, 0.0, 1.0);
  med_rgb_i.g = fmap(c_izq.getGreen(), 0, 1200, 0.0, 1.0);
  med_rgb_i.b = fmap(c_izq.getBlue(), 0, 1200, 0.0, 1.0);
  rgb_promi = (med_rgb_i.r+med_rgb_i.b+med_rgb_i.g)/3;
  dis_i = pow((cal_rgb_i.r - med_rgb_i.r), 2.0) + pow((cal_rgb_i.g - med_rgb_i.g), 2.0) + pow((cal_rgb_i.b - med_rgb_i.b), 2.0);
}

void lec_rgbd(){
  med_rgb_d.r = fmap(c_der.getRed(), 0, 1200, 0.0, 1.0);
  med_rgb_d.g = fmap(c_der.getGreen(), 0, 1200, 0.0, 1.0);
  med_rgb_d.b = fmap(c_der.getBlue(), 0, 1200, 0.0, 1.0);
  rgb_prom = (med_rgb_d.r+med_rgb_d.b+med_rgb_d.g)/3;
  dis_d = pow((cal_rgb_d.r - med_rgb_d.r), 2.0) + pow((cal_rgb_d.g - med_rgb_d.g), 2.0) + pow((cal_rgb_d.b - med_rgb_d.b), 2.0);
}

/////////////////////////////////////////////////
//ACTUALIZACION DE LA VELOCIDAD DE LOS MOTORES //
/////////////////////////////////////////////////
void updateMotors(){                                      //seteador de velocidades de motores con Ticker TU
  if(veld != veld_ant){
    m_der = veld;
    veld_ant = veld;
  }
  if(veli != veli_ant){
    m_izq = veli;
    veli_ant = veli;
  }
}

///////////////////////////
//MOVIMIENTOS DE LA PALA //
///////////////////////////
void palarribah(){
  for(int i = 70; i> -60;i-=1){
    pala.write(i);
    wait_ms(20.0f);
  }
}

void palahbajoh(){
  pala.enable(-60);
  for(int i = -60; i< 70;i+=1){
    pala.write(i);
    wait_ms(20.0f);
  }
}


void pala_rampa(){
  pala.enable(-60);
  for(int i = -60; i< 55;i+=1){
    pala.write(i);
    wait_ms(10.0f);
  }
}

void pala_arr_rampa(){
  for(int i = 55; i>= -60;i-=1){
    pala.write(i);
    wait_ms(10.0f);
  }
  pala.disable();
}

///////////////////////////////////////
//MUESTREO DEL SENSOR DE ACELERACION //
///////////////////////////////////////
void muestreo_mpu(){
  angle_ant = angle[0];
  mpu.getOffset(accOffset, gyroOffset,1.0);
  mpu.getAccelero(acc);
  mpu.getGyro(gry);
  mpu.getAcceleroAngle(angle);
  angle[0] = 0.98 *(angle[0]+gry[0]*0.010) + 0.02*acc[0];
}

void impresion(){
  //pc.printf("%d\t||\t%d\t||\t%d\t||\t%d\t---\t%d\t||\t%d\n", s_e_izq, s_izq, s_der, s_e_der,s_del,s_tras);
  float g = med_rgb_d.g, b = med_rgb_d.b, r = med_rgb_d.r;
  float prom = (g+r+b)/3;
  //float gi = med_rgb_i.g, bi =med_rgb_i.b, ri = med_rgb_i.r, promi=(gi+bi+ri)/3;
  bool RGBD = (r+b-2*prom < 0 && g-prom>0.02f && prom < 0.75f);
  //bool RGBI = (ri-promi<0 && bi-promi < 0 && gi-promi>0.02f && promi < 0.7f);
  /*if(!RGBD){
    pc.printf("IZQ=\tR:\t%.4f\tG:\t%.4f\tB:\t%.4f\t\t", r, g, b);
    pc.printf("------------dis:\t%f\n", dis_i);
    pc.printf("PROM = %.4f\t RP = %.4f\t GP = %.4f\t BP = %.4f\n",prom,r-prom,g-prom,b-prom);
  }*///pc.printf("DER=\tR:\t%.4f\tG:\t%.4f\tB:\t%.4f\t\t", med_rgb_d.r, med_rgb_d.g, med_rgb_d.b);
  //pc.printf("------------dis:\t%f\n", dis_d);
  led_g = !RGBD;
}

///////////////////////////////////////////////////////////////////////      /////////
//          FUNCION DE EJECUCION EN CASO DE UN INTERSECCION          //         //
///////////////////////////////////////////////////////////////////////         //
///////////////////////////////////////////////////////////////////////         //
///////////////////////////////////////////////////////////////////////         //
///////////////////////////////////////////////////////////////////////      /////////

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
  enc_izq.reset();

  if(direccion == 0) {                                          //seguir derecho
    led_g = 0;
    motores(vel_adelante_intersec,vel_adelante_intersec);
    wait_ms(10.0f);
    pulsos_izq = enc_izq.getPulses();
    while(pulsos_izq < 110){
      lec_rgbd();
      lec_rgbi();
      pulsos_izq = enc_izq.getPulses();
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
    }
  }
  if(direccion == 1) {                                //interseccion hacia la izquierdo
    led_b = 0;
    led_r = 0;
    motores(vel_adelante_intersec,vel_adelante_intersec);
    pulsos_izq = enc_izq.getPulses();
    while(pulsos_izq < 110){
      lec_rgbd();
      if(RAZON_RGB_D){
        motores(0.0,0.0);
        direccion = 3;
        break;
      }
      pulsos_izq = enc_izq.getPulses();
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
  if(direccion == 2) {                                    //interseccion hacia la derecha
    led_b = 0;
    led_g = 0;
    motores(vel_adelante_intersec+0.1,vel_adelante_intersec);
    lec_rgbi();
    pulsos_izq = enc_izq.getPulses();
    while(pulsos_izq < 110){
      lec_rgbi();
      if(RAZON_RGB_I){
        motores(0.0,0.0);
        direccion = 3;
        break;
      }
      pulsos_izq = enc_izq.getPulses();
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
  if(direccion == 3) {                                     //pegarse la vuelta
    led_g = 0;
    led_r = 0;
    enc_izq.reset();
    motores(-0.7,0.7);
    pulsos_izq = enc_izq.getPulses();

    while(pulsos_izq > -1000){
      pulsos_izq = enc_izq.getPulses();
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

/////////////////////////////////////////////////////////////////////     /////////
/////////////////////////////////////////////////////////////////////     //     //
//          FUNCION DE EJECUCION EN CASO DE UN OBSTACULO           //     //     //
/////////////////////////////////////////////////////////////////////     //     //
/////////////////////////////////////////////////////////////////////     /////////


void obstaculo(){

  wait_us(500.0f);
  int esquivar = 1;
  distancia_der = u_der.distance();
  wait_ms(60.0f);

  if(esquivar == 1){                    //INICIO DEL GIRO
    motores(-0.7,0.7);
  }
  wait_ms(100.0f);
  enc_izq.reset();

  if(esquivar == 1){
    motores(-0.7,0.7);
    pulsos_izq = enc_izq.getPulses();
    while(pulsos_izq > -550){
      pulsos_izq = enc_izq.getPulses();
    }                                   //FIN DEL GIRO//INICIO DE ESQUIVE

    while(1){
      distancia_der = u_der.distance();
      wait_ms(60.0f);
      if(distancia_der < 8 && distancia_der != 0){
        motores(0.6,0.6);
      }
      else{
        motores(0.75,-0.5);
      }
      s_del = mcp.iread_input(canal_s_del);
      if(s_del < N_DEL){
        break;
      }
    }                                     //FIN DE ESQUIVE
    led_b = 0;
    led_r=0;                              //RETOME DE LINEA
    led_g=0;
    motores(0.6,0.6);
    s_del = mcp.iread_input(canal_s_del);
    while(s_del < N_DEL+10){
      s_del = mcp.iread_input(canal_s_del);
    }
    s_der = mcp.iread_input(canal_s_der);
    while(s_der < N_DER+10){
      s_der = mcp.iread_input(canal_s_der);
    }
    motores(-0.7,0.7);
    s_der = mcp.iread_input(canal_s_der);
    while(s_der > N_DER-10){
      s_der = mcp.iread_input(canal_s_der);
    }
  }
  motores(0.0,0.0);                           //FINALIZA  EL OBSTACULO //REINICIO DEL SEGUIDOR
  distancia_der = 15;
  led_b = 1;
  led_r=1;
  led_g=1;
  wait(0.1f);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////        PROCESO QUE REALIZA EL ROBOT        //////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////


int main(){
  cal_rgb_i.r = I_CALR;               //DATOS PARA MAP DE RGB
  cal_rgb_i.g = I_CALG;
  cal_rgb_i.b = I_CALB;

  cal_rgb_d.r = D_CALR;
  cal_rgb_d.g = D_CALG;
  cal_rgb_d.b = D_CALB;
  //bt.baud(9600);
  while(run){
    led_g = 0;
  }
  led_g = 1;
  pala.setLimits(500,2400);
  pala.enable(-55);                        //LLEVAR LA PALA HACIA ARRIBA
  tu.attach_us(&updateMotors,60.0f);   //ATTACH DE TICKERS
  angle[0] = -87.0f;
  //muestreo_mpu();
  pc.baud(115200);
  distancia_sup = u_del_sup.distance();
  wait(3.0f);
  pala.disable();
  millis.start();
  //  mpu.setAlpha(0);
  unsigned long tiempo = 0;
  //bt.printf("a\n");
  lec_mpu.start();
  //int min_der = 9999;
  pc.printf("Imprimiendo. Ok!!");
  wait(2);
  while(true){
    //////////////////////
    //SEGUIDOR DE LINEA //
    //////////////////////

    s_izq = mcp.iread_input(canal_s_izq);
    s_der = mcp.iread_input(canal_s_der);
    s_e_izq = mcp.iread_input(canal_s_e_izq);
    s_e_der = mcp.iread_input(canal_s_e_der);
    s_del = mcp.iread_input(canal_s_del);
    s_tras = mcp.iread_input(canal_s_tras);
    tiempo = millis.read_ms();
    lec_rgbi();
    lec_rgbd();
    distancia_sup = u_del_sup.distance();
    //pc.printf("\t\t\t   del = %d\n\nizq_e = %d\tizq = %d\t\tder = %d\tder_e = %d\n\n\t\t\t   tra = %d\n",s_del,s_e_izq,s_izq,s_der,s_e_der,s_tras);
    if(((s_izq < N_INT_IZQ && s_del < N_INT_DEL && s_tras < N_INT_TRAS) ||
    (s_der < N_INT_DER && s_del < N_INT_DEL && s_tras < N_INT_TRAS)) &&
    (s_e_der < 950 || s_e_izq < 1050)){
    char del=' ',izq_e=' ',izq=' ',der=' ',der_e=' ',tra=' ';
    if(s_izq < N_INT_IZQ) izq = 'X';
    if(s_del < N_INT_DEL) del = 'X';
    if(s_tras < N_INT_TRAS) tra = 'X';
    if(s_der < N_INT_DER) der = 'X';
    if(s_e_der < 950) der_e='X';
    if(s_e_izq < 1050) izq_e ='X';
    pc.printf("\t\t\t   del = %d%c\n\nizq_e = %d%c\tizq = %d%c\t\tder = %d%c\tder_e = %d%c\n\n\t\t\t   tra = %d%c\n",s_del,del,s_e_izq,izq_e,s_izq,izq,s_der,der,s_e_der,der_e,s_tras,tra);
}
    //if(lec_mpu.read_ms() > 40){
      muestreo_mpu();
      //lec_mpu.reset();
    //}
    //impresion();
    //
    //
    /*if(s_der < min_der){
      min_der = s_der;
      pc.printf("min_der --> %d\n", min_der);
    }*/
    if(estado == seguidor){

      /*led_g = 0;
      wait(2);

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
        bool inters=((((s_izq < N_INT_IZQ && s_del < N_INT_DEL && s_tras < N_INT_TRAS) ||
        (s_der < N_INT_DER && s_del < N_INT_DEL && s_tras < N_INT_TRAS)) &&
        (s_e_der < 950 || s_e_izq < 1050))
        || ((RAZON_RGB_I || RAZON_RGB_D) && tiempo > 500)) && ((fabs(angle[0]) > 80.0f && angle[0] > 0) || (fabs(angle[0]) > 85.5f && angle[0] < 0));
      );
        led_g = !inters;
      }*/
      if((fabs(angle[0]) < 69.0f && angle[0] < 0) && (fabs(angle_ant) < 69.0f && angle_ant < 0)){
        ///////////////////////
        //DETECCION DE RAMPA //
        ///////////////////////
        //bt.printf("rampa\n");
        motores(0.0,0.0);
        wait(0.1f);
        led_g = 0;
        led_r = 0;
        led_b = 0;
        pala_rampa();
        wait(2.0f);
        led_g = 1;
        led_r = 1;
        led_b = 1;
        estado = seguidor_rampa_up;
        pc.printf("estado = rampa_up\n" );
      }
      else if(en_rampa == true && (fabs(angle[0]) < 76.0f && angle[0] > 0) && (fabs(angle_ant) < 76.0f && angle_ant > 0)){
        motores(0.4,0.4);
        estado = seguidor_rampa_down;
        pc.printf("estado = rampa_down\n" );
      }
      /*else if(distancia_sup < 7 && distancia_sup != 0){
        //bt.printf("obstaculo\n");
        motores(0,0);
        obstaculo();
      }*/
      else if (((((s_izq < N_INT_IZQ && s_del < N_INT_DEL && s_tras < N_INT_TRAS) ||
      (s_der < N_INT_DER && s_del < N_INT_DEL && s_tras < N_INT_TRAS)) &&
      (s_e_der < 950 || s_e_izq < 1050)) ||
      ((RAZON_RGB_I || RAZON_RGB_D) && tiempo > 500)) && ((fabs(angle[0]) > 80.0f && angle[0] > 0) || (fabs(angle[0]) > 85.5f && angle[0] < 0)))
      {
        pc.printf("interseccion\n");
        //impresion();
        char del=' ',izq_e=' ',izq=' ',der=' ',der_e=' ',tra=' ';
        if(s_izq < N_INT_IZQ) izq = 'X';
        if(s_del < N_INT_DEL) del = 'X';
        if(s_tras < N_INT_TRAS) tra = 'X';
        if(s_der < N_INT_DER) der = 'X';
        if(s_e_der < 950) der_e='X';
        if(s_e_izq < 1050) izq_e ='X';
        pc.printf("\t\t\t   del = %d%c\n\nizq_e = %d%c\tizq = %d%c\t\tder = %d%c\tder_e = %d%c\n\n\t\t\t   tra = %d%c\n",s_del,del,s_e_izq,izq_e,s_izq,izq,s_der,der,s_e_der,der_e,s_tras,tra);
        motores(-0.01,-0.01);
        interseccion();
      }
      //else if()
      else if(s_der < 950) {
        motores(0.8f,-0.6f-(0.3f*(s_der < 400)));
        /*if(s_der < 400){
          pc.printf("muy negro der\n" );
        }*/
      }
      else if(s_izq < 950) {
        motores(-0.6f-(0.3f*(s_izq < 480)),0.8f);
        //if(s_izq < 480)
        //pc.printf("muy negro izquierdo\n" );
      }
      else {
        motores(0.55f,0.55f);
      }
    }

    ///////////////////////////////
    //seguidor en rampa subiendo //
    ///////////////////////////////

    else if(estado == seguidor_rampa_up){

      if(fabs(angle[0]) > 83.5f && fabs(angle_ant) > 84.0f){
        motores(0.0,0.0);
        a_la_rampa = false;
        en_rampa = true;
        wait(1.0f);
        pala_arr_rampa();
        wait(2.0f);
        estado = seguidor;
      }
      else if(s_izq < N_IZQ && s_der > N_DER) {
        motores(0.5,0.9);
      }
      else if(s_der < N_DER && s_izq > N_IZQ) {
        motores(0.9,0.5);
      }
      else {
        motores(0.8,0.8);
      }
      if(fabs(angle[0]) < 58.0f && angle[0] < 0 && n == true){
        pala.write(68);
        n = false;
      }
      else if(n == false && fabs(angle[0]) > 58.0f && angle[0] < 0){
        pala.write(55);
        n = true;
      }
    }

    //////////////////////////////
    //seguidor en rampa bajando //
    //////////////////////////////

    else if(estado == seguidor_rampa_down){
      if(fabs(angle[0]) > 83.0f && fabs(angle_ant) > 83.0f){
        estado = seguidor;
      }
      /*else if(fabs(angle[0]) < 45.0f && fabs(angle_ant) < 45.0f && angle[0] > 0 && angle_ant > 0){
        pala.enable(-60);
        motores(-0.6,-0.6);
        for(int i = -60;i < 70;i+=1){
          pala.write(i);
          wait_ms(20.0f);
        }
      }*/
      else if(s_izq < N_IZQ && s_der > N_DER) {
        motores(-0.05,0.3);
      }
      else if(s_der < N_DER && s_izq > N_IZQ) {
        motores(0.3,-0.05);
      }
      else {
        motores(0.3,0.3);
      }
    }
  }

}
