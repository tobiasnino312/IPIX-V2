//CODIGO UTILIZADO CON EL ROBOT IPITS 2.0 EN COMPETENCIA INTERZONAL DE ROBOTICA EL 31/08/2025 CON UN RESULTADO FINAL DE UN 3°ER PUESTO//
//LIBRERIAS//
#include <QTRSensors.h>

//SENSORES QTR8A//
QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

//DEFINICIONES DE PINES//
#define PWMder 10
#define IN1der 11 
#define IN2der 12
#define PWMizq 9
#define IN1izq 8
#define IN2izq 7
#define DIP 2
#define LED 5
#define BTN 6
#define IR 13

int estado=0;
int VDER, VIZQ, linea, errorant, error, integral, derivativa, velocidad, VMAX;
float kp, ki, kd, correccion;

//INTERVALOS DE TIEMPOS SENGUN SECTOR DE LA PISTA//
int tiempoinicio=0;
int tiempoactual=0;
int tiempo1=3000;
int tiempo2=999999;
int tiempo3=999999;
int tiempo4=999999;
int tiempo5=999999;
int tiempo6=999999;
int VMIN = 80;
int VEL = 80;

void setup() {
  // Fast PWM con TOP = ICR1
  TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM11);
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS10); // prescaler = 1
  ICR1 = 999; // → 16 kHz exacto

  OCR1A = 0; // Motor A (pin 9)
  OCR1B = 0; // Motor B (pin 10)

//SENSORES//
qtr.setTypeAnalog();
qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, SensorCount);

//PINOUTS// 
pinMode(PWMder,OUTPUT);
pinMode(PWMizq,OUTPUT);
pinMode(IN1der,OUTPUT);
pinMode(IN2der,OUTPUT);
pinMode(IN1izq,OUTPUT);
pinMode(IN2izq,OUTPUT);
pinMode(LED,OUTPUT);
pinMode(BTN,INPUT);
pinMode(DIP,INPUT);
pinMode(IR,OUTPUT);

//ACTIVACION DE LEDS DE SENSORES//
digitalWrite(IR,1);
//LECTURA DIP//
if (digitalRead(DIP)==1){
    kp=0.35;
    ki=0.05;
    kd=38;    
    velocidad=80;
    VMAX=160; 
  }
  else{      
//I (Interior)
    kp=0.35;
    ki=0.05;
    kd=38;    
    velocidad=70;
    VMAX=140; 
  }
//ARRANQUE CALIBRACION//
while (true){
  if ((digitalRead(BTN)==0) && estado==0){
    delay(100);
    calibracion(); 
    estado=1;
    }
  if ((digitalRead(BTN)==0) && estado==1){
    digitalWrite(LED,1);
    tiempoinicio=millis();
    break;
    }
  }
}

void loop() {
  sensores();
  PID();
  MOTORES();
}

void tiempo(){
  tiempoactual = millis() - tiempoinicio;
  if(tiempoactual>=tiempo1){
    if(tiempoactual>=tiempo2){
      if(tiempoactual>=tiempo3){
        if(tiempoactual>=tiempo4){
          if(tiempoactual>=tiempo5){
           if(tiempoactual>=tiempo6){
            VMAX=170;
            velocidad=VMIN;
            }  
           else{VMAX=90;
            velocidad=VMIN;}
          }
         else{VMAX=90;
          velocidad=VMIN;}
        }
        else{VMAX=90;
         velocidad=VMIN;}
      }
      else{VMAX=90;
       velocidad=VMIN;}
    }
    else{VMAX=170;
     velocidad=VMIN;}
  }
}

void PID(){
  integral = integral + error;
  if (integral > 1000) integral = 1000;
  if (integral < -1000) integral = -1000;
  derivativa = error - errorant;
  correccion = kp * error + ki * integral + kd * derivativa;
  errorant = error;  
}
void sensores(){
  linea = qtr.readLineWhite(sensorValues);
  error = linea - 3500;
}

void calibracion(){
  unsigned long startTime = millis();
  while (millis() - startTime < 5000) { // 5 segundos de calibración
    qtr.calibrate();
    digitalWrite(LED,1);
    delay(50);
    digitalWrite(LED,0);
    delay(50);
  }
}

void motorizq(){
  VIZQ=velocidad-correccion;
  if(VIZQ>VMAX){VIZQ=VMAX;}
  if(VIZQ<0){VIZQ=0;}
  digitalWrite(IN1izq,0);
  digitalWrite(IN2izq,1);
  OCR1A = VIZQ; //PWM IZQ//
}
void motorder(){
  VDER=velocidad+correccion;
  if(VDER>VMAX){VDER=VMAX;}
  if(VDER<0){VDER=0;}
  digitalWrite(IN1der,1);
  digitalWrite(IN2der,0);
  OCR1B = VDER; //PWM DER//
}
void MOTORES(){
  motorder();
  motorizq();
}