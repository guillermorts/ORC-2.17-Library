#include <HC_SR04.h>
#include <ORC.h>
// Ejemplo Librería para el kit de robótica de la ORC 2.16 Organizada por Makers UPV
// http://www.makersupv.com
// Incluye la librería de Ultrasonidos HC-SR04: http://playground.arduino.cc/Code/SR04
// Codificado por Guillermo Orts y por Germán Torres

//Límites de motores
const int MOT_L_MIN = -255;
const int MOT_L_MAX = 255;
const int MOT_R_MIN = -255;
const int MOT_R_MAX = 255;
int limites_motores[4];
//Calibracion de colores
int ROJO_CAL[10];
int VERDE_CAL[10];
int AZUL_CAL[10];
int BLANCO_CAL[10];
int NEGRO_CAL[10];
volatile bool S_1[2];
volatile bool S[2];
int limite = 0;
int ObjetoCercano = 50;

extern Servo servoRobot;

void setup() {
  //Inicializa la librería
  inicializar();
  //Metemos en el vector de límites los límites de los motores;
  limites_motores[0] = MOT_L_MIN;
  limites_motores[1] = MOT_L_MAX;
  limites_motores[2] = MOT_R_MIN;
  limites_motores[3] = MOT_R_MAX;
  //Metemos los colores en sus vectores
  ROJO_CAL[0] = 859;
  ROJO_CAL[1] = 695;
  ROJO_CAL[2] = 665;
  VERDE_CAL[0] = 707;
  VERDE_CAL[1] = 755;
  VERDE_CAL[2] = 675;
  AZUL_CAL[0] = 709;
  AZUL_CAL[1] = 665;
  AZUL_CAL[2] = 719;
  BLANCO_CAL[0] = 866;
  BLANCO_CAL[1] = 831;
  BLANCO_CAL[2] = 824;
  NEGRO_CAL[0] = 657;
  NEGRO_CAL[1] = 627;
  NEGRO_CAL[2] = 632;
  //Realiza cálculos para asegurarse de que no interrumpe el programa constantemente:
  inicializar_calibracion_color(ROJO_CAL, VERDE_CAL, AZUL_CAL, BLANCO_CAL, NEGRO_CAL);
  //Espera a que se pulse el botón de inicio:
  limite = 250;
  espera();
  S[0] = true;
  S[1] = true;
}

void loop() {
  int* valores_distancia;
  volatile int Dist_Max[2];
  volatile int Dist_Min[2];
  volatile int Distancia;


  valores_distancia = extremos();
  Dist_Max[0] = valores_distancia[2]; //Angulo
  Dist_Max[1] = valores_distancia[3]; //Cm
  Dist_Min[0] = valores_distancia[0]; //Angulo
  Dist_Min[1] = valores_distancia[1]; //Cm

  if (Dist_Min[1] > ObjetoCercano) { //Bucle para cuando estamos buscando al otro robot
    motores(255, 255, limites_motores);
  }
  else
  { //Bucle pra cuando encontramos al otro robot
    if (Dist_Min[0] < 90) { //El robot esta a la izquierda
      servoRobot.write(90); //Centramos el ultrasonidos
      Distancia = lee_distancia();
      while (Distancia > 100) { //Si no detectamos nada cerca
        Distancia = lee_distancia();
        motores(0, 255, limites_motores); //Giramos hacia a la izquierda
        delay(100);
        motores(0, 0, limites_motores); //Giramos hacia a la izquierda
      }
      Dist_Min[0] = 0;
    }
    if (Dist_Min[0] > 90) { //El robot esta a la derecha
      servoRobot.write(90);//Centramos el ultrasonidos
      Distancia = lee_distancia();
      while (Distancia > 100) { //Si no detectamos nada cerca
        Distancia = lee_distancia();
        motores(255, 0, limites_motores); //Giramos hacia a la izquierda
        delay(100);
        motores(0, 0, limites_motores); //Giramos hacia a la izquierda
      }
      Dist_Min[0] = 0;
    }
    else {
      servoRobot.write(90);
      Distancia = lee_distancia();
      Dist_Min[0] = 0;
    }
    while (Distancia < 75 && S[0] && S[1]) {
      Distancia = lee_distancia();
      bool* S_temp;
      S_temp = lee_linea_bin(limite); //Le pasamos los nuevos valores de lectura de los sensores
      S[0] = S_temp[0];
      S[1] = S_temp[1];
      motores(255, 255, limites_motores); //Embestimos
    }
    if (!S[0] || !S[1]) {
      motores(-255, -255, limites_motores);
      delay(250);
    }
  }
  while (S[0] && S[1])
  {
    bool* S_temp;
    S_temp = lee_linea_bin(limite); //Le pasamos los nuevos valores de lectura de los sensores
    S[0] = S_temp[0];
    S[1] = S_temp[1];
    
  }
  //En el caso de que detectemos linea negra
  motores(-255, -255, limites_motores);
  delay(100);
  motores(255, 0, limites_motores); //Giramos a la derecha para seguir buscando
  delay(250);
  motores(0, 0, limites_motores);
}
