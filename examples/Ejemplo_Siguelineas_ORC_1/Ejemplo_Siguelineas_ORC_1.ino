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
volatile int V_D = 0;
volatile int V_I = 0;
bool salido=false;

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
  S_1[0] = S[0];
  S_1[1] = S[1]; //Le pasamos los valores de la lectura anterior a la variable
  bool* S_temp;
  S_temp = lee_linea_bin(limite); //Le pasamos los nuevos valores de lectura de los sensores
  S[0] = S_temp[0];
  S[1] = S_temp[1];
  Serial.print("L: ");
  Serial.print(S[0]);
  Serial.print(" R: ");
  Serial.println(S[1]);
  if (!salido)
  {
    if (S[0] && S[1] && S_1[0] && S_1[1]) // RECTO
    {
      V_D = 255;
      V_I = 255;
    }
    if (!S[0] && S[1] && S_1[0] && S_1[1]) //GIRO IZQ 1
    {
      V_D = 255;
      V_I = 0;
    }
    if (!S[0] && S[1] && !S_1[0] && S_1[1]) //GIRO IZQ 2
    {
      V_D = 255;
      V_I = 0;
    }
    if (S[0] && !S[1] && S_1[0] && S_1[1]) // GIRO DER 1
    {
      V_D = 0;
      V_I = 255;
    }
    if (S[0] && !S[1] && S_1[0] && !S_1[1]) // GIRO DER 2
    {
      V_D = 0;
      V_I = 255;
    }
    if (!S[0] && S[1] && S_1[0] && !S_1[1]) // GIR IZQ DESDE DER
    {
      V_D = 255;
      V_I = 0;
    }
    if (S[0] && !S[1] && !S_1[0] && S_1[1]) // GIR DER DESDE IZQ
    {
      V_D = 0;
      V_I = 255;
    }
    if (!S[0] && !S[1] && !S_1[0] && !S_1[1]) // ATRAS 1
    {
      V_D = -255;
      V_I = -255;
      salido = true;
    }
    if (!S[0] && !S[1] && S_1[0] && S_1[1]) //ATRAS 2
    {
      V_D = -255;
      V_I = -255;
      salido = true;
    }
  }
  else
  {
    V_I = -255;
    V_D = -255;
    //if ((!S[0] && S_1[0]) || (!S[1] && S_1[1])) //Si vuelve a pisar linea...
    if (!S[0]||!S[1])//Si vuelve a pisar linea...
    {
      salido = false;
    }
  }
motores(V_I, V_D, limites_motores);

}
