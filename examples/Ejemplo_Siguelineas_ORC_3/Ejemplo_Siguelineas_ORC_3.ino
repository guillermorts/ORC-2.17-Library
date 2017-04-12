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
//variables del programa
volatile int S_1[2];
volatile int S[2];
volatile int V_D = 255;
volatile int V_I = 255;
volatile float K_P = 200.0;
volatile float K_D = 50.0;
volatile float K_I = 0;
volatile float error = 0.0;
volatile float error_1 = 0.0;
volatile float error_d = 0.0;
volatile float error_i = 0.0;
volatile float control = 0.0;
volatile long tini = 0, temp = 0;
volatile int maxIR = 370;
volatile int minIR = 70;
volatile int V_avance=255;
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
  espera();
  S[0] = 0;
  S[1] = 0;
  tini = micros();
}

bool DEBUG_LECTURA = true;
bool DEBUG_ERRORES = false;
bool DEBUG_PID = false;
bool DEBUG_SALIDA = false;

void loop() {
  //Lectura de valores de sensores
  S_1[0] = S[0];
  S_1[1] = S[1]; //Le pasamos los valores de la lectura anterior a la variable
  int* S_temp;
  S_temp = lee_linea(); //Le pasamos los nuevos valores de lectura de los sensores
  S[0] = S_temp[0];
  S[1] = S_temp[1];
  if (DEBUG_LECTURA) {
    Serial.print("L: ");
    Serial.print(S[0]);
    Serial.print(" R: ");
    Serial.println(S[1]);
  }

  //Temporización;
  temp = micros() - tini;
  tini = micros();
  //Cálculo del error
  error_1 = error; //almacenamos el error anterior
  error = (2.0 * (float)(S[0] - ((maxIR + minIR) / 2)) / (float)(maxIR - minIR));
  error_d = ((error - error_1) / (temp) ) * 1000000.0;
  error_i += ((error + error_1) / 2.0 ) * ((temp) / 1000000.0);
  if (DEBUG_ERRORES) {
    Serial.print("e: ");
    Serial.print(error);
    Serial.print(" e_i: ");
    Serial.print(error_i);
    Serial.print(" e_d: ");
    Serial.println(error_d);
  }
  //Cálculo de la acción de control
  control = K_P * error + K_I * error_i + K_D * error_d;
  if (DEBUG_PID) {
    Serial.print("p: ");
    Serial.print(K_P * error);
    Serial.print(" i: ");
    Serial.print(K_I * error_i);
    Serial.print(" d: ");
    Serial.println(K_D * error_d);
  }
  //Cálculo de los valores de salida;
  V_I = constrain(V_avance - (int)(control), -255, 255);
  V_D = constrain(V_avance + (int)(control), -255, 255);
  if (DEBUG_SALIDA)
  {
    Serial.print("L: ");
    Serial.print(V_I);
    Serial.print(" R: ");
    Serial.println(V_D);
  }
  motores(V_I, V_D, limites_motores);

}
