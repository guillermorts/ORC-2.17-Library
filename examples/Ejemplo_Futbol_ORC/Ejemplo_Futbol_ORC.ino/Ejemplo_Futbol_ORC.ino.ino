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
int v_i = 0, v_d = 0;
int potx, poty;
void setup() {
  //Inicializa la librería
  inicializar();
  Serial.setTimeout(100);//Ponemos el límite de tiempo para el timeout bajo
  limites_motores[0] = MOT_L_MIN; //Metemos en el vector de límites los límites de los motores;
  limites_motores[1] = MOT_L_MAX;
  limites_motores[2] = MOT_R_MIN;
  limites_motores[3] = MOT_R_MAX;

}

void loop() {
  String recibido = "";
  int indice = 0, indice_1 = 0, i = 0;
  int valores_recibidos[5];
  //Espera a que le llegue algo al puerto serie
  while (!Serial.available());
  recibido = Serial.readStringUntil('\n');
  while (indice < recibido.length())
  { indice_1 = indice + 1;
    indice = recibido.indexOf(',', indice_1);
    valores_recibidos[i] = recibido.substring(indice_1, indice).toInt();
    i++;
  }
  //En valores_recibidos se encuentran las variables para usar: X1,Y1,X2,Y2,interruptor
  //Aquí usamos solo los valores de X e Y
  potx = map(valores_recibidos[0], 0, 1023, -255, 255);
  poty = map(valores_recibidos[1], 0, 1023, -255, 255)
         motores(potx + poty, potx - poty, limites_motores);
}
