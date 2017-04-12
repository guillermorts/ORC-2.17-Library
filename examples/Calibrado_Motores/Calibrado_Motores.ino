#include <HC_SR04.h>
#include <ORC.h>
// Ejemplo Librería para el kit de robótica de la ORC 2.16 Organizada por Makers UPV
// http://www.makersupv.com
// Incluye la librería de Ultrasonidos HC-SR04: http://playground.arduino.cc/Code/SR04
// Codificado por Guillermo Orts y por Germán Torres

//Límites de motores
const int MOT_L_MIN=-255;
const int MOT_L_MAX=255;
const int MOT_R_MIN=-255;
const int MOT_R_MAX=255;
char recibido;
int limites_motores[4];
/*
  Esta función espera por puerto serie una cadena de forma:
  FX,Y //Mover hacia delante
  BX,Y //Mover hacia detrás
  dónde X e Y son los valores de los motores izquiero y derecho
  respectivamente para calibrar que el robot vaya recto.
  Los valores se guardan y codifican en las constantes MOT_L_MIN, MOT_L_MAX,
  MOT_R_MIN y MOT_R_MAX.
*/
void setup() {
  //Inicializa la librería
  inicializar();
  Serial.setTimeout(100);//Ponemos el límite de tiempo para el timeout bajo
limites_motores[0]=MOT_L_MIN;  //Metemos en el vector de límites los límites de los motores;
limites_motores[1]=MOT_L_MAX;
limites_motores[2]=MOT_R_MIN;
limites_motores[3]=MOT_R_MAX;
Serial.println("Preparado!");
}

void loop() {
  //Espera a que le llegue algo al puerto serie
  while (!Serial.available());
  recibido = Serial.read(); //Lee el primer caracter
  if (recibido == 'F') //Si detecta "F"
  { int L;
    int R;
    Serial.println("Recibido F");
    L =int( Serial.parseInt()); //Lee primer valor
    R =int( Serial.parseInt()); //Lee segundo valor
    Serial.print("Rueda L: ");
    Serial.print(L);
    Serial.print(" Rueda R: ");
    Serial.println(R);
    motores(L, R, limites_motores);
  }
  else if (recibido == 'B')
  {
    int L;
    int R;
    Serial.println("Recibido B");
    L =int( Serial.parseInt()); //Lee primer valor
    R =int( Serial.parseInt()); //Lee segundo valor
    Serial.print("Rueda L: ");
    Serial.print(-L);
    Serial.print(" Rueda R: ");
    Serial.println(-R);
    motores(-L, -R,limites_motores);
  }
}
