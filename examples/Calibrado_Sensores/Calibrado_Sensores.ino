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

//Calibracion de colores
int ROJO_CAL[10];
int VERDE_CAL[10];
int AZUL_CAL[10];
int BLANCO_CAL[10];
int NEGRO_CAL[10];

const int limite = 512; //Límite para distinguir línea de blanco

#define tipo_sensor 6//0: Colores/1: IR Lineas analógico/2: Líneas digital/3: Ultrasonidos/4: barrido/5: extremos/6: colores_calibrado
int retardo_muestras = 100;

void setup() {
//Inicializa la librería
  inicializar();
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

  Serial.println("Calibrado de Sensores");
}

void loop() {
  int distancia = 0;
  int* colores;
  bool* linea_b;
  int* linea;
  int* valores_distancias;
  int color_leido;
  switch (tipo_sensor)
  {
    case 0: //Colores
      colores = lee_color();
      Serial.print("R: ");
      Serial.print(colores[0]);
      Serial.print(" G: ");
      Serial.print(colores[1]);
      Serial.print(" B: ");
      Serial.print(colores[2]);
      break;

    case 1: //Linea analógico

      linea = lee_linea();
      Serial.print("Sensor L:");
      Serial.print(linea[0]);
      Serial.print(" Sensor R:");
      Serial.print(linea[1]);
      break;

    case 2:  //Linea digital

      linea_b = lee_linea_bin(limite);
      Serial.print("Sensor L:");
      Serial.print(linea_b[0]);
      Serial.print(" Sensor R:");
      Serial.print(linea_b[1]);
      break;

    case 3://Distancia

      distancia = lee_distancia();
      Serial.print("Distancia: ");
      Serial.print(distancia);
      Serial.print(" cm");
      break;

    case 4://Barrido
      valores_distancias = barrido(90, 4);
      for (int i = 0; i < 14; i++)
      {
        if ((i % 2) == 0)
        {
          Serial.print("Angulo: ");
        }
        else
        {
          Serial.print("Distancia: ");
        }
        Serial.print(valores_distancias[i]);
        Serial.println("");
      }
      break;
    case 5:
      valores_distancias = extremos();
      Serial.print("Min pos: ");
      Serial.print(valores_distancias[0]);

      Serial.print(" Min: ");
      Serial.print(valores_distancias[1]);

      Serial.print(" Max pos: ");
      Serial.print(valores_distancias[2]);

      Serial.print(" Max: ");
      Serial.print(valores_distancias[3]);
      break;
    case 6:
      color_leido = lee_color_calibrado(25); //Tolerancia recomendada según el calibrado
      switch (color_leido)
      {
        case -1:
          Serial.print("No se reconoce el color...");
          break;
        case 0:
          Serial.print("El color es rojo.");
          break;
        case 1:
          Serial.print("El color es verde.");
          break;
        case 2:
          Serial.print("El color es azul.");
          break;
        case 3:
          Serial.print("El color es blanco.");
          break;
        case 4:
          Serial.print("El color es negro.");
          break;
      }
      break;
  }
  Serial.println();
  delay(retardo_muestras); //Espera 1 segundo
}
