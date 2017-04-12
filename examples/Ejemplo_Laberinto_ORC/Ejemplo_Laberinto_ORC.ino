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
//Variables
int giro_preferido = 0; // derecha:0 izquierda:1
int giro = 0; //-1 izquierda, 1 derecha;
int modo = 0; //0->rectas,1->interseccion,2->callejon sin salida, 3-> salido del laberinto
int distancia_izq = 0, distancia_der = 0, distancia_rect = 0;
int color = -1;
int corrige = 0;
const int limite_distancia = 25;
const int limite_desviacion = 25;
const int limite_salida = 150;
const float escalado_giro = 0.05; //AJUSTAR!
const float escalado_recta = 0.05;
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
}

void loop() {
  //leemos sensores
  Serial.print("Modo: ");
  Serial.println(modo);
  int* resultado_medida;
  resultado_medida = barrido(90, 2);
  color = lee_color_calibrado(25);
  //guardamos variables
  distancia_izq = resultado_medida[1];
  distancia_rect = resultado_medida[3];
  distancia_der = resultado_medida[5];
  Serial.print("I: ");
  Serial.print(distancia_izq);
  Serial.print(" R: ");
  Serial.print(distancia_rect);
  Serial.print(" D: ");
  Serial.println(distancia_der);
  int desviacion_lateral = distancia_izq - distancia_der;
  //He de ir recto?
  if ((abs(desviacion_lateral) <= limite_desviacion) && (distancia_izq < limite_distancia) && (distancia_der < limite_distancia))
  {
    modo = 0;
    corrige = 0;
  }
  //Me estoy yendo de lado?
  if ((abs(desviacion_lateral) > limite_desviacion) && (distancia_izq < limite_distancia) && (distancia_der < limite_distancia))
  {
    modo = 0;
    corrige = 1;
  }
  //Giro derecha:
  if ((distancia_der >= limite_distancia) && (distancia_rect < limite_distancia) && (distancia_izq < limite_distancia))
  {
    modo = 1;
    giro=1;
  }
  //Giro izquierda:
  if ((distancia_der < limite_distancia) && (distancia_rect >= limite_distancia) && (distancia_izq >= limite_distancia))
  {
    modo = 1;
    giro=0;
  }
  // bifurcacion derecha:
  if ((distancia_der >= limite_distancia) && (distancia_rect >= limite_distancia) && (distancia_izq < limite_distancia))
  {
    modo = 1;
    if (giro_preferido)
    {
      giro = 1;
    }
    else
    {
      modo = 0;
    }
  }
  // bifurcacion izquierda:
  if ((distancia_der < limite_distancia) && (distancia_rect >= limite_distancia) && (distancia_izq >= limite_distancia))
  {
    modo = 1;
    if (giro_preferido)
    {
      modo = 0;
    }
    else
    {
      giro = -1;
    }
  }
  // bifurcacion T:
  if ((distancia_der >= limite_distancia) && (distancia_rect < limite_distancia) && (distancia_izq >= limite_distancia))
  {
    modo = 1;
    if (giro_preferido)
    {
      giro = 1;
    }
    else
    {
      giro = -1;
    }
  }
  // cruce en +:
  if ((distancia_der >= limite_distancia) && (distancia_rect >= limite_distancia) && (distancia_izq >= limite_distancia) && (distancia_izq < limite_salida) && (distancia_rect < limite_salida) && (distancia_der < limite_salida))
  {
    modo = 1;
    if (giro_preferido)
    {
      giro = 1;
    }
    else
    {
      giro = -1;
    }
  }
  //He salido ya??

  if ((distancia_izq >= limite_salida) && (distancia_rect >= limite_salida) && (distancia_der >= limite_salida))
  {
    modo = 3;
  }
  //Lógica de movimiento
  switch (modo)
  {
    case 0: //sigue recto 10 cm
      if (corrige)
      {
        if(desviacion_lateral>0)
        {
        motores(250, 255, limites_motores);
        delay(10/escalado_recta);
        motores(0, 0, limites_motores);
        }
        if(desviacion_lateral<0)
        {
        motores(255, 250, limites_motores);
        delay(10/escalado_recta);
        motores(0, 0, limites_motores);
        }
      }
      recto(10);
      break;
    case 1: //gira según lo que toque
      if (giro == -1)
      {
        gira(90);
      }
      if (giro == 1)
      {
        gira(-90);
      }
      break;
    case 2: //da media vuelta
      gira(180);
      break;
    case 3:
      while (1)
      {
        LEDs(true, true, true);
        delay(250);
        LEDs(false, false, false);
        delay(250);
      }
      break;
  }
}



void gira(int angulo)
{
  if (angulo > 0)
  {
    motores(255, -255, limites_motores);
    delay(angulo / escalado_giro);
    motores(0, 0, limites_motores);
  }
  else if (angulo < 0)
  {
    motores(-255, 255, limites_motores);
    delay(-angulo / escalado_giro);
    motores(0, 0, limites_motores);
  }
}

void recto( int distancia)
{
  if (distancia > 0)
  {
    motores(255, 255, limites_motores);
    delay(distancia / escalado_recta);
    motores(0, 0, limites_motores);
  }
  else if (distancia < 0)
  {
    motores(-255, -255, limites_motores);
    delay(-distancia / escalado_recta);
    motores(0, 0, limites_motores);
  }
}
