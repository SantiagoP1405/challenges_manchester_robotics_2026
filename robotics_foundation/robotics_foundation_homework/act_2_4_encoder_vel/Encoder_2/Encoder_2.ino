// Estas constantes no cambiarán. Se utilizan para dar nombre a los pines utilizados:
#define variador 34  // Pin de entrada analógica al que está conectado el potenciómetro
#define EnA  26 // PWM para Puente H
#define In1  23 // Pin de salida digital 
#define In2  22 // Pin de salida digital
#define EncA  25 // GPIO para señal A del encoder
#define EncB  27 // GPIO para señal B del encoder

#define freq 980//Frecuencia de PWM
#define resolution 8 //Resolución de PWM 2^8 = 256
#define PWM1_Ch 0 // Canal de PWM

int32_t tiempo_act = 0, tiempo_ant = 0, delta_tiempo = 2e9;
int pwm = 0;        // valor de salida al PWM
int pot;
char opcion;
float posicion=0, posactual = 0, posanterior = 0, velocidad = 0;
float resolucion = 0.0858983536;  //Definir resolución del encoder
int pulsos = 4191;      //Número de pulsos a la salida del motorreductor
int32_t contador = 0, contaux = 0, revoluciones;
volatile bool BSet = 0;
volatile bool ASet = 0;
volatile bool encoderDirection = false;
bool motorEnMovimiento = false; // Variable de estado para el motor

//Función para realizar la lectura de las señales del encoder de cuadratura y
//definir el sentido de giro del motor
void IRAM_ATTR Encoder()
{
  BSet = digitalRead(EncB);
  ASet = digitalRead(EncA);
  if (BSet == ASet)
  {
    contador++;
    encoderDirection = true;
  }
  //Si ambas señales leídas son distintas, el motor gira en sentido horario
  //y se decrementa un contador para saber el número de lecturas
  else
  {
    contador--;
    encoderDirection = false;
  }
  tiempo_act = micros();
  delta_tiempo = tiempo_act - tiempo_ant;
  tiempo_ant = tiempo_act;
}

//Función para leer los valores de un potenciometro para controlar los motores por PWM
void lectura ()
{
   // leer señal analógica de entrada:
  pot = analogRead(variador);
 
  // mapear el rango de la salida:
  pwm = map(pot, 0, 4095, 0, 255);
 
  // cambiar el valor de salida:
  ledcWrite(PWM1_Ch, pwm);
  //analogWrite(EnA, pwm);
}

//Función detener el motor
void detener ()
{
    
  digitalWrite(In1, LOW);
  digitalWrite(In2, LOW);
  Serial.println("EL motor está detenido");
  delay(250);
}

//Función giro derecha del motor
void derecha ()
{
  digitalWrite(In1, HIGH);
  digitalWrite(In2, LOW);
}

//Función giro izquierda del motor
void izquierda ()
{
  digitalWrite(In1, LOW);
  digitalWrite(In2, HIGH);
}


void pose ()
{
  if (encoderDirection)
  {
    posicion = contador * resolucion; //Convertir a grados
    if (contador >= pulsos) //Contar por revoluciones
    {
      revoluciones++;
      contador = 0;
    }
  }
  else
  {
    posicion = contador * resolucion; //Convertir a grados 
    if (contador <= -pulsos) //Contar por revoluciones
    {
      revoluciones--;
      contador = 0;
    }
  }
  //Cálculo de la velocidad mediante un delta de tiempo  
  velocidad = 60000000/(pulsos * delta_tiempo); //  Convertir a segundos nuevamente multiplicando por 60000000
  //Se pregunta por la velocidad, cuando hay una inversion de giro, para hacerla positiva
  if (velocidad < 0)
    velocidad = abs(velocidad);
  encoderDirection = false; // Reiniciar la variable después de usarla
  
}

void setup ()
{
  
  pinMode(variador, INPUT);
  pinMode(EnA, OUTPUT); 
  pinMode(In1, OUTPUT);
  pinMode(In2, OUTPUT); 

  // configurar funciones PWM
  ledcSetup(PWM1_Ch, freq, resolution);
  ledcAttachPin(EnA, PWM1_Ch);
  
  // inicializar serial a 115200 bps:
  Serial.begin(115200);
  
  pinMode(EncA, INPUT_PULLUP);    // Señal A del encoder como entrada con pull-up
  pinMode(EncB, INPUT_PULLUP);    // Señal B del encoder como entrada con pull-up
  attachInterrupt(digitalPinToInterrupt(EncA), Encoder, CHANGE); // Asignar la función Encoder a la interrupción de cambio en la señal A
    
}

void loop()
{
  lectura();
  if (Serial.available () > 0)
    opcion=Serial.read();
    switch (opcion)
    {
      case 'D':
      motorEnMovimiento = true; // Variable de estado para el motor  
      derecha();
      break;

      case 'I':
      motorEnMovimiento = true; // Variable de estado para el motor  
      izquierda();
      break;

      case 'S':
      motorEnMovimiento = false; // Variable de estado para el motor  
      detener();
      break;
    }
    // Imprimir datos solo si el motor está en movimiento  
    if (motorEnMovimiento) {  
      imprimirdatos();  
    } 
  
  
 
}

void imprimirdatos()
{
  Serial.print ("Posicion:    ");
  pose();
  Serial.print (revoluciones);
  Serial.print ("     Revoluciones  y      " );
  Serial.print (posicion);
  Serial.println ("    Grados" );
  Serial.print ("A una velocidad estimada de:    ");
  Serial.print (velocidad);
  Serial.println ("         RPM");
  Serial.println("");
  delay(100);
}
