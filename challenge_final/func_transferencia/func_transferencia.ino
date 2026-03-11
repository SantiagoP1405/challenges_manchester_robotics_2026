// ============================================================
//  ESP32 - Motor Identification via ROS2 Serial
//  Basado en el código original del usuario.
//
//  Cambios respecto al original:
//    - Se elimina el potenciómetro (pin 34)
//    - El PWM se recibe por Serial desde ROS2
//    - Protocolo entrada : "CMD:<-255 a 255>\n"  (entero con signo)
//                          "D\n"  → giro derecha
//                          "I\n"  → giro izquierda
//                          "S\n"  → detener
//    - Protocolo salida  : "DATA:<pwm_norm>,<velocidad>,<posicion>\n"
//      (mismo formato que espera el nodo ROS2)
// ============================================================

// ── Pines ─────────────────────────────────────────────────────
#define EnA  26   // PWM para Puente H
#define In1  23   // Pin de salida digital
#define In2  22   // Pin de salida digital
#define EncA 25   // GPIO para señal A del encoder
#define EncB 27   // GPIO para señal B del encoder

// ── Configuración PWM (LEDC) ──────────────────────────────────
#define freq       980   // Frecuencia de PWM (igual que original)
#define resolution 8     // Resolución 8 bits → 0-255
#define PWM1_Ch    0     // Canal PWM

// ── Encoder ───────────────────────────────────────────────────
float   resolucion  = 0.0858983536;  // grados por pulso
int     pulsos      = 4191;          // pulsos por vuelta a la salida

int32_t tiempo_act   = 0;
int32_t tiempo_ant   = 0;
int32_t delta_tiempo = 2000000000;   // valor inicial grande para evitar /0
int32_t contador     = 0;
int32_t revoluciones = 0;
float   posicion     = 0;
float   velocidad    = 0;

volatile bool BSet            = false;
volatile bool ASet            = false;
volatile bool encoderDirection = false;

// ── Estado motor ──────────────────────────────────────────────
bool motorEnMovimiento = false;
int  pwm      = 0;   // magnitud PWM actual (0-255)
int  pwm_signo = 1;  // +1 derecha, -1 izquierda

// ── Temporización envío de datos ──────────────────────────────
unsigned long lastSendTime   = 0;
const int     SEND_PERIOD_MS = 20;   // 50 Hz → suficiente para rqt_plot

// ─────────────────────────────────────────────────────────────
//  ISR Encoder (tu lógica original intacta)
// ─────────────────────────────────────────────────────────────
void IRAM_ATTR Encoder()
{
  BSet = digitalRead(EncB);
  ASet = digitalRead(EncA);

  if (BSet == ASet)
  {
    contador++;
    encoderDirection = true;
  }
  else
  {
    contador--;
    encoderDirection = false;
  }

  tiempo_act   = micros();
  delta_tiempo = tiempo_act - tiempo_ant;
  tiempo_ant   = tiempo_act;
}

// ─────────────────────────────────────────────────────────────
//  Funciones de movimiento (tu código original)
// ─────────────────────────────────────────────────────────────
void derecha()
{
  digitalWrite(In1, HIGH);
  digitalWrite(In2, LOW);
}

void izquierda()
{
  digitalWrite(In1, LOW);
  digitalWrite(In2, HIGH);
}

void detener()
{
  digitalWrite(In1, LOW);
  digitalWrite(In2, LOW);
  ledcWrite(PWM1_Ch, 0);
  pwm               = 0;
  motorEnMovimiento = false;
}

// ─────────────────────────────────────────────────────────────
//  pose() — tu lógica original intacta
// ─────────────────────────────────────────────────────────────
void pose()
{
  if (encoderDirection)
  {
    posicion = contador * resolucion;
    if (contador >= pulsos)
    {
      revoluciones++;
      contador = 0;
    }
  }
  else
  {
    posicion = contador * resolucion;
    if (contador <= -pulsos)
    {
      revoluciones--;
      contador = 0;
    }
  }

  // Velocidad en RPM mediante delta de tiempo entre pulsos
  velocidad = 60000000.0 / ((float)pulsos * (float)delta_tiempo);
  if (velocidad < 0)
    velocidad = abs(velocidad);
    //velocidad = round(velocidad);

  encoderDirection = false;
}

// ─────────────────────────────────────────────────────────────
//  Aplicar PWM con signo recibido por Serial
//  pwm_signed: -255 (izquierda full) a +255 (derecha full)
// ─────────────────────────────────────────────────────────────
void aplicarPWM(int pwm_signed)
{
  pwm_signed = constrain(pwm_signed, -255, 255);

  if (pwm_signed == 0)
  {
    detener();
    return;
  }

  motorEnMovimiento = true;

  if (pwm_signed > 0)
  {
    pwm_signo = 1;
    pwm       = pwm_signed;
    derecha();
  }
  else
  {
    pwm_signo = -1;
    pwm       = -pwm_signed;
    izquierda();
  }

  ledcWrite(PWM1_Ch, pwm);
}

// ─────────────────────────────────────────────────────────────
//  imprimirdatos() — envía datos a ROS2 en formato DATA:
// ─────────────────────────────────────────────────────────────
void imprimirdatos()
{
  pose();

  // Formato parseado por el nodo ROS2:
  //   DATA:<pwm_normalizado>,<velocidad_rpm>,<posicion_grados>
  float pwm_norm = (float)(pwm_signo * pwm) / 255.0f;

  Serial.print("DATA:");
  Serial.print(pwm_norm, 4);
  Serial.print(",");
  Serial.print(velocidad, 3);
  Serial.print(",");
  Serial.println(posicion);
}

// ─────────────────────────────────────────────────────────────
//  Setup
// ─────────────────────────────────────────────────────────────
void setup()
{
  pinMode(EnA, OUTPUT);
  pinMode(In1, OUTPUT);
  pinMode(In2, OUTPUT);

  // Configurar canal PWM (LEDC)
  ledcSetup(PWM1_Ch, freq, resolution);
  ledcAttachPin(EnA, PWM1_Ch);

  // Encoder con pull-up interno
  pinMode(EncA, INPUT_PULLUP);
  pinMode(EncB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(EncA), Encoder, CHANGE);

  Serial.begin(115200);
  while (!Serial) delay(10);

  detener();
  Serial.println("READY");
}

// ─────────────────────────────────────────────────────────────
//  Loop
// ─────────────────────────────────────────────────────────────
void loop()
{
  // ── Leer comandos enviados por el nodo ROS2 ──────────────
  if (Serial.available() > 0)
  {
    String linea = Serial.readStringUntil('\n');
    linea.trim();

    if (linea.startsWith("CMD:"))
    {
      // Formato principal: "CMD:<entero con signo>"
      // Ejemplo: "CMD:128"  → 50% PWM derecha
      //          "CMD:-200" → ~78% PWM izquierda
      int val = linea.substring(4).toInt();
      aplicarPWM(val);
    }
    else if (linea == "D")
    {
      // Comando simple de dirección (compatible con protocolo original)
      motorEnMovimiento = true;
      derecha();
    }
    else if (linea == "I")
    {
      motorEnMovimiento = true;
      izquierda();
    }
    else if (linea == "S")
    {
      detener();
    }
    else if (linea == "RESET_ENC")
    {
      // Reiniciar contadores del encoder
      noInterrupts();
      contador     = 0;
      revoluciones = 0;
      posicion     = 0;
      interrupts();
    }
  }

  // ── Enviar datos a ROS2 cada SEND_PERIOD_MS ─────────────
  unsigned long now = millis();
  if (motorEnMovimiento && (now - lastSendTime >= SEND_PERIOD_MS))
  {
    imprimirdatos();
    lastSendTime = now;
  }
}
