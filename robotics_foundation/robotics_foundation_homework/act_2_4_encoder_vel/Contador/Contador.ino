  #define EncA  25 // GPIO para señal A del encoder
#define EncB  26 // GPIO para señal B del encoder

char opcion;
volatile bool BSet = 0;
volatile bool ASet = 0;
long contador = 0;

void IRAM_ATTR Encoder()
{
  BSet = digitalRead(EncB);
  ASet = digitalRead(EncA);
  if (BSet == ASet)
  {
    contador++; 
  }
  else
  {
    contador--;
  }
}

void setup ()
{
  Serial.begin(115200);
  pinMode(EncA, INPUT_PULLUP);    // Señal A del encoder como entrada con pull-up
  pinMode(EncB, INPUT_PULLUP);    // Señal B del encoder como entrada con pull-up
  attachInterrupt(digitalPinToInterrupt(EncA), Encoder, CHANGE); // Asignar la función Encoder a la interrupción de cambio en la señal A
}

void loop()
{
  if (Serial.available() > 0) {
    char opcion = Serial.read();
    if (opcion == 'C') {
      contador = 0;
      Serial.println("Contador reiniciado.");
    }
  }
  Serial.print("Pulsos: ");
  Serial.println(contador);
}
