/*ZONA DE INCLUSIÓN DE LIBRERIAS*/
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <SoftwareSerial.h>
#include <NewPing.h>


String data="";

/*DEFINICIÓN DE PINES*/
#define rxPin 2                                                       //Pines para el envío se Arduino Uno a NodeMCU.
#define txPin 3

//Sensor de humedad DHT22
#define DHT_PIN 5 // Se define el pin para conectar el sensor DHT22
#define DHT_TIPO DHT22 //Definimos el modelo del sensor DHT22
#define LED 16

//Sensor LDR
#define LDR_PIN A0

//Sensor FC-28
#define sensorFCPin A1 // Sensor FC-28 pin analogico
#define pinfc28V  9 //control de alimentacion
int humedadSuelo = 0;
float valHumsuelo;

//PIR
int pinPir = 8;
int val = 0; //valor que recibimos del pir
int pirEstado = LOW; //estado inicial del pir, no hay movimiento

//pines SensorUltrasonico
#define TRIG_PIN  6      // trigger
#define  ECO_PIN  7      // echo
unsigned long tiempo, distancia;
NewPing sonar(TRIG_PIN, ECO_PIN);


/*DECLARACIÓN DE VARIABLES GLOBALES*/
unsigned int valor_medido;                                      //Valor obtenido al leer por A0.
int ledPin = 10;                                                     //Pin de conexión del led de prueba.
float LR;                                                               //Variable para almacenar la luminosidad relativa.


/*PROTOTIPADO*/
SoftwareSerial Trans(rxPin, txPin);               //Definimos puerto serie virtual. Pines para Rx y Tx.
DHT dht(DHT_PIN, DHT_TIPO);                        //Nombramos y configuramos el sensor para poder inicializarlo.

/*Variables DHT22*/
float t;
float h;

long tdhtx = 0;


void setup()
{
  pinMode(ledPin, OUTPUT);
  pinMode (txPin , OUTPUT);
  Serial.begin(9600);
  Trans.begin(9600);                            //Velocidad de envío para la transmisión al NodeMCU.
  Serial.println("DHT22 test!");
  //Configuramos el los pines FC-28
  pinMode(sensorFCPin, INPUT);
  pinMode(pinfc28V, OUTPUT);
  dht.begin();                                  //Arrancamos el sensor.
}


void loop()
{
  long tiempo = millis();
  //messageData="";


  sensorUltrasonico();
  sensorPir();
  if (tiempo - tdhtx > 2000) { //Envia cada 2 segundos
    tdhtx = tiempo;

    sensorDHT();
    sensorLDR();
    sensorFC28();


    data= "T" + String(t);
    data = data + "H" + String(h);
    data = data + "L" + String(LR);
    data = data + "VHS" + String(valHumsuelo);
    data = data + "U" + String(distancia);
    data = data + "Mov" + String(val);

    Trans.println(data);
    Serial.println(data);

    //    Serial.print("Temperatura: ");
    //    Serial.println(t);
    //    Serial.print("Humedad: ");
    //    Serial.println(h);
    //    Serial.print("Luminosidad: ");
    //    Serial.println(LR);
    //    Serial.print("Humedad Suelo: ");
    //    Serial.println(valHumsuelo);
    //    Serial.print("Distancia: ");
    //    Serial.println(distancia);
  }
}

void sensorDHT() {
  //Enviando humedad y temperatura
  t = dht.readTemperature();
  h = dht.readHumidity();
  if (isnan(h) || isnan(t)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }
}
void sensorLDR() {
  valor_medido = analogRead(LDR_PIN);                      //Valor de luminosidad medido en A0
  LR = (valor_medido * 100.0) / 1023.0;               //Luminosidad medida en tanto por ciento
  if (isnan(LR)) {
    Serial.println(F("Failed to read from LDR sensor!"));
    return;
  }
}

void sensorFC28() {

  //Enviando humedad y temperatura
  //Aplique energía al sensor de humedad del suelo
  digitalWrite(pinfc28V, HIGH);
  delay(10); // espera de 10 milisegundos
  humedadSuelo = analogRead(sensorFCPin);
  // Apague el sensor para reducir la corrosión del metal
  //con el tiempo
  digitalWrite(pinfc28V, LOW);
  //Convertir el valor en porcentaje
  valHumsuelo = map(humedadSuelo, 1023, 0, 0, 100);
  if (isnan(valHumsuelo)) {
    Serial.println(F("Failed to read from FC-28 sensor!"));
    return;
  }
}

void sensorPir()
{
  //Sensor de Movimiento
  val = digitalRead(pinPir);
  //  if (val == HIGH) { //si está activado
  //    //digitalWrite(pinLED,HIGH);//encender LED movimiento
  //    if (pirEstado == LOW) { //si previamente estaba apagado
  //      Serial.println("Sensor activado");
  //      pirEstado = HIGH;
  //    }
  //  } else { //si está desactivado
  //    //digitalWrite(pinLED,LOW);
  //    if (pirEstado == HIGH) { //si previamente estaba encendido
  //      Serial.println("Sensor apagado");
  //      pirEstado = LOW;
  //    }
  //  }
}

void sensorUltrasonico() {

  tiempo = sonar.ping_median(5);//tiempo promedio de ir y venir del trigger
  distancia = tiempo / US_ROUNDTRIP_CM;

  if (isnan(distancia)) {
    Serial.println(F("Failed to read from sensorUltrasonico!"));
    return;
  }
  delay(50);
}
