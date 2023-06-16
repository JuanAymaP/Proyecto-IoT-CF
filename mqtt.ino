#include <WiFi.h>
#include <PubSubClient.h>
#include "DHT.h"

//Quitar las barras del sensor que corresponda
#define DHTTYPE DHT11   // DHT 11
//#define DHTTYPE DHT21   // DHT 21 (AM2301)
//#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321

//Escribir los datos de conexion
const char* ssid = "MICASAKC";
const char* password = "KCS860816";

//Incluir la direccion del servidor MQTT
const char* mqtt_server = "192.168.1.39";

// Conectamos como cliente al Servidor
WiFiClient espClient;
PubSubClient client(espClient);

// Pin de entrada para DHT
const int DHTPin = 14;

// Pin luces pasillo
const int lamp2 = 13;

// Pin luces comedor
const int lamp = 25;

//byte sensorPin = 27;
byte indicator = 26;

// Inicializamos DHT sensor.
DHT dht(DHTPin, DHTTYPE);

// Tiempo para las variables
long now = millis();
long lastMeasure = 0;

void setup_wifi() {

  delay(10);
  Serial.println();
  Serial.print("Conectando a: ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("WiFi conectado - ESP IP : ");
  Serial.println(WiFi.localIP());
}

void callback(String topic, byte* message, unsigned int length) 
{
  //Serial.print("Mensajes llegado al topic: ");
  //Serial.print(topic);
  //Serial.print(". Mensaje: ");
  String messageTemp;

  for (int i = 0; i < length; i++) {
    //Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  //Serial.println();

  if (topic == "esp32") {
    //Serial.print("Comedor: ");
    if (messageTemp == "on") {
      digitalWrite(lamp, HIGH);
      //Serial.print("Encendido");
    }
    else if (messageTemp == "off") {
      digitalWrite(lamp, LOW);
      //Serial.print("Apagado");
    }
  }
  Serial.println();
}

void reconnect() {

  while (!client.connected()) {
    Serial.print("Intentando conectar a MQTT...");

    if (client.connect("ESP32Client")) {
      Serial.println("conectado");

      client.subscribe("esp32");
    } else {
      Serial.print("fallo, rc=");
      Serial.print(client.state());
      Serial.println(" volveremos a conectar en 5 segundos");
      digitalWrite(2, LOW);
      delay(5000);
    }
  }
}

void setup() {

  //pinMode(sensorPin, INPUT);
  pinMode(indicator, OUTPUT);
  pinMode(lamp, OUTPUT);
  dht.begin();

  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

}

void loop() {

  if (!client.connected()) {
    reconnect();
  }
  if (!client.loop())
    client.connect("ESP32Client");

  now = millis();
  if (now - lastMeasure > 5000) {
    lastMeasure = now;



    // float hif = dht.computeHeatIndex(f, h);
    // static char temperatureTemp[7];
    // dtostrf(hic, 6, 2, temperatureTemp);



    //byte state = digitalRead(sensorPin);
    //digitalWrite(indicator, state);
    //if (state == 1)Serial.println("Movimiento");
    //else if (state == 0)Serial.println("Nada");
    //delay(500);

  }
}
