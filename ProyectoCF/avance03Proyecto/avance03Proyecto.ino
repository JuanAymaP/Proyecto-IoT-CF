//#include <WiFi.h> // Con el ESP8266: #include <ESP8266WiFi.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Ultrasonic.h>  

//humidity sensor
#include <DHT.h> //Cargamos la librería DHT tambien necesitas la libreria ADAFRUIT UNIFIED SENSOR
#define DHT_PIN D4 // Se define el pin GPIO2 = D4  ESP8266 para conectar el sensor DHT22
#define DHT_TIPO DHT22 //Definimos el modelo del sensor DHT22
#define LED 16

//Sensor Humedad
const int sensorPin = A0; // Sensor FC-28
int humedadSuelo = 0;
//sensorHumedadSuelo
int pinfc28 = D1;
//PIR
int pinPir=D2;
//pines SensorDistancia
int TRIG = D2;      // trigger en pin 10
int ECO = D3;      // echo en pin 9
unsigned int DISTANCIA;
Ultrasonic ultrasonic(TRIG, ECO);
int val=0;//valor que recibimos del pir
int pirEstado=LOW; //estado inicial del pir, no hay movimiento


DHT dht(DHT_PIN, DHT_TIPO); //Crea objeto sensorTH

//Configuracion del user
const char* ssid = "redmi";
const char* password = "rod963XD";

char data_temp[12] = "";
char data_humi[12] = "";
char data_humiSuelo[12] = "";
char data_digital[12] = "";
char data_dist[12] = "";
//String messageData;

//IP del servidor BROKER
const char *mqtt_broker = "192.168.202.11";

//const char *mqtt_broker = "15.229.78.220";
const int mqtt_port = 1883;
//const char* mqtt_server="test.mosquitto.org";


/*
  // MQTT Broker
  const char *mqtt_broker = "broker.emqx.io";
  const char *mqtt_username = "emqx";
  const char *mqtt_password = "publico";
  const int mqtt_port = 1883;
*/

//Inicializamos el nodeMCU para wifi y Mqtt
WiFiClient esp82;
PubSubClient client(esp82);

long lastMsg = 0;
long tdhtx = 0;
char msg[50];
int value = 0;

//********************Funciones para Mqtt******************

//--------------------Configuración Wifi-------------------
void config_wifi() {
  //Conectandose a una red Wifi
  Serial.println("Conectandose a la red ...");
  Serial.println(ssid);
  WiFi.begin(ssid, password);

  // Mientras el ESP82 no se conecte al AP:
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("ESP82 conectado, su IP es: ");
  Serial.println(WiFi.localIP());
}
//--------------------Lectura de datos que llegan despues de suscribir-------------------
void callback(String topic, byte* message, unsigned int length) {
  //Serial.println("Mensaje que llega del topic: ");
  Serial.print("Mensaje que llega del topic[");
  Serial.print(topic);
  Serial.print("] ");
  //Serial.println(". Message: ");
  String messageData = "";

  for (int i = 0; i < length; i++) {
    //Serial.println((char)message[i]);
    Serial.print((char)message[i]);
    messageData += (char)message[i];
  }
  Serial.println();
  Serial.print("Tamaño del mensaje :");
  Serial.println(length);
  Serial.println();
  Serial.println("-----------------------");
  Serial.println(messageData);

  //ACTUADORES EL LED
  if (messageData == "ON") {
    Serial.println("LED");
    digitalWrite(LED, HIGH);
  }
  else {
    digitalWrite(LED, LOW);
  }

}

//--------------------Conexión a broker/ Suscribir-------------------
void reconnect() {
  while (!client.connected()) {
    Serial.println("Iniciando conexión con Broker...");
    String clienteId = "ESP82";
    if (client.connect(clienteId.c_str())) {
      //if (client.connect(clienteId.c_str(), mqtt_username, mqtt_password)) {
      client.subscribe("esp82iot");//topic para suscribir
      //client.subscribe("esp82iot/");//topic para suscribir
      Serial.println("Conexión exitosa");
    }
    else {
      Serial.print("Failed, rc=");
      Serial.print(client.state());
      Serial.print(" esperando 3 segundos");
      delay(3000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  //LED
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);

  //Configuramos el los pins
  pinMode(A0, INPUT);
  pinMode(pinfc28, OUTPUT);
//pinMode(pinPir, INPUT);
  config_wifi();
  dht.begin(); //Inicio del DHTxx
  //client.setServer(mqtt_broker,1883);//Conexión al servidor/Broker
  client.setServer(mqtt_broker, mqtt_port);
  client.setCallback(callback);//Llamada a los callback para ver si recibo mensajes del broker
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  long tiempo = millis();
  //messageData="";

  
  if (tiempo - tdhtx > 2000) { //Envia cada 2 segundos
    tdhtx = tiempo;

    //Enviando humedad y temperatura
    float t = dht.readTemperature();
    float h = dht.readHumidity();
    DISTANCIA = ultrasonic.read();
    Serial.print("FFF:  ");
    Serial.println(DISTANCIA);
      
    if(isnan(DISTANCIA)){
        Serial.print(F("Failed to read from ultrasonic sensor!"));
//      return;
    }
    if (isnan(h) || isnan(t)) {
      Serial.println(F("Failed to read from DHT sensor!"));
      return;
    }
    
    sprintf(data_dist, "%u", DISTANCIA); //dar formato a un numero entero, flotante, double, etc a String (3 enteros.2 decimales flotantes)
    client.publish("distancia", data_dist); //el topic se llama humedad
    Serial.print("Publish message distancia: ");
    //Serial.print("Distancia: ");
    Serial.println(data_dist);//msg

    sprintf(data_temp, "%3.2f", t); //dar formato a un numero entero, flotante, double, etc a String (3 enteros.2 decimales flotantes)
    client.publish("temperatura", data_temp); //el topic se llama temperatura
    Serial.print("Publish message temperatura: ");
    Serial.println(data_temp);//msg

    sprintf(data_humi, "%3.2f", h); //dar formato a un numero entero, flotante, double, etc a String (3 enteros.2 decimales flotantes)
    client.publish("humedad", data_humi); //el topic se llama humedad
    Serial.print("Publish message humedad: ");
    //Serial.print("Humedad: ");
    Serial.println(data_humi);//msg
    sensorFC28();
  }
  
}

void sensorFC28() {

  //Enviando humedad y temperatura
  //Aplique energía al sensor de humedad del suelo
  digitalWrite(pinfc28, HIGH);
  delay(10); // espera de 10 milisegundos
  humedadSuelo = analogRead(A0);
  // Apague el sensor para reducir la corrosión del metal
  //con el tiempo
  digitalWrite(pinfc28, LOW);
  //Convertir el valor en porcentaje
  float valHumsuelo = map(humedadSuelo,1023,0,0,100);

  if (isnan(humedadSuelo)) {
    Serial.print(F("Failed to read from FC-28 sensor!"));
    return;
  }

  sprintf(data_humiSuelo, "%3.2f", valHumsuelo); //dar formato a un numero entero, flotante, double, etc a String (3 enteros.2 decimales flotantes)
  client.publish("humedadSuelo", data_humiSuelo); //el topic se llama humedadSuelo
  Serial.print("Publish message humedadSuelo: ");
  //Serial.print("humedadSuelo: ");
  Serial.println(data_humiSuelo);//msg
}


void sensorPir()
{ 

    //Sensor de Movimiento
    val=digitalRead(pinPir);
    if(val==HIGH){//si está activado
      //digitalWrite(pinLED,HIGH);//encender LED movimiento
      if(pirEstado==LOW){//si previamente estaba apagado
        Serial.println("Sensor activado");
        pirEstado=HIGH;
      }
    }else{//si está desactivado
        //digitalWrite(pinLED,LOW);
        if(pirEstado==HIGH){//si previamente estaba encendido
          Serial.println("Sensor apagado");
          pirEstado=LOW;
        }
    }}

/*Para NODE RED
  topics:
  =======
  publish
  =======
  humedad
  temperatura

  suscribir
  =========
  esp82iot

*/
