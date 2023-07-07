#include <SoftwareSerial.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

SoftwareSerial SerialESP8266(13, 15); // RX, TX

const int lonbuffer = 12;                                       // longitud del buffer
char buffer[lonbuffer];                                         // buffer para almacenar el comando
float luminosidad;                                              // valor del tercer parámetro

//Pines de los focos
#define greenPin D4
#define redPin D3

//Pin del ventilador
#define ventiladorPin D5


//Configuracion del user
//const char* ssid = "TP_Ayma";
//const char* password = "09089327";

const char* ssid = "redmi";
const char* password = "rod963XD";

char data_temp[12] = "";
char data_humi[12] = "";
char data_humiSuelo[12] = "";
char data_digital[12] = "";
char data_dist[12] = "";
char data_lumi[12] = "";
char data_Mag1[12] = "";
char data_Mag2[12] = "";

//IP del servidor BROKER
//const char *mqtt_broker = "192.168.0.110";
const char *mqtt_broker = "192.168.173.11";
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
  //FOCOS DE LA PUERTA DE ENTRADA Y HABITACIÓN 1
  if (String(topic) == "esp82iot") {
    if (messageData == "redOn") {
      Serial.println("red On");
      digitalWrite(redPin, HIGH);
    } else if (messageData == "redOff") {
      Serial.println("red Off");
      digitalWrite(redPin, LOW);
    }
    if (messageData == "greenOn") {
      Serial.println("green On");
      digitalWrite(greenPin, HIGH);
    } else if (messageData == "greenOff") {
      Serial.println("greenOff");
      digitalWrite(greenPin, LOW);
    }
  }
  //VENTILADOR
  if (String(topic) == "esp82iot/ventilador") {
    if (messageData == "On") {
      Serial.println("ventilador On");
      digitalWrite(ventiladorPin, LOW);
    } else if (messageData == "Off") {
      Serial.println("ventilador off");
      digitalWrite(ventiladorPin, HIGH);
    }
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
      client.subscribe("esp82iot/ventilador");
      Serial.println("Conexión exitosa");
    }
    else {
      Serial.println("Failed, rc=");
      Serial.print(client.state());
      Serial.print(" esperando 3 segundos");
      delay(3000);
    }
  }
}

void setup()
{
  Serial.begin(9600);
  SerialESP8266.begin(9600);
  config_wifi();
  pinMode(greenPin, OUTPUT);
  pinMode(redPin, OUTPUT);
  pinMode(ventiladorPin, OUTPUT);
  digitalWrite(ventiladorPin,HIGH);
  //client.setServer(mqtt_broker,1883);//Conexión al servidor/Broker
  client.setServer(mqtt_broker, mqtt_port);
  client.setCallback(callback);//Llamada a los callback para ver si recibo mensajes del broker
}


void loop()
{
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  checkSerialCom();
}

void checkSerialCom() {
  if (SerialESP8266.available() > 0)
  {
    SerialESP8266.readBytesUntil('T', buffer, lonbuffer);
    float t = SerialESP8266.parseFloat();

    SerialESP8266.readBytesUntil('H', buffer, lonbuffer);
    float h = SerialESP8266.parseFloat();

    SerialESP8266.readBytesUntil('L', buffer, lonbuffer);
    float l = SerialESP8266.parseFloat();

    SerialESP8266.readBytesUntil('VHS', buffer, lonbuffer);
    float valHumsuelo = SerialESP8266.parseFloat();

    SerialESP8266.readBytesUntil('U', buffer, lonbuffer);
    int distancia = SerialESP8266.parseInt();

    SerialESP8266.readBytesUntil('Mov', buffer, lonbuffer);
    int movimiento = SerialESP8266.parseInt();

    if (movimiento == 1) {
      client.publish("pir", "1"); //el topic se llama pir
      //client.publish("pir", mov); //el topic se llama pir
    } else {
      client.publish("pir", "0"); //el topic se llama pir
    }

    SerialESP8266.readBytesUntil('Mag1', buffer, lonbuffer);
    int sensorMgn1 = SerialESP8266.parseInt();

    SerialESP8266.readBytesUntil('Mag2', buffer, lonbuffer);
    int sensorMgn2 = SerialESP8266.parseInt();

    sprintf(data_temp, "%3.2f", t); //dar formato a un numero entero, flotante, double, etc a String (3 enteros.2 decimales flotantes)
    client.publish("temperatura", data_temp); //el topic se llama temperatura
    Serial.print("Publish message temperatura: ");
    Serial.println(data_temp);//msg

    sprintf(data_humi, "%3.2f", h); //dar formato a un numero entero, flotante, double, etc a String (3 enteros.2 decimales flotantes)
    client.publish("humedad", data_humi); //el topic se llama humedad
    Serial.print("Publish message humedad: ");
    Serial.println(data_humi);//msg

    sprintf(data_lumi, "%3.2f", l); //dar formato a un numero entero, flotante, double, etc a String (3 enteros.2 decimales flotantes)
    client.publish("luminosidad", data_lumi); //el topic se llama luminosidad
    Serial.print("Publish message luminosidad: ");
    Serial.println(data_lumi);//msg

    sprintf(data_humiSuelo, "%3.2f", valHumsuelo); //dar formato a un numero entero, flotante, double, etc a String (3 enteros.2 decimales flotantes)
    client.publish("humedadSuelo", data_humiSuelo); //el topic se llama humedadSuelo
    Serial.print("Publish message humedadSuelo: ");
    Serial.println(data_humiSuelo);//msg

    sprintf(data_dist, "%u", distancia); //dar formato a un numero entero, flotante, double, etc a String (3 enteros.2 decimales flotantes)
    client.publish("distancia", data_dist); //el topic se llama distanciaObj
    Serial.print("Publish message distanciaObj: ");
    Serial.print(data_dist);//msg
    Serial.println(" cm");

    Serial.print("Publish message movimiento: ");
    Serial.println(movimiento);//msg

    sprintf(data_Mag1, "%d", sensorMgn1); //dar formato a un numero entero, flotante, double, etc a String (3 enteros.2 decimales flotantes)
    client.publish("puertaEntrada", data_Mag1); //el topic se llama puertaEntrada
    
    sprintf(data_Mag2, "%d", sensorMgn2); //dar formato a un numero entero, flotante, double, etc a String (3 enteros.2 decimales flotantes)
    client.publish("habitacion1", data_Mag2);

    
  }
}
