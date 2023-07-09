const int sensorPin = A0;
const int relePin = 3;
const int umbralMinimo = 200;  		// Valor mínimo del sensor de suelo
const int umbralMaximo = 800;  		// Valor máximo del sensor de suelo

void setup() {
	pinMode(relePin, OUTPUT); 		//pin de salida relé, esto enviará una señal al relé
	pinMode(sensorPin, INPUT); 		//pin de entrada procedente del sensor de suelo
}

void loop() { 
	int valorSensor = digitalRead(sensorPin);  		// leyendo la señal que viene del sensor de suelo
	int valorRelativo = map(valorSensor, umbralMinimo, umbralMaximo, 0, 1);  	// Mapeo del valor del sensor a un rango de 0 a 1
  
	if(valorRelativo == HIGH) {			// si el nivel de agua está lleno, corte el relé
		digitalWrite(relePin, LOW); 	// low para cortar el rele
	} else {
		digitalWrite(relePin, HIGH); 	//high para seguir probando señal y suministro de agua
	}
	
	delay(400); 
}
