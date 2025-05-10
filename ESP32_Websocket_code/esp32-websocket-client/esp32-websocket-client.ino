/* ESP32 WebSocket client example
 * Este código implementa un cliente WebSocket para controlar un TWIN DIGITAL ROBOT usando ESP32
 * El sistema recibe comandos JSON desde un servidor y ejecuta movimientos del robot
 */

// Inclusión de librerías necesarias
#include <WiFi.h>              // Para la conexión WiFi
#include <WebSocketsClient.h>  // Para la comunicación WebSocket
#include <ArduinoJson.h>       // Para procesar mensajes JSON

// Configuración de la red WiFi WOKWI
//#define WIFI_SSID "Wokwi-GUEST"
//#define WIFI_PASSWORD ""
// Configuración de la red WiFi
#define WIFI_SSID "LAB. ROBOTICA"
#define WIFI_PASSWORD "LabRoboticaUAI"
// Definir el canal WiFi acelera la conexión
#define WIFI_CHANNEL 6

// Configuración de timeouts y reintentos
#define WIFI_TIMEOUT 10000     // 10 segundos timeout
#define WIFI_RETRY_DELAY 500   // 0.5 segundos entre intentos
#define MAX_WIFI_RETRIES 20    // Máximo número de intentos

#define LED 2                 // led por defecto

#define M1_A  16
#define M1_B  17
#define M1_E  25  // Para pwm

#define M2_A  19
#define M2_B  23
#define M2_E  5  // Para pwm

// Variables globales
WebSocketsClient webSocket;
bool isConnected = false;
unsigned long lastConnectionAttempt = 0;

// Configuración del servidor WebSocket
const char* address = "lrfia.uai.edu.ar";    // Dirección del servidor
int port = 8007;                             // Puerto del servidor
const char* route = "/ws";                   // Ruta del WebSocket


// Función para mostrar datos en formato hexadecimal (útil para debugging)
void hexdump(const void *mem, uint32_t len, uint8_t cols = 16) {
	const uint8_t* src = (const uint8_t*) mem;
	Serial.printf("\n[HEXDUMP] Address: 0x%08X len: 0x%X (%d)", (ptrdiff_t)src, len, len);
	for(uint32_t i = 0; i < len; i++) {
		if(i % cols == 0) {
			Serial.printf("\n[0x%08X] 0x%08X: ", (ptrdiff_t)src, i);
		}
		Serial.printf("%02X ", *src);
		src++;
	}
	Serial.printf("\n");
}

void Led_blink(){
    digitalWrite(LED,HIGH);
    delay(300);
    // Aquí iría el código para controlar los motores
    
    digitalWrite(LED,LOW);
    delay(300);

}

void AtrasRuedaIzq()
{
    digitalWrite(M1_A,HIGH);
    digitalWrite(M1_B,LOW);
}

void AdelanteRuedaIzq()
{
    digitalWrite(M1_A,LOW);
    digitalWrite(M1_B,HIGH);
}

void AtrasRuedaDer()
{
    digitalWrite(M2_A,HIGH);
    digitalWrite(M2_B,LOW);
}

void AdelanteRuedaDer()
{
    digitalWrite(M2_A,LOW);
    digitalWrite(M2_B,HIGH);
}

void Detener(){
    digitalWrite(M2_A,LOW);
    digitalWrite(M2_B,LOW);
    digitalWrite(M1_A,LOW);
    digitalWrite(M1_B,LOW);
}

// Funciones básicas de movimiento del robot
bool avanzar() {
  try {
    Serial.println("Avanzando");
    AdelanteRuedaDer();
    AdelanteRuedaIzq();
    // Aquí iría el código para controlar los motores
    delay(300);
    Detener();
    return true;
  } catch (...) {
    Serial.println("Error al avanzar");
    return false;
  }
}



bool retroceder() {
  try {
    Serial.println("Retrocediendo");
    AtrasRuedaDer();
    AtrasRuedaIzq();
    // Aquí iría el código para controlar los motores
    delay(300);
    Detener();
    return true;
  } catch (...) {
    Serial.println("Error al retroceder");
    return false;
  }
}

bool rotar_izq() {
  try {
    Serial.println("Rotando a la izquierda");
    digitalWrite(LED,HIGH);
    digitalWrite(M1_A,LOW);
    digitalWrite(M1_B,HIGH);
    digitalWrite(M2_A,HIGH);
    digitalWrite(M2_B,LOW);
    // Aquí iría el código para controlar los motores
    delay(300);
    digitalWrite(LED,LOW);
    digitalWrite(M1_A,LOW);
    digitalWrite(M1_B,LOW);
    digitalWrite(M2_A,LOW);
    digitalWrite(M2_B,LOW);
    return true;
  } catch (...) {
    Serial.println("Error al rotar izquierda");
    return false;
  }
}

bool rotar_der() {
  try {
    Serial.println("Rotando a la derecha");
    digitalWrite(LED,HIGH);
    digitalWrite(M1_A,HIGH);
    digitalWrite(M1_B,LOW);
    digitalWrite(M2_A,LOW);
    digitalWrite(M2_B,HIGH);
    // Aquí iría el código para controlar los motores
    delay(300);
    digitalWrite(LED,LOW);
    digitalWrite(M1_A,LOW);
    digitalWrite(M1_B,LOW);
    digitalWrite(M2_A,LOW);
    digitalWrite(M2_B,LOW);
    return true;
  } catch (...) {
    Serial.println("Error al rotar derecha");
    return false;
  }
}

// Función principal para procesar los datos JSON recibidos
bool Procesar_dato(uint8_t * payload, size_t length) {
    String jsonString = "";
    for (size_t i = 0; i < length; i++) {
        jsonString += (char) payload[i];
    }

    Serial.println("Mensaje recibido:");
    Serial.println(jsonString);

    // Deserializar JSON con manejo de errores
    DynamicJsonDocument doc(1024);
    DeserializationError error = deserializeJson(doc, jsonString);

    if (error) {
        Serial.print("Error al deserializar JSON: ");
        Serial.println(error.c_str());
        return false;
    }

    // Verificar campos requeridos
    if (!doc.containsKey("object_id") || !doc.containsKey("command")) {
        Serial.println("Error: JSON incompleto, faltan campos requeridos");
        return false;
    }

    String object_id = doc["object_id"];
    String command = doc["command"];
    String type = doc["type"];

    // Procesar comandos con manejo de errores
    if (object_id == "robot0") {
        if (command == "mover") {
            bool success = false;
            if (type == "avanzar") {
                success = avanzar();
            }
            else if (type == "retroceder") {
                success = retroceder();
            }
            else if (type == "rotar") {
                int rot_y = doc["rotation"]["y"];
                if (rot_y > 0) {
                    success = rotar_der();
                } else {
                    success = rotar_izq();
                }
            }

            // Enviar confirmación al servidor
            String response;
            if (success) {
                response = "{\"status\":\"success\",\"action\":\"" + type + "\"}";
            } else {
                response = "{\"status\":\"error\",\"action\":\"" + type + "\"}";
            }
            webSocket.sendTXT(response);
            return success;
        }
    }
    return false;
}

// Manejador de eventos WebSocket
void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
	switch(type) {
		case WStype_DISCONNECTED:
			Serial.println("[WSc] Desconectado!");
			isConnected = false;
			break;
		case WStype_CONNECTED:
			Serial.printf("[WSc] Conectado a: %s\n", payload);
			isConnected = true;
			// Enviar mensaje de conexión con información del dispositivo
			webSocket.sendTXT("{\"device\":\"robot0\",\"status\":\"connected\",\"version\":\"1.0\"}");
			break;
		case WStype_TEXT:
			if (!Procesar_dato(payload, length)) {
				Serial.println("Error procesando el comando");
				webSocket.sendTXT("{\"status\":\"error\",\"message\":\"Error procesando comando\"}");
			}
			break;
		case WStype_BIN:
			Serial.printf("[WSc] Datos binarios recibidos, longitud: %u\n", length);
			hexdump(payload, length);
			break;
		case WStype_ERROR:
			Serial.println("[WSc] Error en la conexión!");
			break;
		case WStype_FRAGMENT_TEXT_START:
		case WStype_FRAGMENT_BIN_START:
		case WStype_FRAGMENT:
		case WStype_FRAGMENT_FIN:
			break;
	}
}

bool connectToWiFi() {
    if (millis() - lastConnectionAttempt < WIFI_RETRY_DELAY) {
        return false;
    }
    
    lastConnectionAttempt = millis();
    
    if (WiFi.status() == WL_CONNECTED) {
        return true;
    }

    Serial.print("Conectando a WiFi ");
    Serial.print(WIFI_SSID);

    WiFi.begin(WIFI_SSID, WIFI_PASSWORD, WIFI_CHANNEL);

    int retries = 0;
    while (WiFi.status() != WL_CONNECTED && retries < MAX_WIFI_RETRIES) {
        delay(WIFI_RETRY_DELAY);
        Serial.print(".");
        retries++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println(" Conectado!");
        Serial.print("Dirección IP: ");
        Serial.println(WiFi.localIP());
        return true;
    } else {
        Serial.println(" Falló la conexión!");
        return false;
    }
}

// Configuración inicial del WebSocket
void setupWebSocket() {
  Serial.println("Configurando WebSocket...");
  // Configurar dirección del servidor, puerto y ruta
	webSocket.begin(address, port, route);

	// Configurar el manejador de eventos
	webSocket.onEvent(webSocketEvent);

	// Configurar reconexión automática cada 5 segundos si falla la conexión
	webSocket.setReconnectInterval(5000);
}

// Función de configuración inicial
void setup(void) {
  // Iniciar comunicación serial para debugging
  Serial.begin(9600);
  pinMode(LED,OUTPUT);
  pinMode(M1_A,OUTPUT);
  pinMode(M1_B,OUTPUT);
  pinMode(M1_E,OUTPUT);
  pinMode(M2_A,OUTPUT);
  pinMode(M2_B,OUTPUT);
  pinMode(M2_E,OUTPUT);

  Led_blink();

  Serial.println("\nIniciando sistema...");
  
  if (!connectToWiFi()) {
      Serial.println("Error de conexión WiFi inicial");
      ESP.restart();  // Reiniciar si falla la conexión inicial
  }

  // Configurar y iniciar el cliente WebSocket
  setupWebSocket();

  
}

// Bucle principal
void loop(void) {
    // Verificar conexión WiFi y reconectar si es necesario
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("Conexión WiFi perdida");
        if (connectToWiFi()) {
            setupWebSocket();
        }
    }
    
    webSocket.loop();
    
    // Verificar estado de la conexión cada cierto tiempo
    static unsigned long lastCheck = 0;
    if (millis() - lastCheck > 30000) {  // Cada 30 segundos
        lastCheck = millis();
        if (isConnected) {
            webSocket.sendTXT("{\"status\":\"heartbeat\"}");
        }
    }


}
