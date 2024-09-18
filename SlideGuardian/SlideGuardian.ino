#include <BluetoothSerial.h>  // Librería para Bluetooth
#include <WiFi.h>  // Librería para WiFi
#include <WiFiClientSecure.h>  // Librería para cliente WiFi seguro
#include <PubSubClient.h>  // Librería para MQTT
#include <Preferences.h>  // Librería para almacenar preferencias

// Certificado de CA para la conexión segura
const char *ca_cert = \
"-----BEGIN CERTIFICATE-----\n" \
"-----END CERTIFICATE-----\n";

// Datos del broker MQTT y temas de publicación/suscripción
const char *mqtt_broker = "j2e9711e.ala.us-east-1.emqxsl.com";
const char *topic_vibration = "/nodejs/mqtt/vibration";
const char *topic_moisture = "/nodejs/mqtt/moisture";
const char *topic_relay = "/nodejs/mqtt/relay";
const char *mqtt_username = "";
const char *mqtt_password = "";
const int mqtt_port = 8883;

// Inicialización de Bluetooth, WiFi, MQTT y preferencias
BluetoothSerial BT;
WiFiClientSecure espClientSecure;
PubSubClient client(espClientSecure);
Preferences preferences;

// Pines de los sensores y relé
const int moistureSensorPin = A0;
const int vibrationSensorPin = 25;
const int relayPin = 16;

// Variables para controlar el estado y lectura de sensores
bool newCredentialsReceived = false; // Indica si se han recibido nuevas credenciales de WiFi a través de Bluetooth.
int moisture, sensorAnalog; // 'moisture' almacena el porcentaje de humedad del suelo calculado. 'sensorAnalog' almacena la lectura analógica del sensor de humedad.
bool manualRelayControl = false; // Indica si el relé está siendo controlado manualmente (true) o automáticamente (false).
bool relayState = false; // Almacena el estado actual del relé. 'true' significa que el relé está encendido, 'false' significa que está apagado.
bool lastRelayState = false; // Almacena el estado previo del relé para detectar cambios y poder publicar estos cambios en el broker MQTT.
unsigned long lastSensorReadTime = 0; // Almacena la última vez que se leyeron los sensores. Utilizado para controlar la frecuencia de las lecturas.
const unsigned long sensorReadInterval = 1000; // Intervalo de tiempo (en milisegundos) entre cada lectura de los sensores. En este caso, se lee cada segundo (1000 ms).
unsigned long manualControlEndTime = 0; // Almacena el tiempo en el que finalizará el control manual del relé. Utilizado para volver al control automático después de un periodo.
const unsigned long manualControlDuration = 10000; // Duración (en milisegundos) del periodo de control manual del relé. En este caso, el control manual dura 10 segundos (10000 ms).

// Conexión a WiFi con las credenciales recibidas
void connectToWiFi(const String& ssid, const String& password) {
  Serial.print("Conectando a WiFi: ");
  Serial.println(ssid);

  WiFi.begin(ssid.c_str(), password.c_str());

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nConectado a la red WiFi");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nFallo en la conexión WiFi");
  }
}

// Reintenta la conexión al broker MQTT
void reconnectMQTT() {
  while (!client.connected()) {
    Serial.print("Intentando conectar al broker MQTT...");
    String client_id = "landslide-alert";
    client_id += String(WiFi.macAddress());
    if (client.connect(client_id.c_str(), mqtt_username, mqtt_password)) {
      Serial.println("Conectado al broker MQTT");
      client.subscribe(topic_relay);
    } else {
      Serial.print("Fallo en la conexión con el estado ");
      Serial.println(client.state());
      delay(2000);
    }
  }
}

// Callback para recibir mensajes del broker MQTT
void callback(char* topic, byte* payload, unsigned int length) {
  String message;
  for (int i = 0; i < length; i++) { // Convierte el contenido del mensaje (payload) de tipo byte a un String. Recorre cada byte del payload y lo añade al String message.
    message += (char)payload[i];
  }

  Serial.print("Mensaje recibido [");
  Serial.print(topic);
  Serial.print("]: ");
  Serial.println(message);

  if (String(topic) == topic_relay) { // Comprueba si el mensaje recibido es para el tema del relé.
    manualRelayControl = true; // Activa el control manual del relé.
    manualControlEndTime = millis() + manualControlDuration; // Establece el tiempo en el que finalizará el control manual.
    if (message == "1") {
      relayState = true;
    } else if (message == "0") {
      relayState = false;
    }
  }
}

void setup() {
  Serial.begin(115200);
  BT.begin("Landslide Monitor");
  preferences.begin("wifiCreds", false);

  pinMode(vibrationSensorPin, INPUT);
  pinMode(relayPin, OUTPUT);

  // Lee las credenciales de WiFi almacenadas
  String ssid = preferences.getString("ssid", "");
  String password = preferences.getString("password", "");

  // Si hay credenciales almacenadas, intenta conectarse a WiFi.
  if (ssid != "") {
    connectToWiFi(ssid, password);
  } else {
    Serial.println("Esperando credenciales de WiFi...");
  }

  espClientSecure.setCACert(ca_cert); // Establece el certificado de CA para la conexión segura.
  client.setServer(mqtt_broker, mqtt_port); // Configura el broker MQTT.
  client.setCallback(callback); // Establece la función de callback para MQTT.
}

void loop() {
  if (BT.available()) {
    String data = BT.readStringUntil('\n');
    data.trim();

    // Si los datos son "1" o "0", se controla manualmente el relé.
    if (data == "1" || data == "0") {
      manualRelayControl = true;
      relayState = (data == "1");
      manualControlEndTime = millis() + manualControlDuration;
      BT.println(relayState ? 1 : 0);
    } else {
      // Si los datos no son "1" o "0", se consideran como credenciales de WiFi.
      String ssid = data;
      String password = BT.readStringUntil('\n');
      password.trim();

      connectToWiFi(ssid, password);

      // Si la conexión a WiFi es exitosa, se almacenan las credenciales.
      if (WiFi.status() == WL_CONNECTED) {
        preferences.putString("ssid", ssid);
        preferences.putString("password", password);
        newCredentialsReceived = true;
      }
    }
  }

  if (WiFi.status() == WL_CONNECTED) {
    // Si se han recibido nuevas credenciales, se desconecta del broker MQTT.
    if (newCredentialsReceived) {
      newCredentialsReceived = false;
      client.disconnect();
      Serial.println("Desconectado del broker MQTT para intentar nueva conexión WiFi");
    }

    // Si no está conectado al broker MQTT, intenta reconectar.
    if (!client.connected()) {
      reconnectMQTT();
    }

    client.loop();  // Mantiene la conexión al broker MQTT.

    unsigned long currentMillis = millis(); // Obtiene el tiempo actual en milisegundos desde que comenzó a ejecutarse el programa.
    if (currentMillis - lastSensorReadTime >= sensorReadInterval) { // Verifica si ha pasado el tiempo suficiente desde la última lectura de los sensores para realizar otra lectura, según el intervalo de tiempo especificado por sensorReadInterval.
      lastSensorReadTime = currentMillis; // Actualiza el tiempo de la última lectura de los sensores al tiempo actual.

      sensorAnalog = analogRead(moistureSensorPin); // Lee el valor analógico del sensor de humedad.
      moisture = (100 - ((sensorAnalog / 4095.00) * 100)); // Convierte el valor analógico a porcentaje de humedad.
      Serial.print("Humedad = ");
      Serial.print(moisture);
      Serial.println("%");
      BT.println(moisture);

      bool vibrationDetected = digitalRead(vibrationSensorPin); // Lee el estado del sensor de vibración.
      bool highMoisture = (moisture > 70); // Comprueba si la humedad es alta.

      // Si el control manual ha terminado, vuelve al control automático.
      if (manualRelayControl && currentMillis >= manualControlEndTime) {
        manualRelayControl = false;
        Serial.println("Control manual terminado, volviendo al control automático.");
      }

      // Control automático del relé basado en la detección de vibración o alta humedad.
      if (!manualRelayControl) {
        relayState = vibrationDetected || highMoisture;
      }

      // Publica el estado de vibración y humedad en el broker MQTT y por Bluetooth.
      if (vibrationDetected) {
        Serial.println("Vibración detectada...");
        client.publish(topic_vibration, "Vibración detectada");
        BT.println(1);
      } else {
        Serial.println("...");
        client.publish(topic_vibration, "Sin vibración");
        BT.println(0);
      }

      if (highMoisture) {
        Serial.println("Alta humedad detectada...");
      }

      digitalWrite(relayPin, relayState ? HIGH : LOW);  // Activa o desactiva el relé.
      BT.println(relayState ? 1 : 0);  // Envía el estado del relé por Bluetooth.

      char msg_moisture[50]; // Almacena el mensaje de humedad antes de enviarlo por el broker MQTT.
      snprintf(msg_moisture, 50, "%d", moisture); // Formatea el valor de humedad en una cadena de caracteres.
      client.publish(topic_moisture, msg_moisture); // Publica el valor de humedad en el broker MQTT.

      // Si el estado del relé ha cambiado, lo publica en el broker MQTT.
      if (relayState != lastRelayState) {
        lastRelayState = relayState;
        char msg_relay[50];
        snprintf(msg_relay, 50, "%d", relayState ? 1 : 0);
        client.publish(topic_relay, msg_relay);
      }
    }
  }
}
