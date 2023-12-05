#include <Arduino.h>
#include <Wifi.h>
#include <ArduinoMqttClient.h>
#include <TroykaIMU.h>
#include <TroykaMeteoSensor.h>




char ssid[] = "SSID_NAME"; // Название сети
char pass[] = "SSID_PASS"; // Пароль для сети

#define USERNAME "username" // username указанный в dealgate
#define PASSWORD "password" // password указанный в dealgate



byte picoID[] = 
{
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff,             //here must be 6 mac address byte
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //here must be 8 flash chip id byte
  0xf0, 0xf0                                      //here must be 2 custom byte
};

char picoIDs[32];


WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);
Barometer barometer(0x5D);
TroykaMeteoSensor meteoSensor(0x44);


const char  broker[] = "mqtt.dealgate.ru";
int       port = 1883;
String    tempTopic;
String    pressTopic;
String    humTopic;

const long interval = 10000;
unsigned long previousMillis = 0;

void setup() 
{
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.print("Attempting to connect to WPA SSID: ");
  Serial.println(ssid);
  while (WiFi.begin(ssid, pass) != WL_CONNECTED) {
    // failed, retry
    Serial.print(".");
    delay(500);
  }
  WiFi.macAddress(picoID);
  for (int i = 0; i < 32; i+=2) 
  {
    char buf[2];
    sprintf(buf, "%02x", picoID[i/2]);
    picoIDs[i] = buf[0];
    picoIDs[i+1] = buf[1];
  }
  
  Serial.println("You're connected to the network");
  Serial.print("picoID: ");
  Serial.println(picoIDs);
  Serial.println();
  
  //generate topics
  tempTopic = String(picoIDs);
  tempTopic += "/temperature";
  pressTopic = String(picoIDs);
  pressTopic += "/pressure";
  humTopic = String(picoIDs);
  humTopic += "/humidity";


  //connect mqtt server
  Serial.print("Attempting to connect to the MQTT broker: ");
  Serial.println(broker);
  mqttClient.setKeepAliveInterval(50000);
  mqttClient.setId(picoIDs);
  mqttClient.setUsernamePassword(USERNAME, PASSWORD);
  if (!mqttClient.connect(broker, port)) {
    Serial.print("MQTT connection failed! Error code = ");
    Serial.println(mqttClient.connectError());
    rp2040.reboot();
  }

  Serial.println("You're connected to the MQTT broker!");
  Serial.println();

  barometer.begin();
  Serial.println("LPS is ready");
  Serial.println();

  meteoSensor.begin();
  Serial.println("SHT is ready");
  Serial.println();

  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {

  mqttClient.poll();

  if (!wifiClient.connected() or !mqttClient.connected())
  {
    rp2040.reboot();
  }
  if (millis() <= previousMillis)
    previousMillis = millis();

  if (millis() - previousMillis >= interval) {
    digitalWrite(LED_BUILTIN, HIGH);
    // save the last time a message was sent
    int state = meteoSensor.read();
    float pressure = barometer.readPressurePascals();
    float temperature = (barometer.readTemperatureC() + meteoSensor.getTemperatureC())/2.0;//тепмературу выдаём как среднее арифметическое между данными с LPS и SHT
    float humidity = meteoSensor.getHumidity();
    Serial.print("SHT state: "); 
    Serial.println(state);
    Serial.print("WIFI Status: "); 
    Serial.println(wifiClient.connected());
    Serial.print("Is mqtt connected: ");
    Serial.println(mqttClient.connected());
    previousMillis = millis();

    Serial.print("Sending message to topic: ");
    Serial.println(tempTopic);
    Serial.println(temperature);

    Serial.print("Sending message to topic: ");
    Serial.println(pressTopic);
    Serial.println(pressure);

    Serial.print("Sending message to topic: ");
    Serial.println(humTopic);
    Serial.println(humidity);

    // send message, the Print interface can be used to set the message contents
    mqttClient.beginMessage(tempTopic);
    mqttClient.print(temperature);
    mqttClient.endMessage();

    mqttClient.beginMessage(pressTopic);
    mqttClient.print(pressure);
    mqttClient.endMessage();

    mqttClient.beginMessage(humTopic);
    mqttClient.print(humidity);
    mqttClient.endMessage();

    Serial.println();
    digitalWrite(LED_BUILTIN, LOW);
  }
}
