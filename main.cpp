#include <Arduino.h>
#include <Wifi.h>
#include <ArduinoMqttClient.h>
#include <TroykaIMU.h>
#include <TroykaMeteoSensor.h>
#include <RPi_Pico_TimerInterrupt.h>

bool timer_int(struct repeating_timer *t);  // Прототип функции обработчика прерываний

const char SSID[] = "ssid";                 // Название сети
const char SSIDPass[] = "pass";             // Пароль для сети

const char Username[] = "username";         // логин указанный в MQTT-брокере
const char Password[] = "password";         // пароль указанный в MQTT-брокере

const char Brocker[] = "mqtt.dealgate.ru";  // Брокер
const int Port = 1883;                      // Порт подключения брокера

const int Interval = 1000000 * 10;          // Интервал отправки данных в микросекундах(второе число - секунды)
RPI_PICO_Timer ITimer(0);                   // Экземпляр таймера


byte picoID[] = 
//Идентификатор устройства
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


String    tempTopic;
String    pressTopic;
String    humTopic;

unsigned long previousMillis = 0;

void setup() 
{

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  Serial.begin(9600);
  Serial.print("#t# - Attempting to connect to WPA SSID: ");
  Serial.println(SSID);
  while (WiFi.begin(SSID, SSIDPass) != WL_CONNECTED) 
  {
    // failed, retry
    Serial.print(".");
    delay(500);
  }
  Serial.println("#t# - You're connected to the network");

  pico_unique_board_id_t p;     //Структура с ID чипа памяти
  pico_get_unique_board_id(&p);

  WiFi.macAddress(picoID);      //Записывыаем MAC адресс в первые 6 байт идентификатора

  for (int i = 0; i <8; i++) 
    picoID[i+6] = p.id[i];      //Записываем ID чипа памяти в следующие 8 байт идентификатора

  for (int i = 0; i < 32; i+=2) //Конвертируем ID в строку
  {
    char buf[2];
    sprintf(buf, "%02x", picoID[i/2]);
    picoIDs[i] = buf[0];
    picoIDs[i+1] = buf[1];
  }
  Serial.print("#t# - picoID: ");
  
  Serial.println(picoIDs);
  Serial.println();
  
  //Генерируем топики
  tempTopic = String(picoIDs);
  tempTopic += "/temperature";
  pressTopic = String(picoIDs);
  pressTopic += "/pressure";
  humTopic = String(picoIDs);
  humTopic += "/humidity";


  //Подключаемся к брокеру
  Serial.print("#t# - Attempting to connect to the MQTT broker: ");
  Serial.println(Brocker);
  mqttClient.setKeepAliveInterval(50000);
  mqttClient.setId(picoIDs);
  mqttClient.setUsernamePassword(Username, Password);
  if (!mqttClient.connect(Brocker, Port)) 
  {
    Serial.print("#t# - MQTT connection failed! Error code = ");
    Serial.println(mqttClient.connectError());
    rp2040.reboot();  // Если произошла ошибка при подключении к бркеру перезагружаемся
  }
  Serial.println("#t# - You're connected to the MQTT broker!");
  Serial.println();

  barometer.begin();
  Serial.println("#t# - LPS is ready");
  Serial.println();

  meteoSensor.begin();
  Serial.println("#t# - SHT is ready");
  Serial.println();

  if (ITimer.attachInterruptInterval(Interval, timer_int))
  {
    Serial.print(F("#t# - Starting ITimer OK, millis() = ")); Serial.println(millis());
  }
  else
    Serial.println(F("#t# - Can't set ITimer. Select another Timer, freq. or timer"));

  digitalWrite(LED_BUILTIN, LOW);
}

void loop() 
{
  
}

bool timer_int(struct repeating_timer *t)
{
  mqttClient.poll();  //Проверяем есть ли новые сообщения

  if (!wifiClient.connected() or !mqttClient.connected())
  {
    Serial.println("#t# - Connection failed, rebooting");
    rp2040.reboot();  //Если произошёл разрыв соединения перезагружаем плату
  }

  digitalWrite(LED_BUILTIN, HIGH);
  // save the last time a message was sent
  int state = meteoSensor.read();
  float pressure = barometer.readPressurePascals();
  float temperature = (barometer.readTemperatureC() + meteoSensor.getTemperatureC())/2.0;//тепмературу выдаём как среднее арифметическое между данными с LPS и SHT
  float humidity = meteoSensor.getHumidity();

  Serial.print("#t# - SHT state: "); 
  Serial.println(state);
  Serial.print("#t# - WIFI Status: "); 
  Serial.println(wifiClient.connected());
  Serial.print("#t# - Is mqtt connected: ");
  Serial.println(mqttClient.connected());

  Serial.print("#t# - Sending message to topic: ");
  Serial.println(tempTopic);
  Serial.println(temperature);

  Serial.print("#t# - Sending message to topic: ");
  Serial.println(pressTopic);
  Serial.println(pressure);

  Serial.print("#t# - Sending message to topic: ");
  Serial.println(humTopic);
  Serial.println(humidity);
  Serial.println();

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

  digitalWrite(LED_BUILTIN, LOW);
  return true;
}
