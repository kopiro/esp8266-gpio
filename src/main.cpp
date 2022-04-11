#include <WiFiManager.h>
#include <ESP8266WiFi.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <ArduinoOTA.h>
#include <ESP8266mDNS.h>
#include <SoftwareSerial.h>

#define MQTT_CONN_RETRY_INTERVAL 60000

#define DHT_ENABLED 1
#define DHT_PIN 16
#define DHT_POLLING_TIMEOUT 30000

#define PIR_ENABLED 1
#define PIR_PIN 4

// #define SERVO_ENABLED 1
// #define SERVO_PIN 5

#define BOARD_ID "toiletgpio"

char mqtt_server[64];
char mqtt_username[64];
char mqtt_password[64];

unsigned long currentTime = millis();

#define AVAILABILITY_ONLINE "online"
#define AVAILABILITY_OFFLINE "offline"

char MQTT_TOPIC_AVAILABILITY[80];
char MQTT_TOPIC_CALLBACK[80];
char MQTT_TOPIC_DEBUG[80];

char MQTT_TOPIC_TEMPERATURE[80];
char MQTT_TOPIC_HUMIDITY[80];
char MQTT_TOPIC_MOTION[80];
char MQTT_TOPIC_SERVO[80];

WiFiManager wifiManager;
WiFiClient wifiClient;
PubSubClient mqttClient;

WiFiManagerParameter wifi_param_mqtt_server("mqtt_server", "MQTT server", mqtt_server, sizeof(mqtt_server));
WiFiManagerParameter wifi_param_mqtt_username("mqtt_user", "MQTT username", mqtt_username, sizeof(mqtt_username));
WiFiManagerParameter wifi_param_mqtt_password("mqtt_pass", "MQTT password", mqtt_password, sizeof(mqtt_password));

unsigned long lastMqttConnectionAttempt = millis();

void sendMQTTMessage(const char *topic, const char *message, bool retained)
{
    Serial.printf("MQTT message - topic: <%s>, message: <%s> -> ", topic, message, retained);

    if (mqttClient.publish(topic, message))
    {
        Serial.println("sent");
    }
    else
    {
        Serial.println("error");
    }
}

void saveConfig()
{
    Serial.println("Saving config...");

    DynamicJsonDocument json(512);
    json["mqtt_server"] = wifi_param_mqtt_server.getValue();
    json["mqtt_username"] = wifi_param_mqtt_username.getValue();
    json["mqtt_password"] = wifi_param_mqtt_password.getValue();
    ;

    File configFile = SPIFFS.open("/config.json", "w");
    if (!configFile)
    {
        Serial.println("Failed to open config file for writing");
        return;
    }

    Serial.printf("Saving JSON: %s", json.as<String>().c_str());

    serializeJson(json, configFile);
    configFile.close();

    Serial.println("Config saved, please reboot");
}

void loadConfig()
{
    Serial.println("Loading config");

    if (!SPIFFS.begin())
    {
        Serial.println("Failed to open SPIFFS");
        return;
    }

    if (!SPIFFS.exists("/config.json"))
    {
        Serial.println("Config file not found, please configure the ESP by connecting to its Wi-Fi hotspot");
        return;
    }

    File configFile = SPIFFS.open("/config.json", "r");
    if (!configFile)
    {
        Serial.println("Failed to open config file");
        return;
    }

    const size_t size = configFile.size();
    std::unique_ptr<char[]> buf(new char[size]);

    configFile.readBytes(buf.get(), size);
    DynamicJsonDocument json(512);

    if (DeserializationError::Ok != deserializeJson(json, buf.get()))
    {
        Serial.println("Failed to parse config fileDebug");
        return;
    }

    strcpy(mqtt_server, json["mqtt_server"]);
    strcpy(mqtt_username, json["mqtt_username"]);
    strcpy(mqtt_password, json["mqtt_password"]);

    wifi_param_mqtt_server.setValue(mqtt_server, sizeof(mqtt_server));
    wifi_param_mqtt_username.setValue(mqtt_username, sizeof(mqtt_username));
    wifi_param_mqtt_password.setValue(mqtt_password, sizeof(mqtt_password));

    Serial.printf("Config JSON: %s", json.as<String>().c_str());
}

void setupGeneric()
{
    Serial.begin(9600);
    loadConfig();

    Serial.printf("Board Identifier: %s", BOARD_ID);

    snprintf(MQTT_TOPIC_AVAILABILITY, 80, "%s/availability", BOARD_ID);
    snprintf(MQTT_TOPIC_CALLBACK, 80, "%s/callback", BOARD_ID);
    snprintf(MQTT_TOPIC_DEBUG, 80, "%s/debug", BOARD_ID);

    snprintf(MQTT_TOPIC_TEMPERATURE, 80, "%s/temperature", BOARD_ID);
    snprintf(MQTT_TOPIC_HUMIDITY, 80, "%s/humidity", BOARD_ID);
    snprintf(MQTT_TOPIC_MOTION, 80, "%s/motion", BOARD_ID);
    snprintf(MQTT_TOPIC_SERVO, 80, "%s/servo", BOARD_ID);
}

bool portalRunning = false;

void setupWifi()
{
    wifiManager.setConfigPortalBlocking(false);
    wifiManager.setDebugOutput(false);
    wifiManager.setSaveParamsCallback(saveConfig);

    wifiManager.addParameter(&wifi_param_mqtt_server);
    wifiManager.addParameter(&wifi_param_mqtt_username);
    wifiManager.addParameter(&wifi_param_mqtt_password);

    if (wifiManager.autoConnect(BOARD_ID))
    {
        WiFi.mode(WIFI_STA);
        wifiManager.startWebPortal();
    }
    else
    {
        Serial.println("Failed to connect to WiFi, starting AP");
    }
}

void loopWifi()
{
    wifiManager.process();
}

void mqttEnsureConnected()
{
    if (mqttClient.connect(BOARD_ID, mqtt_username, mqtt_password, MQTT_TOPIC_AVAILABILITY, 1, true, AVAILABILITY_OFFLINE))
    {
        mqttClient.subscribe(MQTT_TOPIC_CALLBACK);
        sendMQTTMessage(MQTT_TOPIC_AVAILABILITY, AVAILABILITY_ONLINE, true);
    }
    else
    {
        Serial.println("Unable to connect to MQTT broker");
    }
}

void loopMQTT()
{
    if (mqttClient.connected())
    {
        mqttClient.loop();
        return;
    }

    if (currentTime - lastMqttConnectionAttempt < MQTT_CONN_RETRY_INTERVAL)
    {
        return;
    }

    Serial.println("Connection to MQTT lost, reconnecting...");
    lastMqttConnectionAttempt = currentTime;

    mqttEnsureConnected();
}

void setupOTA()
{
    ArduinoOTA.onStart([]() {});
    ArduinoOTA.onEnd([]() {});
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {});
    ArduinoOTA.onError([](ota_error_t error) {});
    ArduinoOTA.setHostname(BOARD_ID);
    ArduinoOTA.setPassword(BOARD_ID);
    ArduinoOTA.begin();
}

void loopOTA()
{
    ArduinoOTA.handle();
}

void setupMDNS()
{
    MDNS.begin(BOARD_ID);
}

void loopMDNS()
{
    MDNS.update();
}

#ifdef DHT_ENABLED
#include "DHTesp.h"

DHTesp dht;
unsigned long lastTimeDHTTime = 0;

void setupDHT()
{
    dht.setup(DHT_PIN, DHTesp::DHT11);
}

void loopDHT()
{
    if (currentTime - lastTimeDHTTime < DHT_POLLING_TIMEOUT)
        return;

    Serial.printf("DHT polling tick");

    lastTimeDHTTime = currentTime;

    TempAndHumidity values = dht.getTempAndHumidity();

    if (!isnan(values.temperature))
    {
        sendMQTTMessage(MQTT_TOPIC_TEMPERATURE, String(values.temperature).c_str(), false);
    }
    else
    {
        Serial.printf("DHT temperature error: %s", dht.getStatusString());
    }

    if (!isnan(values.humidity))
    {
        sendMQTTMessage(MQTT_TOPIC_HUMIDITY, String(values.humidity).c_str(), false);
    }
    else
    {
        Serial.printf("DHT humidity error: %s", dht.getStatusString());
    }
}

#endif

#ifdef PIR_ENABLED
bool pirState = false;

void setupPIR()
{
    pinMode(PIR_PIN, INPUT);
}

void loopPIR()
{
    if (digitalRead(PIR_PIN) == HIGH)
    {
        if (pirState == false)
        {
            pirState = true;
            sendMQTTMessage(MQTT_TOPIC_MOTION, "1", false);
        }
    }
    else
    {
        if (pirState == true)
        {
            pirState = false;
            sendMQTTMessage(MQTT_TOPIC_MOTION, "0", false);
        }
    }
}
#endif

#ifdef SERVO_ENABLED
#include <Servo.h>

Servo servo;

void setupServo()
{
    servo.attach(SERVO_PIN);
    servo.write(0);
}

void loopServo()
{
}

void handleServo(DynamicJsonDocument json)
{
    String method = json["method"].as<String>();
    int degree = json["degree"].as<int>();

    if (!degree)
        degree = 30;

    if (method == "degree")
    {
        servo.write(degree);
        sendMQTTMessage(MQTT_TOPIC_SERVO, "1", false);
        return;
    }

    if (method == "push")
    {
        int delayMs = json["delayMs"].as<int>();
        int cycles = json["cycles"].as<int>();
        int delayMsBetweenCycles = json["delayMsBetweenCycles"].as<int>();

        if (!delayMs)
            delayMs = 600;
        if (!cycles)
            cycles = 1;
        if (!delayMsBetweenCycles)
            delayMsBetweenCycles = 1000;

        for (int i = 0; i < cycles; i++)
        {
            servo.write(degree);
            delay(delayMs);
            servo.write(0);

            if ((i + 1) != cycles)
            {
                delay(delayMsBetweenCycles);
            }
        }

        sendMQTTMessage(MQTT_TOPIC_SERVO, "1", false);
        return;
    }

    Serial.printf("Unknown servo method: %s", method.c_str());
}

#endif

void mqttCallback(char *topic, byte *payload, unsigned int length)
{
    char payloadText[length + 1];
    snprintf(payloadText, length + 1, "%s", payload);
    Serial.printf("MQTT callback with topic <%s> and payload <%s>", topic, payloadText);

    DynamicJsonDocument commandJson(256);
    DeserializationError err = deserializeJson(commandJson, payloadText);

    if (err)
    {
        Serial.printf("Error deserializing JSON");
        return;
    }

    String command = commandJson["command"].as<String>();
#ifdef SERVO_ENABLED
    if (command == "servo")
    {
        handleServo(commandJson);
        return;
    }
#endif

    Serial.printf("Unknown callback command: %s", command.c_str());
}

// -------------------

void setupMQTT()
{
    mqttClient.setClient(wifiClient);

    mqttClient.setServer(mqtt_server, 1883);
    mqttClient.setKeepAlive(10);
    mqttClient.setBufferSize(2048);
    mqttClient.setCallback(mqttCallback);

    mqttEnsureConnected();
}

void setup()
{
    setupGeneric();

    setupWifi();
    setupMDNS();
    setupOTA();
    setupMQTT();

#ifdef PIR_ENABLED
    setupPIR();
#endif
#ifdef DHT_ENABLED
    setupDHT();
#endif
#ifdef SERVO_ENABLED
    setupServo();
#endif
}

void loop()
{
    currentTime = millis();

    loopWifi();
    loopMDNS();
    loopOTA();
    loopMQTT();

#ifdef PIR_ENABLED
    loopPIR();
#endif
#ifdef DHT_ENABLED
    loopDHT();
#endif
#ifdef SERVO_ENABLED
    loopServo();
#endif
}