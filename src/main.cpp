#include "Arduino.h"
#include <ESP8266WiFi.h>
#include <WiFiManager.h>
#include <ESP8266mDNS.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <ArduinoOTA.h>

#define MQTT_POLLING_TIMEOUT 60000

#ifndef BOARD_ID
#define BOARD_ID "esp8266"
#endif

char mqtt_server[64];
char mqtt_username[64];
char mqtt_password[64];

#define AVAILABILITY_ONLINE "online"
#define AVAILABILITY_OFFLINE "offline"

char MQTT_TOPIC_AVAILABILITY[80];
char MQTT_TOPIC_CALLBACK[80];
char MQTT_TOPIC_DEBUG[80];

char MQTT_TOPIC_TEMPERATURE[80];
char MQTT_TOPIC_HUMIDITY[80];
char MQTT_TOPIC_MOTION[80];
char MQTT_TOPIC_SERVO[80];
char MQTT_TOPIC_UP_DOWN[80];
char MQTT_TOPIC_IRDA[80];
char MQTT_TOPIC_AMBIENT_LIGHT[80];

WiFiManager wifiManager;
WiFiClient wifiClient;
PubSubClient mqttClient;

WiFiManagerParameter wifi_param_mqtt_server("mqtt_server", "MQTT server", mqtt_server, sizeof(mqtt_server));
WiFiManagerParameter wifi_param_mqtt_username("mqtt_user", "MQTT username", mqtt_username, sizeof(mqtt_username));
WiFiManagerParameter wifi_param_mqtt_password("mqtt_pass", "MQTT password", mqtt_password, sizeof(mqtt_password));

unsigned long mqttLastTime = millis();

void sendMQTTMessage(const char *topic, const char *message, bool retained)
{
    Serial.printf("MQTT message - topic: <%s>, message: <%s> -> ", topic, message);

    if (mqttClient.publish(topic, message))
    {
        Serial.println("sent");
    }
    else
    {
        Serial.println("error");
    }
}

void logError(const char *msg)
{
    Serial.print("[ERROR] ");
    Serial.println(msg);
    sendMQTTMessage(MQTT_TOPIC_DEBUG, msg, false);
}

void saveConfig()
{
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

    serializeJson(json, configFile);
    configFile.close();

    Serial.printf("Saved JSON: %s", json.as<String>().c_str());
}

void loadConfig()
{
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
}

void setupGeneric()
{
    Serial.begin(9600);

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    loadConfig();

    Serial.printf("Board Identifier: %s\n", BOARD_ID);

    snprintf(MQTT_TOPIC_AVAILABILITY, 80, "%s/availability", BOARD_ID);
    snprintf(MQTT_TOPIC_CALLBACK, 80, "%s/callback", BOARD_ID);
    snprintf(MQTT_TOPIC_DEBUG, 80, "%s/debug", BOARD_ID);

    snprintf(MQTT_TOPIC_TEMPERATURE, 80, "%s/temperature", BOARD_ID);
    snprintf(MQTT_TOPIC_HUMIDITY, 80, "%s/humidity", BOARD_ID);
    snprintf(MQTT_TOPIC_MOTION, 80, "%s/motion", BOARD_ID);
    snprintf(MQTT_TOPIC_SERVO, 80, "%s/servo", BOARD_ID);
    snprintf(MQTT_TOPIC_IRDA, 80, "%s/irda", BOARD_ID);
    snprintf(MQTT_TOPIC_AMBIENT_LIGHT, 80, "%s/ambientlight", BOARD_ID);
    snprintf(MQTT_TOPIC_UP_DOWN, 80, "%s/updown", BOARD_ID);
}

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
        digitalWrite(LED_BUILTIN, HIGH);
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

void mqttConnect()
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

    if (millis() < mqttLastTime + MQTT_POLLING_TIMEOUT)
    {
        return;
    }

    Serial.println("Connection to MQTT lost, reconnecting...");
    mqttLastTime = millis();

    mqttConnect();
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

#ifndef DHT_POLLING_TIMEOUT
#define DHT_POLLING_TIMEOUT 60000
#endif

DHTesp dht;
unsigned long dhtLastTime = 0;

void setupDHT()
{
    dht.setup(DHT_PIN, DHTesp::DHT11);
}

void loopDHT()
{
    if (millis() < dhtLastTime + DHT_POLLING_TIMEOUT)
    {
        return;
    }

    dhtLastTime = millis();

    TempAndHumidity values = dht.getTempAndHumidity();

    Serial.printf("DHT: %f %f\n", values.temperature, values.humidity);

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

#ifndef UP_PIN
#define UP_PIN 5
#endif

#ifndef DOWN_PIN
#define DOWN_PIN 4
#endif

#ifdef UP_DOWN_ENABLED

void setupUpDown()
{
    pinMode(UP_PIN, OUTPUT);
    digitalWrite(UP_PIN, LOW);

    pinMode(DOWN_PIN, OUTPUT);
    digitalWrite(DOWN_PIN, LOW);
}

void loopUpDown()
{
}

void handleUpDown(DynamicJsonDocument json)
{
    String direction = json["direction"].as<String>();
    uint8_t pin = 0;
    if (direction == "up")
    {
        pin = UP_PIN;
    }
    else if (direction == "down")
    {
        pin = DOWN_PIN;
    }
    else
    {
        logError("Invalid direction");
        return;
    }

    unsigned long int timeMs = json["timeMs"].as<unsigned long int>();
    if (!timeMs)
    {
        timeMs = 1000;
    }

    digitalWrite(pin, HIGH);
    delay(timeMs);
    digitalWrite(pin, LOW);
    delay(200);

    sendMQTTMessage(MQTT_TOPIC_UP_DOWN, "on", false);
}

#endif

#ifdef PIR_ENABLED

#ifndef PIR_POLLING_TIMEOUT
#define PIR_POLLING_TIMEOUT 2000
#endif

bool pirState = false;
unsigned long pirLastTime = 0;

void setupPIR()
{
    pinMode(PIR_PIN, INPUT);
}

void loopPIR()
{
    if (millis() < pirLastTime + PIR_POLLING_TIMEOUT)
    {
        return;
    }

    pirLastTime = millis();
    int pirValue = digitalRead(PIR_PIN);

    Serial.printf("PIR: %d\n", pirValue);

    if (pirValue == HIGH)
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

    digitalWrite(LED_BUILTIN, !pirState);
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

        sendMQTTMessage(MQTT_TOPIC_SERVO, "on", false);
        return;
    }

    Serial.printf("Unknown servo method: %s", method.c_str());
}
#endif

#ifdef IRDA_ENABLED
#include <IRremote.h>

void setupIRDA()
{
#ifdef IRDA_RECV_PIN
    IrReceiver.begin(IRDA_RECV_PIN);
#endif
#ifdef IRDA_SEND_PIN
    IrSender.begin(IRDA_SEND_PIN);
#endif
}

void loopIRDA()
{
#ifdef IRDA_RECV_PIN
    if (IrReceiver.decode())
    {
        IrReceiver.printIRResultShort(&Serial);
        IrReceiver.resume();
    }
#endif
}

void handleIRDA(DynamicJsonDocument json)
{
    uint16_t address = json["address"].as<uint16_t>();
    uint8_t code = json["code"].as<uint8_t>();
    if (!address || !code)
    {
        logError("IRDA_UNDEFINED_CODE");
        return;
    }

    Serial.printf("IRDA sending: %d %d\n", address, code);

    digitalWrite(LED_BUILTIN, LOW);

    IrSender.sendNEC(address, code, 1);
    sendMQTTMessage(MQTT_TOPIC_IRDA, "on", false);

    digitalWrite(LED_BUILTIN, HIGH);
}
#endif

#ifdef AMBIENT_LIGHT_ENABLED

#ifndef AMBIENT_LIGHT_POLLING_TIMEOUT
#define AMBIENT_LIGHT_POLLING_TIMEOUT 2000
#endif

unsigned long ambientLightLastTime = 0;

void setupAmbientLight()
{
}

void loopAmbientLight()
{
    if (millis() < ambientLightLastTime + AMBIENT_LIGHT_POLLING_TIMEOUT)
    {
        return;
    }

    ambientLightLastTime = millis();

    int value = analogRead(A0);
    Serial.printf("Ambient light: %d\n", value);

    sendMQTTMessage(MQTT_TOPIC_AMBIENT_LIGHT, String(value).c_str(), false);
}

#endif

void mqttCallback(char *topic, byte *payload, unsigned int length)
{
    char payloadText[length + 1];
    snprintf(payloadText, length + 1, "%s", payload);

    Serial.printf("MQTT callback with topic <%s> and payload <%s>", topic, payloadText);
    sendMQTTMessage(MQTT_TOPIC_DEBUG, payloadText, false);

    DynamicJsonDocument commandJson(256);
    DeserializationError err = deserializeJson(commandJson, payloadText);

    if (err)
    {
        logError("INVALID_JSON");
        return;
    }

    String command = commandJson["command"].as<String>();

    if (command == "availability")
    {
        sendMQTTMessage(MQTT_TOPIC_AVAILABILITY, AVAILABILITY_ONLINE, true);
        return;
    }

#ifdef UP_DOWN_ENABLED
    if (command == "updown")
    {
        handleUpDown(commandJson);
        return;
    }
#endif

#ifdef SERVO_ENABLED
    if (command == "servo")
    {
        handleServo(commandJson);
        return;
    }
#endif

#ifdef IRDA_ENABLED
    if (command == "irda")
    {
        handleIRDA(commandJson);
        return;
    }
#endif

    Serial.printf("Unknown callback command: %s", command.c_str());
    logError("MQTT_INVALID_COMMAND");
}

// -------------------

void setupMQTT()
{
    mqttClient.setClient(wifiClient);

    mqttClient.setServer(mqtt_server, 1883);
    mqttClient.setKeepAlive(10);
    mqttClient.setBufferSize(2048);
    mqttClient.setCallback(mqttCallback);

    mqttConnect();
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
#ifdef IRDA_ENABLED
    setupIRDA();
#endif
#ifdef AMBIENT_LIGHT_ENABLED
    setupAmbientLight();
#endif
#ifdef UP_DOWN_ENABLED
    setupUpDown();
#endif
}

void loop()
{
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
#ifdef IRDA_ENABLED
    loopIRDA();
#endif
#ifdef AMBIENT_LIGHT_ENABLED
    loopAmbientLight();
#endif
#ifdef UP_DOWN_ENABLED
    loopUpDown();
#endif
}