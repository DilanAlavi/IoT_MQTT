#include <WiFi.h>
#include <PubSubClient.h>

// Constants and configuration data
const char* WIFI_SSID = "COMTECO-N4197055";
const char* WIFI_PASS = "VDLTT61050";
const char* MQTT_BROKER_HOST = "broker.hivemq.com";
const int MQTT_BROKER_PORT = 1883;
const char* MQTT_CLIENT_ID = "grupo9_iot";    
const char* PUBLISH_TOPIC = "ucb.test/alavi";
const char* SUBSCRIBE_TOPIC = "ucb.test/alavi";

// Pin definitions
struct Pins {
    static const int TRIGGER = 18;
    static const int ECHO = 19;
    static const int LED_10 = 13;
    static const int LED_20 = 12;
    static const int LED_30 = 14;
    static const int LED_40 = 27;
};

class UltrasonicSensor {
public:
    UltrasonicSensor(int triggerPin, int echoPin) : triggerPin(triggerPin), echoPin(echoPin) {
        pinMode(triggerPin, OUTPUT);
        pinMode(echoPin, INPUT);
    }

    long measureDistance() {
        digitalWrite(triggerPin, LOW);
        delayMicroseconds(2);
        digitalWrite(triggerPin, HIGH);
        delayMicroseconds(10);
        digitalWrite(triggerPin, LOW);
        long duration = pulseIn(echoPin, HIGH);
        return (duration / 2) / 29.1;
    }

private:
    int triggerPin, echoPin;
};

class LEDController {
public:
    LEDController(int pin10, int pin20, int pin30, int pin40)
    : pins{pin10, pin20, pin30, pin40} {
        for (int i = 0; i < 4; ++i) {
            pinMode(pins[i], OUTPUT);
        }
    }

    void controlLEDs(long distance) {
        digitalWrite(pins[0], distance < 10);
        digitalWrite(pins[1], distance >= 10 && distance < 20);
        digitalWrite(pins[2], distance >= 20 && distance < 30);
        digitalWrite(pins[3], distance >= 30);
    }

    void turnOffAllLEDs() {
        for (int i = 0; i < 4; ++i) {
            digitalWrite(pins[i], LOW);
        }
    }

private:
    int pins[4];
};

class MQTTHandler {
public:
    MQTTHandler(const char* host, int port, const char* clientId, 
                const char* pubTopic, const char* subTopic) 
    : mqttClient(wifiClient), publishTopic(pubTopic), subscribeTopic(subTopic) {
        mqttClient.setServer(host, port);
        mqttClient.setCallback([this](char* topic, byte* payload, unsigned int length) {
            this->callback(topic, payload, length);
        });
    }

    void ensureConnected() {
        if (!mqttClient.connected()) {
            reconnect();
        }
        mqttClient.loop();
    }

    void publishMessage(const String& message) {
        if (!mqttClient.publish(publishTopic, message.c_str())) {
            Serial.println("Publish failed");
        }
    }

    void activateSystem(bool activate) {
        systemActive = activate;
        if (!activate) ledController.turnOffAllLEDs();
    }

    bool isSystemActive() const {
        return systemActive;
    }

private:
    WiFiClient wifiClient;
    PubSubClient mqttClient;
    const char* publishTopic;
    const char* subscribeTopic;
    bool systemActive = false;
    LEDController ledController = LEDController(Pins::LED_10, Pins::LED_20, Pins::LED_30, Pins::LED_40);

    void reconnect() {
        while (!mqttClient.connected()) {
            Serial.print("Attempting MQTT connection...");
            if (mqttClient.connect(MQTT_CLIENT_ID)) {
                Serial.println("connected");
                mqttClient.subscribe(subscribeTopic);
            } else {
                Serial.print("failed, rc=");
                Serial.print(mqttClient.state());
                Serial.println(" try again in 5 seconds");
                delay(5000);
            }
        }
    }

    void callback(const char* topic, byte* payload, unsigned int length) {
        String message;
        for (int i = 0; i < length; i++) {
            message += (char)payload[i];
        }
        Serial.println("Message received: " + message);

        if (message == "LED_ON") {
            activateSystem(true);
        } else if (message == "LED_OFF") {
            activateSystem(false);
        }
    }
};

// Global objects
UltrasonicSensor ultrasonic(Pins::TRIGGER, Pins::ECHO);
MQTTHandler mqtt(MQTT_BROKER_HOST, MQTT_BROKER_PORT, MQTT_CLIENT_ID, PUBLISH_TOPIC, SUBSCRIBE_TOPIC);

void setup() {
    Serial.begin(115200);
    connectToWiFi();
    mqtt.ensureConnected();
}

void loop() {
    mqtt.ensureConnected();
    if (mqtt.isSystemActive()) {
        long distance = ultrasonic.measureDistance();
        mqtt.publishMessage(String(distance) + " cm");
    }
}

void connectToWiFi() {
    Serial.print("Connecting to WiFi...");
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("WiFi connected");
}
