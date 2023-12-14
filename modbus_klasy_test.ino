#include <MQTT.h>
#include <WiFi.h>

#include "IPAddress.h"
#include "device.h"
#include "comms.h"
#include "secrets.h"

#define MAIN_LOOP_DELAY 10

bool is_connected = false;

char ssid[] = SECRET_SSID;
char pass[] = SECRET_PASS;

WiFiClient wifi_client;
MQTTClient mqtt_client;

IntSerializer int_ser;

#define RS485_HW_SERIAL Serial1
#define RS485_TX_PIN 0
#define RS485_DE_RE_PIN 2
RS485Class rs485(RS485_HW_SERIAL, RS485_TX_PIN, RS485_DE_RE_PIN, RS485_DE_RE_PIN);

#define MODBUS_SLAVE_ID 1

InputDevice* in_devices[MAX_INPUT_DEVICES_COUNT];
OutputDevice* out_devices[MAX_OUTPUT_DEVICES_COUNT];
#ifdef COMMS_MQTT
CommsMqttInput comms_in(in_devices,&mqtt_client);
CommsMqttOutput comms_out(out_devices,&mqtt_client);
#else
CommsModbusInput comms_in(in_devices,&rs485);
CommsModbusOutput comms_out(out_devices,&rs485);
#endif


void connect() {
    digitalWrite(LED_BUILTIN,HIGH);

    IPAddress ip = IPAddress(192,168,50,186);
    mqtt_client.begin(ip,1883, wifi_client);
    Serial.print("checking wifi...");
    while (WiFi.status() != WL_CONNECTED) {
        Serial.print(".");
        delay(1000);
    }

    Serial.print("\nconnecting...");
    while (!mqtt_client.connect(__FILE__)) {
        Serial.print(".");
        delay(1000);
    }

    Serial.println("\nconnected!");
    is_connected = true;
}

void connect_modbus(){
    if (!ModbusRTUServer.begin(rs485,MODBUS_SLAVE_ID,9600,SERIAL_8N1)){
        while(1){
            Serial.println("Failed to start Modbus RTU Server!");
            delay(500);
        }
    }
    is_connected = true;
}

void setup() {
    Serial.begin(115200);

#ifdef COMMS_MQTT
    WiFi.begin(ssid, pass);
    connect();
#else
    connect_modbus();
#endif

    comms_in.add_device(new DigitalWriteInputDevice((Serializer*) &int_ser,"ledzik",16));
    comms_out.add_device(new ButtonDevice((Serializer*) &int_ser,"batoooonik",17));

    comms_in.init();
    comms_out.init();

#ifdef COMMS_MQTT
    comms_in.publish_all_topics_debug();
#endif
}

void setup1() {
    pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
#ifdef COMMS_MQTT
    if (!mqtt_client.connected()) {
        is_connected = false;
        connect();
    }
#endif

    comms_out.output_all();
    comms_in.loop();

    delay(MAIN_LOOP_DELAY);
}

void loop1(){
    if(is_connected){
        static long int last_blink_time_millis = 0;
        long int current_millis = millis();

        if (current_millis >= last_blink_time_millis + 350) {
            digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
            last_blink_time_millis = current_millis;
        }
    }
}