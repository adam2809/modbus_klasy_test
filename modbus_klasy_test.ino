#include <MQTT.h>
#include <WiFi.h>

#include "IPAddress.h"
#include "device.h"
#include "comms.h"
#include "secrets.h"
#include <ArduinoRS485.h>

#define MAIN_LOOP_DELAY 10

bool is_connected = false;

char ssid[] = SECRET_SSID;
char pass[] = SECRET_PASS;

WiFiClient wifi_client;
MQTTClient mqtt_client;

#define RS485_HW_SERIAL Serial1
#define RS485_TX_PIN 0
#define RS485_DE_RE_PIN 15
RS485Class rs485(RS485_HW_SERIAL, RS485_TX_PIN, RS485_DE_RE_PIN, RS485_DE_RE_PIN);

IntSerializer int_ser;

OutputDevice* out_devices_arr[MAX_OUTPUT_DEVICES_COUNT];
Vector<OutputDevice*> out_devices(out_devices_arr);
CommsMqttOutput comms_mqtt_out(&out_devices,&mqtt_client);
CommsModbusOutput comms_modbus_out(&out_devices,&rs485);

InputDevice* in_devices_arr[MAX_INPUT_DEVICES_COUNT];
Vector<InputDevice*> in_devices(in_devices_arr);
CommsMqttInput comms_mqtt_in(&in_devices,&mqtt_client);
CommsModbusInput comms_modbus_in(&in_devices,&rs485);

void connect_wifi() {
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
    if (!ModbusRTUServer.begin(rs485,1,9600,SERIAL_8N1)){
        while(1){
            Serial.println("Failed to start Modbus RTU Server!");
            delay(500);
        }
    }
}

void setup() {
    digitalWrite(LED_BUILTIN,HIGH);
    Serial.begin(115200);

    WiFi.begin(ssid, pass);
    connect_wifi();
    connect_modbus();

    in_devices.push_back(new DigitalWriteInputDevice((Serializer*) &int_ser,"ledzik",16));
    out_devices.push_back(new ButtonDevice((Serializer*) &int_ser,"batoooonik",17));

    comms_modbus_in.init();
    comms_modbus_out.init();
    comms_mqtt_in.init();
    comms_mqtt_out.init();

    comms_mqtt_in.publish_all_topics_debug();
}

void setup1() {
    pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
    if (!mqtt_client.connected()) {
        is_connected = false;
        connect_wifi();
    }

    comms_mqtt_out.output_all();
    comms_mqtt_in.loop();

    comms_modbus_out.output_all();
    comms_modbus_in.loop();

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