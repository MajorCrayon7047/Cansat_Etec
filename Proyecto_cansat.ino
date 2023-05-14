#include <DHT_U.h>
#include <DHT.h>
#include <Adafruit_BMP085.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include <BH1750.h>
#include <ArduinoJson.h>
#include "FS.h"
#include <LittleFS.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include "heltec.h"

//variables globales
uint16_t luz;
float gasVoltage;
#define DHTPIN 2
#define DHTTYPE    DHT22
#define BAND 915E6  //you can set band here directly,e.g. 433E6,868E6,915E6
const int mpuAddress = 0x68;  //Puede ser 0x68 o 0x69
int16_t ax, ay, az;
int16_t gx, gy, gz;
unsigned long ultima=0;

//En estos arrays se van a guardar los datos medidos por cada sensor
float bmp180[3];        
float dht11[2];
int16_t mpu6050[6];
float NEO_gps[3];

//objetos:
Adafruit_BMP085 bmp;
DHT dht(DHTPIN, DHTTYPE);
MPU6050 mpu(mpuAddress);
BH1750 Luxometro;
TinyGPSPlus gps;
SoftwareSerial gpsSerial(2, 3);  // RX, TX
TinyGPSPlus gps;

//Funciones para la lectura de sensores:
void read_bmp(){
    bmp180[0] = bmp.readTemperature();
    bmp180[1] = bmp.readPressure() * 0.01;
    bmp180[2] = bmp.readAltitude(1080.0); //lee la altitud tomando de referencia 1080.0 hPa
}
void read_dht(){
    dht11[0] = dht.readTemperature();
    dht11[1] = dht.readHumidity();
}
void read_MPU(){
    mpu.getAcceleration(&ax, &ay, &az);
    mpu.getRotation(&gx, &gy, &gz);
    mpu6050[0] = ax;
    mpu6050[1] = ay;
    mpu6050[2] = az;
    mpu6050[3] = gx;
    mpu6050[4] = gy;
    mpu6050[5] = gz;
}
void read_light(){
    luz = Luxometro.readLightLevel();
}
void read_gas(){    //MQ sensor de gas
    int read = analogRead(A0);
    gasVoltage = read * (3.3 / 1023.0);
}
void read_GPS(){
    while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      if (gps.location.isValid()) {
        // Obtener y imprimir la latitud, longitud y altitud
        Serial.print("Latitud: ");
        Serial.println(gps.location.lat(), 6);
        Serial.print("Longitud: ");
        Serial.println(gps.location.lng(), 6);
        Serial.print("Altitud: ");
        Serial.println(gps.altitude.meters());
        NEO_gps[0] = gps.location.lat();
        NEO_gps[1] = gps.location.lng();
        NEO_gps[2] = gps.location.meters();
      }
    }
  }
}

void save_local_data(String data){
        if (LittleFS.exists("/data.json")){
            File file = LittleFS.open("/data.json", "r");
            DynamicJsonBuffer  jsonBuffer(512);
            JsonObject& json = jsonBuffer.parseObject(file);
            delay(1);
            file.close();
            if (!json.success()) {
                Serial.println("parseObject() failed");
                return
                }
        else{
            DynamicJsonDocument json(512);
        }
        int jsonSize = json.size() + 1;
        json[jsonSize] = data;
        File file = LittleFS.open("/config.json", "w");
        if (!file){
            Serial.println("No funco");
            return;
        }
        file.print(json);
        delay(1);
        file.close();
  }

void setup(){
	Serial.begin(9600);
    gpsSerial.begin(9600);
    if(!LittleFS.begin()){
        Serial.println("An Error has occurred while mounting LittleFS");
        return;
    }
    if (!bmp.begin()){
        Serial.println("No se encontro el sensor BMP180 ahora me quedare en un bucle infinito");
        while(1) {}
    }
    dht.begin();
    Wire.begin();
    mpu.initialize();
    Luxometro.begin(BH1750_CONTINUOUS_HIGH_RES_MODE);
    Serial.println(mpu.testConnection() ? F("IMU iniciado correctamente") : F("Error al iniciar IMU"));
    Heltec.begin(true /*DisplayEnable Enable*/, true /*Heltec.LoRa Disable*/, true /*Serial Enable*/, true /*PABOOST Enable*/, BAND /*long BAND*/);  
}

void loop(){
    if (millis()>=ultima){
    ultima = millis() + 500;
	// leer sensores
    read_bmp();
    read_dht();
    read_gas();
    read_light();
    read_MPU();
    read_GPS();

    //Se almacenan en una variable los valores de los sensores en formato JSON
    String data = String("{hum:") + string(dht11[1]) + String(",temp:") + String(bmp180[0]) + String(",press:") + String(bmp180[1]) + String(",alt:") + String(bmp180[2]) + String(",gas:") + String(gasVoltage) + String(",luz:") + String(luz) + String(",acele:[") + String(mpu6050[0]) + String(",") + String(mpu6050[1]) + String(",") + String(mpu6050[2]) + String("],giro:") + String(mpu6050[3]) + String(",") + String(mpu6050[4]) + String(",") + String(mpu6050[5]) + String(",GPS:[") + String(NEO_gps[0]) + String(",") + String(NEO_gps[1]) + String(",") + String(NEO_gps[2]) + String("}]");

    //Se guarda los valores en el alamcenamiento interno del ESP32 en formato JSON
    save_local_data(data);
    
    //Envio de paquete via LoRa en formato JSON para decodificarlo en el receptor
    LoRa.beginPacket();     //CABECERA DEL PAQUETE DE DATOS
    LoRa.setTxPower(5,RF_PACONFIG_PASELECT_PABOOST);
    LoRa.print(data);
    LoRa.endPacket();       //COLA DEL PAQUETE DE DATOS
    }
}
//Realizado por MajorCrayon7047 (https://github.com/MajorCrayon7047)