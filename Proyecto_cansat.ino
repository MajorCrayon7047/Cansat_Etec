#include <DHT_U.h>
#include <DHT.h>
#include <Adafruit_BMP085.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include <BH1750.h>
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
void read_gas(){
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

void setup(){
	Serial.begin(9600);
    gpsSerial.begin(9600);
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

    //Envio de paquete via LoRa en formato JSON para decodificarlo en el receptor
    LoRa.beginPacket();
    LoRa.setTxPower(5,RF_PACONFIG_PASELECT_PABOOST);
    LoRa.print(F("{hum:"));
    LoRa.print(dht11[1]);
    LoRa.print(",temp:");
    LoRa.print(bmp180[0]);
    LoRa.print(",press:");
    LoRa.print(bmp180[1]);
    LoRa.print(",alt:");
    LoRa.print(bmp180[2]);
    LoRa.print(",gas:");
    LoRa.print(gasVoltage);
    LoRa.print(",luz:");
    LoRa.print(luz);
    LoRa.print(",acele:[");
    LoRa.print(mpu6050[0]);
    LoRa.print(",");
    LoRa.print(mpu6050[1]);
    LoRa.print(",");
    LoRa.print(mpu6050[2]);
    LoRa.print("],giro:");
    LoRa.print(mpu6050[3]);
    LoRa.print(",");
    LoRa.print(mpu6050[4]);
    LoRa.print(",");
    LoRa.print(mpu6050[5]);
    LoRa.print(",GPS:[");
    LoRa.print(NEO_gps[0]);
    LoRa.print(",");
    LoRa.print(NEO_gps[1]);
    LoRa.print(",");
    LoRa.print(NEO_gps[2]);
    LoRa.print("]}");
    LoRa.endPacket();
    }
}