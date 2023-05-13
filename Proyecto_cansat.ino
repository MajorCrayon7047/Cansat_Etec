#include <DHT_U.h>
#include <DHT.h>
#include <Adafruit_BMP085.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include <BH1750.h>

//variables globales
float bmp180[3];
float dht11[2];
int16_t mpu6050[6];
uint16_t luz;
float gasVoltage;
#define DHTPIN 2
#define DHTTYPE    DHT22
const int mpuAddress = 0x68;  //Puede ser 0x68 o 0x69
int16_t ax, ay, az;
int16_t gx, gy, gz;

//objetos:
Adafruit_BMP085 bmp;
DHT dht(DHTPIN, DHTTYPE);
MPU6050 mpu(mpuAddress);
BH1750 Luxometro;

//Funciones:
void read_bmp(){
    bmp180[0] = bmp.readTemperature();
    bmp180[1] = bmp.readPressure() * 0.01;
    bmp180[2] = bmp.readAltitude(1080.0); //esto hay que revisarlo
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
    int read = analogRead(A0)
    gasVoltage = read * (3.3 / 1023.0);
}

void setup(){
	Serial.begin(115200);
    if (!bmp.begin()){
        Serial.println("No se encontro el sensor BMP180 ahora me quedare en un bucle infinito, jodete");
        while(1) {}
    }
    dht.begin();
    Wire.begin();
    mpu.initialize();
    Luxometro.begin(BH1750_CONTINUOUS_HIGH_RES_MODE);
    Serial.println(mpu.testConnection() ? F("IMU iniciado correctamente") : F("Error al iniciar IMU"));
}

void loop(){
	//ACA VA LA COMUNICACION DE LORA POINT TO POINT (PTP)
}
