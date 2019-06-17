#include <SoftwareSerial.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

MPU6050 sensor;
char dato;
String a = "";
int ax, ay, az;
int gx, gy, gz;
long tiempo_prev;
float dt;
short ang_x, ang_y;
float ang_x_prev, ang_y_prev;
//Variables usadas por el filtro pasa bajos
long f_ax, f_ay, f_az;
int p_ax, p_ay, p_az;
long f_gx, f_gy, f_gz;
int p_gx, p_gy, p_gz;
int counter = 0;

//Valor de los offsets
int ax_o, ay_o, az_o;
int gx_o, gy_o, gz_o;

SoftwareSerial ModBluetooth(10, 11); // RX | TX

void setup()
{
  dato = '0';
  ModBluetooth.begin(9600);
  Serial.begin(9600);
  Wire.begin();
  sensor.initialize();
  ModBluetooth.println("MODULO CONECTADO");
  if (sensor.testConnection())
    Serial.println("Sensor iniciado correctamente");
  else
    Serial.println("Error al iniciar el sensor");

  // Leer los offset los offsets anteriores
  ax_o = sensor.getXAccelOffset();
  ay_o = sensor.getYAccelOffset();
  az_o = sensor.getZAccelOffset();
  gx_o = sensor.getXGyroOffset();
  gy_o = sensor.getYGyroOffset();
  gz_o = sensor.getZGyroOffset();

  Serial.println("Offsets:");
  Serial.print(ax_o);
  Serial.print("\t");
  Serial.print(ay_o);
  Serial.print("\t");
  Serial.print(az_o);
  Serial.print("\t");
  Serial.print(gx_o);
  Serial.print("\t");
  Serial.print(gy_o);
  Serial.print("\t");
  Serial.print(gz_o);
  Serial.print("\t");
}

void loop()
{
  if (ModBluetooth.available())
  {
    if (dato == '0')
    {
      do
      {
        dato = ModBluetooth.read();
        Serial.print("disconected");
        delay(1000);
      } while (dato != '1');
    }
    if (dato == '1')
    {
      do
      {
        dato = ModBluetooth.read();
        lecturaGiroAcel();
        delay(500);
      } while (dato != '0');
    }
  }
}

/*
  if (dato == "0")
  {
    Serial.print("If 0");
    Serial.print("dato");
  }
  if (dato == "1")
  {
    lecturaGiroAcel();
    Serial.print("dato");
  }
  dato = ModBluetooth.read();
 */

void lecturaGiroAcel()
{
  // Leer las aceleraciones y velocidades angulares
  sensor.getAcceleration(&ax, &ay, &az);
  sensor.getRotation(&gx, &gy, &gz);

  dt = (millis() - tiempo_prev) / 1000.0;
  tiempo_prev = millis();

  //Calcular los ángulos con acelerometro
  float accel_ang_x = atan(ay / sqrt(pow(ax, 2) + pow(az, 2))) * (180.0 / 3.14);
  float accel_ang_y = atan(-ax / sqrt(pow(ay, 2) + pow(az, 2))) * (180.0 / 3.14);

  //Calcular angulo de rotación con giroscopio y filtro complemento
  ang_x = 0.98 * (ang_x_prev + (gx / 131) * dt) + 0.02 * accel_ang_x;
  ang_y = 0.98 * (ang_y_prev + (gy / 131) * dt) + 0.02 * accel_ang_y;

  ang_x_prev = ang_x;
  ang_y_prev = ang_y;

  //Mostrar los angulos separadas por un [tab]
  //Serial.print("Rotacion en X:  ");
  //Serial.print(ang_x);
  // Serial.print("tRotacion en Y: ");
  // Serial.println(ang_y);
  String mystring = "/";
  mystring = mystring + ang_x;
  mystring = mystring + "#";
  mystring = mystring + ang_y;
  Serial.println(mystring);
  // Serial.print(mystring);

  ModBluetooth.print(mystring);
  delay(300);
}

void lecturaAngular()
{
  // Leer las aceleraciones y velocidades angulares
  sensor.getAcceleration(&ax, &ay, &az);
  sensor.getRotation(&gx, &gy, &gz);

  //Mostrar las lecturas separadas por un [tab]
  Serial.print("a[x y z] g[x y z]:\t");
  Serial.print(ax);
  Serial.print("\t");
  Serial.print(ay);
  Serial.print("\t");
  Serial.print(az);
  Serial.print("\t");
  Serial.print(gx);
  Serial.print("\t");
  Serial.print(gy);
  Serial.print("\t");
  Serial.println(gz);
  delay(100);
}

void calibrarDispositivo()
{
  Serial.println("nnEnvie cualquier caracter para empezar la calibracionnn");
  // Espera un caracter para empezar a calibrar
  while (true)
  {
    if (Serial.available())
      break;
  }
  Serial.println("Calibrando, no mover IMU");
  // Leer las aceleraciones y velocidades angulares
  sensor.getAcceleration(&ax, &ay, &az);
  sensor.getRotation(&gx, &gy, &gz);

  // Filtrar las lecturas
  f_ax = f_ax - (f_ax >> 5) + ax;
  p_ax = f_ax >> 5;

  f_ay = f_ay - (f_ay >> 5) + ay;
  p_ay = f_ay >> 5;

  f_az = f_az - (f_az >> 5) + az;
  p_az = f_az >> 5;

  f_gx = f_gx - (f_gx >> 3) + gx;
  p_gx = f_gx >> 3;

  f_gy = f_gy - (f_gy >> 3) + gy;
  p_gy = f_gy >> 3;

  f_gz = f_gz - (f_gz >> 3) + gz;
  p_gz = f_gz >> 3;

  //Cada 100 lecturas corregir el offset
  if (counter == 100)
  {
    //Mostrar las lecturas separadas por un [tab]
    Serial.print("promedio:");
    Serial.print("t");
    Serial.print(p_ax);
    Serial.print("\t");
    Serial.print(p_ay);
    Serial.print("\t");
    Serial.print(p_az);
    Serial.print("\t");
    Serial.print(p_gx);
    Serial.print("\t");
    Serial.print(p_gy);
    Serial.print("\t");
    Serial.println(p_gz);

    //Calibrar el acelerometro a 1g en el eje z (ajustar el offset)
    if (p_ax > 0)
      ax_o--;
    else
    {
      ax_o++;
    }
    if (p_ay > 0)
      ay_o--;
    else
    {
      ay_o++;
    }
    if (p_az - 16384 > 0)
      az_o--;
    else
    {
      az_o++;
    }

    sensor.setXAccelOffset(ax_o);
    sensor.setYAccelOffset(ay_o);
    sensor.setZAccelOffset(az_o);

    //Calibrar el giroscopio a 0º/s en todos los ejes (ajustar el offset)
    if (p_gx > 0)
      gx_o--;
    else
    {
      gx_o++;
    }
    if (p_gy > 0)
      gy_o--;
    else
    {
      gy_o++;
    }
    if (p_gz > 0)
      gz_o--;
    else
    {
      gz_o++;
    }

    sensor.setXGyroOffset(gx_o);
    sensor.setYGyroOffset(gy_o);
    sensor.setZGyroOffset(gz_o);

    counter = 0;
  }
  counter++;
}

void modBluetooth()
{
  if (ModBluetooth.available())
  {
    char VarChar;

    VarChar = ModBluetooth.read();

    if (VarChar == '1')
    {
      Serial.print("Giroscopio...");
      lecturaGiroAcel();
    }
    if (VarChar == '0')
    {
      Serial.print("LED APAGADO#");
    }
  }
}
