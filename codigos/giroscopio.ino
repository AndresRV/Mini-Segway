#include <Wire.h>
 #include <Timer.h>
#define DEBUG true
 
//Direccion I2C de la IMU
#define MPU 0x68
 
//Ratios de conversion
#define A_R 16384.0
#define G_R 131.0
 
//Conversion de radianes a grados 180/PI
#define RAD_A_DEG = 57.295779
 
//MPU-6050 da los valores en enteros de 16 bits
//Valores sin refinar
int16_t AM,AcX, AcY, AcZ, GyX, GyY, GyZ;
 float GyYA,vel,u,K1,K2,Kr;
//Angulos
float Acc[2];
float Angle[2];

float Gy[2];
Timer t;
void setup()
{
Wire.begin();
Wire.beginTransmission(MPU);
Wire.write(0x6B);
Wire.write(0);
Wire.endTransmission(true);
Serial.begin(9600);
  t.every(10,control);
  K1=24.555808;
K2=0.3737;
Kr=25.04580;
pinMode(8,OUTPUT);
  pinMode(9,OUTPUT);
  pinMode(10,OUTPUT);
}
void loop()
{
 
   //Leer los valores del Acelerometro de la IMU
   Wire.beginTransmission(MPU);
   Wire.write(0x3B); //Pedir el registro 0x3B - corresponde al AcX
   Wire.endTransmission(false);
   Wire.requestFrom(MPU,6,true); //A partir del 0x3B, se piden 6 registros
   AcX=Wire.read()<<8|Wire.read(); //Cada valor ocupa 2 registros
   AcY=Wire.read()<<8|Wire.read();
   AcZ=Wire.read()<<8|Wire.read();
 
   //Se calculan los angulos Y, X respectivamente.
   Acc[1] = atan(-1*(AcX/A_R)/sqrt(pow((AcY/A_R),2) + pow((AcZ/A_R),2)))*RAD_TO_DEG;
   Acc[0] = atan((AcY/A_R)/sqrt(pow((AcX/A_R),2) + pow((AcZ/A_R),2)))*RAD_TO_DEG;
 
   //Leer los valores del Giroscopio
   Wire.beginTransmission(MPU);
   Wire.write(0x43);
   Wire.endTransmission(false);
   Wire.requestFrom(MPU,4,true); //A diferencia del Acelerometro, solo se piden 4 registros
   GyX=Wire.read()<<8|Wire.read();
   GyY=Wire.read()<<8|Wire.read();
 
   //Calculo del angulo del Giroscopio
   Gy[0] = GyX/G_R;
   Gy[1] = GyY/G_R;
 
   //Aplicar el Filtro Complementario
   Angle[0] = 0.98 *(Angle[0]+Gy[0]*0.010) + 0.02*Acc[0];
   Angle[1] = 0.98 *(Angle[1]+Gy[1]*0.010) + 0.02*Acc[1];
   t.update();  
   #if DEBUG  
   //Mostrar los valores por consola
   //Serial.print("Angle rX: "); Serial.print(Angle[0]); Serial.print("\n");
   //Serial.print("Angle rY: "); Serial.print(Angle[1]); Serial.print("\n------------\n");
   #endif
   
   //delay(10); //Nuestra dt sera, pues, 0.010, que es el intervalo de tiempo en cada medida
}
void control()
{
  vel=(Angle[1]-GyYA)/(0.01);
  u=-K1*Angle[1]-K2*vel+Kr*(0);
     //Serial.print("U: "); Serial.println(u); 
  if(u>0)
  {
   AM= map(u, 0,1700, 200,255);
    digitalWrite(8,HIGH);
  digitalWrite(9,LOW);
  analogWrite(10,AM);
  //Serial.print("m1: "); Serial.println(AM); 
//digitalWrite(10,LOW);  
   }
   else
   {
  AM= map(u, -1700,0, 255, 200);
    digitalWrite(8,LOW);
  digitalWrite(9,HIGH);
  analogWrite(10,AM); 
 //Serial.print("m2: "); Serial.println(AM); 
 //digitalWrite(10,LOW);  
  }
  GyYA=Angle[1];
  
}
