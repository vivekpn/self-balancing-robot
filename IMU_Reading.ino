#include <Wire.h>

#define ACC_ADDRESS 0x53

// Gyroscope ITG3200 
#define GYRO 0x68 // gyro address, binary = 11101001 when AD0 is connected to Vcc (see schematics of your breakout board)
#define G_SMPLRT_DIV 0x15
#define G_DLPF_FS 0x16
#define G_INT_CFG 0x17
#define G_PWR_MGM 0x3E

#define G_TO_READ 8 // 2 bytes for each axis x, y, z

// offsets are chip specific. 
int g_offx = -90;
int g_offy = -50;
int g_offz = 10;

/*void setup()
{
//  Serial.begin(9600);
 // Wire.begin();
 // init_ACC();
 // init_Gyro();
 // byte id[1];
 // readFrom(ACC_ADDRESS, 0x00,1,id);
 // Serial.print("id= ");Serial.println(id[0],HEX);
  
  
} */

void IMU_Reading(float IMU_val[]) 
{
  float Rx,Ry,Rz;
  float pitch,roll,X_rate,Y_rate,IMU_Roll,IMU_Pitch;
  int ACC_Data[3],Gyro_Data[4];
  static int this_time=0,previous_time=0,dt=0;
  
  
  init_ACC();
  init_Gyro();
  
  get_Acc_Data(ACC_Data);
  Rx=ACC_Data[0];
  Ry=ACC_Data[1];
  Rz=ACC_Data[2];
  pitch=57*atan2(Ry,sqrt(Rx*Rx+Rz*Rz));
  roll=57*atan2(Rx,sqrt(Ry*Ry+Rz*Rz));
 // Serial.print("  roll: ");Serial.print(roll); 
 // Serial.print("  pitch: ");
 //Serial.println(pitch); 
  
  get_Gyro_Data(Gyro_Data);
  X_rate = float(Gyro_Data[0])/14.375;
  Y_rate = float(Gyro_Data[1])/14.375;
  
  this_time=millis();
  dt=this_time-previous_time;
 //   Serial.print(" dt : ");Serial.println(dt);  
  
  // kalman
 // IMU_Roll =kalmanCalculateRoll(roll,-Y_rate,dt);  // directions of roll and y_rate are opposite 
  IMU_Pitch =kalmanCalculatePitch(pitch,X_rate,dt);
 // IMU_val[0] = IMU_Roll; 
    IMU_val[1] = IMU_Pitch;
    previous_time = this_time; 
//  Serial.print("  x rate : ");Serial.print(X_rate); 
 // Serial.print("  y rate: ");Serial.println(Y_rate);  
 // Serial.print(" IMU Roll: ");
//  Serial.println(IMU_Roll);  
 // Serial.print(" IMU Pitch: ");Serial.println(IMU_Pitch);  
 // delay(40);
  
}


void init_ACC()
{
  
  // initial power mode settings to change from standby mode to measurement mode
  writeTo(ACC_ADDRESS,0x2D,0);
  writeTo(ACC_ADDRESS,0x2D,16);
  writeTo(ACC_ADDRESS,0x2D,8);
  
}

void init_Gyro()
{
  /*****************************************
  * ITG 3200
  * power management set to:
  * clock select = internal oscillator
  *     no reset, no sleep mode
  *   no standby mode
  * sample rate to = 125Hz
  * parameter to +/- 2000 degrees/sec
  * low pass filter = 5Hz
  * no interrupt
  ******************************************/
  writeTo(GYRO, G_PWR_MGM, 0x01);
  writeTo(GYRO, G_SMPLRT_DIV, 0x07); // EB, 50, 80, 7F, DE, 23, 20, FF
  writeTo(GYRO, G_DLPF_FS, 0x1E); // +/- 2000 dgrs/sec, 1KHz, 1E, 19
  writeTo(GYRO, G_INT_CFG, 0x00);
}
void get_Acc_Data(int * ACC_result)
{
  byte data[6];
  readFrom(ACC_ADDRESS,0x32,6,data); // read 6 bytes from adress 0x32 and return it in data array
  ACC_result[0]=(((int)data[1])<<8)|data[0]; 
  ACC_result[1]=(((int)data[3])<<8)|data[2]; 
  ACC_result[2]=(((int)data[5])<<8)|data[4]; 
}


void get_Gyro_Data(int * result)
{
  /**************************************
  Gyro ITG-3200 I2C
  registers:
  temp MSB = 1B, temp LSB = 1C
  x axis MSB = 1D, x axis LSB = 1E
  y axis MSB = 1F, y axis LSB = 20
  z axis MSB = 21, z axis LSB = 22
  *************************************/

  int regAddress = 0x1B;
  int temp, x, y, z;
  byte buff[G_TO_READ];
  
  readFrom(GYRO, regAddress, G_TO_READ, buff); //read the gyro data from the ITG3200
  
  result[0] = ((buff[2] << 8) | buff[3]) + g_offx;
  result[1] = ((buff[4] << 8) | buff[5]) + g_offy;
  result[2] = ((buff[6] << 8) | buff[7]) + g_offz;
  result[3] = (buff[0] << 8) | buff[1]; // temperature
  
}

void writeTo(int Device,byte Add,byte data)
{
  Wire.beginTransmission(Device);
  Wire.write(Add);
  Wire.write(data);
  Wire.endTransmission();
}

void readFrom(int Device,byte Add,int num_of_bytes,byte buff[])
{
  Wire.beginTransmission(Device);
  Wire.write(Add);
  Wire.endTransmission();
  Wire.beginTransmission(Device);
  Wire.requestFrom(Device,num_of_bytes);
  int i=0;
  while(Wire.available())
  {
    buff[i++]=Wire.read();
  }
  Wire.endTransmission();
}
