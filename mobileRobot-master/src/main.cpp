#include <mbed.h>
#include"MPU6050.h"
#include <PwmOut.h>
#include "millis.h"

#define PERIOD 0.00002
#define CLOCKWISE 0
#define COUNTER_CLOCKWISE 1

//Encoder pin init
InterruptIn encoderPin1(PD_10);
InterruptIn encoderPin2(PD_9);
InterruptIn encoderPin3(PB_11);
InterruptIn encoderPin4(PB_12);

PwmOut pwFrontRightClockwise(PB_13);
PwmOut pwFrontRightCounterClockWise(PB_14);

PwmOut pwFrontLeftClockwise(PD_12);
PwmOut pwFrontLeftCounterClockWise(PD_13);

PwmOut pwRearRightClockwise(PD_14);
PwmOut pwRearRightCounterClockWise(PD_15);

PwmOut pwRearLeftClockwise(PB_4);
PwmOut pwRearLeftCounterClockWise(PB_5);

//UART communication to send and receive information
Serial serial(PA_2, PA_3);

//UART communication to debug through PC
Serial Debug(PB_10, PB_11);

//I2C communication MPU6050
I2C MPU6050(PB_7, PB_6); 

Ticker flipper;

unsigned long pulse1 = 0;
unsigned long pulse2 = 0;
unsigned long pulse3 = 0;
unsigned long pulse4 = 0;
unsigned long rpm1;
unsigned long rpm2;
unsigned long rpm3;
unsigned long rpm4;

// Circular buffers for serial TX and RX data - used by interrupt routines
const int buffer_size = 255;
char rx_buffer[buffer_size+1];
volatile int rx_in = 0;
volatile int rx_out = 0;
int i = 0;
char receiveBuff[256];
bool isStarted = false;
int countt = 0;
int newLineCount = 0;

void motor_count_1(void)
{
  pulse1++;
}

void motor_count_2(void)
{
  pulse2++;
}

void motor_count_3(void)
{
  pulse3++;
}

void motor_count_4(void)
{
  pulse4++;
}



//Struct motor speed 
struct motorSpeed_t
{
  uint32_t motor1SpeedIn;
  uint32_t motor2SpeedIn;
  uint32_t motor3SpeedIn;
  uint32_t motor4SpeedIn;

  uint32_t motor1SpeedOut;
  uint32_t motor2SpeedOut;
  uint32_t motor3SpeedOut;
  uint32_t motor4SpeedOut;
};

//Motor control function
void motor_control1(uint8_t speed, uint8_t direction);
void motor_control2(uint8_t speed, uint8_t direction);
void motor_control3(uint8_t speed, uint8_t direction);
void motor_control4(uint8_t speed, uint8_t direction);

struct motorSpeed_t motorSpeed;

void tick_handler(void) {
  rpm1 = pulse1 * 60;
  pulse1 = 0;
  motorSpeed.motor1SpeedOut = rpm1 / 7;

  rpm2 = pulse2 * 60;
  pulse2 = 0;
  motorSpeed.motor2SpeedOut = rpm2 / 7;

  rpm3 = pulse3 * 60;
  pulse3 = 0;
  motorSpeed.motor3SpeedOut = rpm3 / 7;

  rpm4 = pulse4 * 60;
  pulse4 = 0;
  motorSpeed.motor4SpeedOut = rpm4 / 7;
}

//Some MPU6050 functions
void MPU6050_write(uint8_t u8RegAddress, uint8_t u8Value);
uint8_t MPU6050_read(char u8RegAddress);
void MPU6050_init(void);
void MPU6050_getAccel(sAxisAcceleration* sAccel);
void MPU6050_getGyro(sAxisGyro* sGyro);

//Rx interrupt handler function
void Rx_interrupt(void);

//String parse function
void speed_string_parse(char* buffer, struct motorSpeed_t* motorSpeed);

int main() {

  flipper.attach(&tick_handler, 1.0);
  //startMillis();
  //Encoder pin init
  encoderPin1.rise(&motor_count_1);
  encoderPin2.rise(&motor_count_2);
  encoderPin3.rise(&motor_count_3);
  encoderPin4.rise(&motor_count_4);

  //Setup serial baudrate
  serial.baud(115200);
  Debug.baud(115200);

  //Setup serial Rx interrupt
  serial.attach(&Rx_interrupt, Serial::RxIrq);

  //Init MPU6050
  MPU6050_init();

  MPU6050.frequency(100000);

  //Init pwm output
  pwFrontRightClockwise.period(PERIOD);
  pwFrontRightCounterClockWise.period(PERIOD);
  pwFrontLeftClockwise.period(PERIOD);
  pwFrontLeftCounterClockWise.period(PERIOD);
  pwRearRightClockwise.period(PERIOD);
  pwRearRightCounterClockWise.period(PERIOD);
  pwRearLeftClockwise.period(PERIOD);
  pwRearLeftCounterClockWise.period(PERIOD);

  while(1) {
    // put your main code here, to run repeatedly:
    speed_string_parse(rx_buffer, &motorSpeed);

    motor_control1(motorSpeed.motor1SpeedIn/70, 0);

    motor_control2(motorSpeed.motor2SpeedIn/70, 0);

    motor_control3(motorSpeed.motor3SpeedIn/70, 0);

    motor_control4(motorSpeed.motor4SpeedIn/70, 0);

    speed_string_parse(rx_buffer, &motorSpeed);

    Debug.printf("pulse:%lu\n",pulse1);

    Debug.printf("Motor speed: %lu\n", rpm1);
    Debug.printf("string is %s\n", rx_buffer);
    Debug.printf("Count is %d\n", countt);
    Debug.printf("Newline Count is %d\n", newLineCount);
    Debug.printf("Speed m1: %lu speed m2: %lu  speed m3: %lu speed m4: %lu\n", motorSpeed.motor1SpeedIn, motorSpeed.motor2SpeedIn, motorSpeed.motor3SpeedIn, motorSpeed.motor4SpeedIn);

    serial.printf("$%lu,%lu,%lu,%lu,", motorSpeed.motor1SpeedOut, motorSpeed.motor2SpeedOut, motorSpeed.motor3SpeedOut, motorSpeed.motor4SpeedOut);
    //wait_ms(200);
  }
}

void motor_control1(uint8_t speed, uint8_t direction)
{
  float fSpeedPeriod = (PERIOD * speed) / 100;

  if (direction == CLOCKWISE) {
    pwFrontRightClockwise.pulsewidth(fSpeedPeriod);
    pwFrontRightCounterClockWise.pulsewidth(0);
  } else if (direction == COUNTER_CLOCKWISE) {
    pwFrontRightCounterClockWise.pulsewidth(fSpeedPeriod);
    pwFrontRightClockwise.pulsewidth(0);
  }
}

void motor_control2(uint8_t speed, uint8_t direction)
{
  float fSpeedPeriod = (PERIOD * speed) / 100;
  
  if (direction == CLOCKWISE) {
    pwFrontLeftClockwise.pulsewidth(fSpeedPeriod);
    pwFrontLeftCounterClockWise.pulsewidth(0);
  } else if (direction == COUNTER_CLOCKWISE) {
    pwFrontLeftCounterClockWise.pulsewidth(fSpeedPeriod);
    pwFrontLeftClockwise.pulsewidth(0);
  }
}

void motor_control3(uint8_t speed, uint8_t direction)
{
  float fSpeedPeriod = (PERIOD * speed) / 100;
  
  if (direction == CLOCKWISE) {
    pwRearRightClockwise.pulsewidth(fSpeedPeriod);
    pwRearRightCounterClockWise.pulsewidth(0);
  } else if (direction == COUNTER_CLOCKWISE) {
    pwRearRightCounterClockWise.pulsewidth(fSpeedPeriod);
    pwRearRightClockwise.pulsewidth(0);
  }
}

void motor_control4(uint8_t speed, uint8_t direction)
{
  float fSpeedPeriod = (PERIOD * speed) / 100;
  
  if (direction == CLOCKWISE) {
    pwRearLeftClockwise.pulsewidth(fSpeedPeriod);
    pwRearLeftCounterClockWise.pulsewidth(0);
  } else if (direction == COUNTER_CLOCKWISE) {
    pwRearLeftCounterClockWise.pulsewidth(fSpeedPeriod);
    pwRearLeftClockwise.pulsewidth(0);
  }
}


/*MPU6050 Initialization*/
void MPU6050_init(void)
{
  //First let's Wake up the MPU6050 by writing 0 to the Power management register
  MPU6050_write(MPU6050_REG_PWR_MGMT_1, MPU6050_WAKE_UP);
}

void MPU6050_write(uint8_t u8RegAddress, uint8_t u8Value)
{
  char cTxBuff[2];
  cTxBuff[0] = (char)u8RegAddress;
  cTxBuff[1] = (char)u8Value;


  //Send the Reg address and then send the value
  MPU6050.write(MPU6050_I2C_ADDR, cTxBuff, 2);
}

uint8_t MPU6050_read(char u8RegAddress)
{
  char cDataRcv = 0;

  //Send the Register Address and then read the value the MPU6050 sends back
  MPU6050.write(MPU6050_I2C_ADDR, &u8RegAddress, 1 );
  MPU6050.read(MPU6050_I2C_ADDR, &cDataRcv , 1);

  return (uint8_t)cDataRcv;
}

void MPU6050_getAccel(sAxisAcceleration* sAccel)
{
  char cAddress = MPU6050_REG_ACCEL_XOUT_H;
  uint8_t cAccel[6];

  //Read 6 registers from the Register X_ACCEL_OUT_H to get the value of 3 axis acceleration
  MPU6050.write(MPU6050_I2C_ADDR, &cAddress, 1);
  MPU6050.read(MPU6050_I2C_ADDR, (char*)cAccel , 6);


  sAccel->u16XAccel = ((cAccel[0] << 8) | cAccel[1]);
  sAccel->u16YAccel = ((cAccel[2] << 8) | cAccel[3]);
  sAccel->u16ZAccel = ((cAccel[4] << 8) | cAccel[5]);
}

void MPU6050_getGyro(sAxisGyro* sGyro)
{
  char cAddress = MPU6050_REG_GYRO_XOUT_H;
  uint8_t cGyro[6];

  //Read 6 registers from the Register X_GYRO_OUT_H to get the value of 3 axis gyro
  MPU6050.write(MPU6050_I2C_ADDR, &cAddress, 1);
  MPU6050.read(MPU6050_I2C_ADDR, (char*)cGyro , 6);

  sGyro->u16XGyro = ((cGyro[0] << 8) | cGyro[1]);
  sGyro->u16YGyro = ((cGyro[2] << 8) | cGyro[3]);
  sGyro->u16ZGyro = ((cGyro[4] << 8) | cGyro[5]);
}

void Rx_interrupt(void)
{
  //while ((serial.readable()) && (((rx_in + 1) % buffer_size) != rx_out)) {
   rx_buffer[rx_in] = serial.getc();
   if (rx_buffer[rx_in] == '$') {
    isStarted = true;
    countt++;
   } else if (rx_buffer[rx_in] == '\n') {
    isStarted = false;
    newLineCount++;
   }  
   rx_in = (rx_in + 1) % buffer_size;

   if (!isStarted) {
     rx_buffer[rx_in-1] ='\0';
     rx_in = 0;
   }
}

void speed_string_parse(char* buffer, struct motorSpeed_t* motorSpeed)
{
  int j = 0;
  int i = 0;
  char motor1_buff[10];
  char motor2_buff[10];
  char motor3_buff[10];
  char motor4_buff[10];
  int index_of_comma[10];
  
  if(buffer[0] != '$' && buffer[strlen(buffer)] != ',') {
    Debug.printf("Error received string!!\n");
    goto exit;
  }

  for (j = 0; j < strlen(buffer); j++) {
    if (buffer[j] == ',')
      index_of_comma[i++] = j;
  }

  strncpy(motor1_buff, buffer + 1, index_of_comma[0] - 1);
  motor1_buff[index_of_comma[0] - 1] = '\0';
  motorSpeed->motor1SpeedIn = atoi(motor1_buff);

  strncpy(motor2_buff, buffer + index_of_comma[0] + 1, index_of_comma[1] - index_of_comma[0] - 1);
  motor2_buff[index_of_comma[1] - index_of_comma[0] - 1] = '\0';
  motorSpeed->motor2SpeedIn = atoi(motor2_buff);

  strncpy(motor3_buff, buffer + index_of_comma[1] + 1, index_of_comma[2] - index_of_comma[1] - 1);
  motor3_buff[index_of_comma[2] - index_of_comma[1] - 1] = '\0';
  motorSpeed->motor3SpeedIn = atoi(motor3_buff);
  
  strncpy(motor4_buff, buffer + index_of_comma[2] + 1, index_of_comma[3] - index_of_comma[2] - 1);
  motor4_buff[index_of_comma[3] - index_of_comma[2] - 1] = '\0';
  motorSpeed->motor4SpeedIn = atoi(motor4_buff);

  exit: ;
}