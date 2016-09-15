/*
    OTTO REmote Drive:
    Drive the oTTO Tractor with a Remote RC Controller
   by Rick Anderson (ricklon)

*/
#include <Arduino.h>
#include <SoftPWMServo.h>
#include "MPU9250.h"

#define DEBUG_SERIAL true
#define MAX_CMD_BUF  40

//Setup RC Controller
const int channels = 4;

/*
   What are the channels:
   0: thr: throttle
   1: str, steering
   2: aux1: pos1: OK, pos:2 Emergency Stop
   3: aux2:
*/

int ch1; // Here's where we'll keep our channel values
int ch2;
int ch3;
int ch4;

int ch[4];
int chmin[4];
int chmid[4];
int chmax[4];


/*
 * Automatic Set Up
 */

#define MAX_CMD_BUF  20
#define CMD_STR 0
#define CMD_DIR 1
#define CMD_GAS 2
#define CMD_TIME 3

unsigned long last_time;



/*
 * 
 * 
   RC Controller states
    out of range or off
    kill
    enable
*/
#define ACTION 0

//Setup Motor Controller
const int PIN_M1_DIR = 14; 
const int PIN_M2_DIR = 3;
const int PIN_M1_PWM = 4;
const int PIN_M2_PWM = 7;
const int PIN_KILL = 23;

//Setup Steering Control
const int PIN_STR = 10;

//shoot through delay
int PREV_DIR = LOW;
const int SHOOT_DELAY = 250;

//imu constants

//imu unit object
MPU9250 ottoIMU;

//Autonomous Mode info
boolean AUTOMODE = false;


int initIMU() {
  // Read the WHO_AM_I register, this is a good test of communication
  byte c = ottoIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  if (c == 0x71) // WHO_AM_I should always be 0x68
  {
    Serial.println("MPU9250 is online...");
    ottoIMU.MPU9250SelfTest(ottoIMU.SelfTest);
    // Calibrate gyro and accelerometers, load biases in bias registers
    ottoIMU.calibrateMPU9250(ottoIMU.gyroBias, ottoIMU.accelBias);
    ottoIMU.initMPU9250();
    // Initialize device for active mode read of acclerometer, gyroscope, and
    // temperature
    Serial.println("MPU9250 initialized for active data mode....");
    // Read the WHO_AM_I register of the magnetometer, this is a good test of
    // communication
    byte d = ottoIMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
    Serial.print("AK8963 "); Serial.print("I AM "); Serial.print(d, HEX);
    Serial.print(" I should be "); Serial.println(0x48, HEX);

    // Get magnetometer calibration from AK8963 ROM
    ottoIMU.initAK8963(ottoIMU.magCalibration);
    // Initialize device for active mode read of magnetometer
    Serial.println("AK8963 initialized for active data mode....");

  }
  else
  {
    Serial.print("Could not connect to MPU9250: 0x");
    Serial.println(c, HEX);
    //while (1) ; // Loop forever if communication doesn't happen
    return false;
  }
  return true;
}

void setThrottle(int ch_data) {
  int thr;
  int DIR;
  thr = ch_data;
  //map the channel data to throttle data

  if (ch_data > 1115 && ch_data < 1520) { //reverse
    thr = map(ch_data, 1115, 1520, 255, 0 );
    DIR = LOW;
  } else if (ch_data > 1550 && ch_data < 1940) { //forward
    thr = map(ch_data, 1551, 1940, 0, 255  );
    DIR = HIGH;
  }
  else {
    thr = 0; //stop
    DIR = LOW;
  }

  //shoot through protection
  if ( DIR != PREV_DIR) {
    delay(SHOOT_DELAY);
  }
  PREV_DIR = DIR;

  digitalWrite(PIN_M1_DIR, DIR);
  digitalWrite(PIN_M2_DIR, DIR);
  SoftPWMServoPWMWrite(PIN_M1_PWM, thr); //these aren't servos use pwm
  SoftPWMServoPWMWrite(PIN_M2_PWM, thr);//these aren't servos use pwm
  Serial.printf("thr: ch: %d, dir: %d, pwm: %d\n ", ch_data, DIR, thr);
  delay(25);
}
/*
   printIMU to serial port
*/
void printIMU()
{
  ottoIMU.readAccelData(ottoIMU.accelCount);  // Read the x/y/z adc values
  ottoIMU.getAres();
  ottoIMU.ax = (float)ottoIMU.accelCount[0] * ottoIMU.aRes; // - accelBias[0];
  ottoIMU.ay = (float)ottoIMU.accelCount[1] * ottoIMU.aRes; // - accelBias[1];
  ottoIMU.az = (float)ottoIMU.accelCount[2] * ottoIMU.aRes; // - accelBias[2];
  ottoIMU.readGyroData(ottoIMU.gyroCount);  // Read the x/y/z adc values
  ottoIMU.getGres();
  ottoIMU.gx = (float)ottoIMU.gyroCount[0] * ottoIMU.gRes;
  ottoIMU.gy = (float)ottoIMU.gyroCount[1] * ottoIMU.gRes;
  ottoIMU.gz = (float)ottoIMU.gyroCount[2] * ottoIMU.gRes;

  Serial.printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%lu\n", ottoIMU.ax, ottoIMU.ay,  ottoIMU.az, ottoIMU.yaw, ottoIMU.pitch, ottoIMU.roll, millis() );
  ottoIMU.count = millis();
  ottoIMU.sumCount = 0;
  ottoIMU.sum = 0;

  delay(10);

}

void setSteering(int ch_data) {
  int pos;
  //map the channel data to steering data
  /*
     ch[1]_str:
     left: 1120
     center: 1523
     right: 1933

     car_str:
     left: 1300: 1400
     center:  1500
     right: 1582: 1633
  */

  if (ch_data > 1100 && ch_data < 1500) { //right
    pos = map(ch_data, 1100, 1500, 1589, 1525 );
  } else if (ch_data > 1550 && ch_data < 1937) { //left
    pos = map(ch_data, 1550, 1935, 1550, 1400  );
  }
  else {
    pos = 1500; //straight range: 1500 - 1550 ch_str
  }


  SoftPWMServoServoWrite(PIN_STR, pos);
  Serial.printf("str: ch: %d servo: %d\n ", ch_data, pos);
  delay(25);
}




void doAction() {
  ch[0] = pulseIn(A1, HIGH, 25000); // Read the pulse width of
  ch[1] = pulseIn(A2, HIGH, 25000); // each channel
  ch[2] = pulseIn(A3, HIGH, 25000);
  ch[3] = pulseIn(A4, HIGH, 25000);
  //check for kill switch
  if (ch[2] > 1500 ) {
    digitalWrite(PIN_KILL, HIGH);
    Serial.println("KILL HIGH");
  }
  else {
    digitalWrite(PIN_KILL, LOW);
  }//end kill switch

  //check if auto on
  if (ch[3] > 1500 ) {
      //auto mode is on
      //Check if command waiting
      if (Serial.available() > 0) {
        doAutoCommands();
      }
      return;
    }
    else if (ch[0] == 0 ) //check for RCCommands
    {
      Serial.printf("Out of Range or Powered Off\n");
      //set brake
      //kill the machine
    }
    else
    {
      /*
         steering 1100 - 1500 map left
         steering between 1500 - 1600 straight
          steering 1600 - 1900 map left
      */
      setSteering(ch[0]);
      /*
         Throttling:
          1100 - 1500: reverse
          1500 - 1600: no throttle
          1600 - 1900: forward
      */
      setThrottle(ch[1]);
    }
}



void autoSteer(int str) //0 - 255
{
  int pos;
  if (str < 124 ) //steer right
  {
    pos = map(str, 127, 0, 1525, 1589);
  }
  else if (str > 130) //steer left
  {
    pos = map(str, 130, 255, 1550, 1400);
  }
  else
  {
    pos = 1500;
  }


  // SoftPWMServoServoWrite(PIN_STR, pos);
  Serial.printf("str: ch: %d servo: %d\n ", str, pos);
  delay(25);
}

void autoThrottle(int DIR, int thr) {

  //shoot through protection
  if ( DIR != PREV_DIR) {
    delay(SHOOT_DELAY);
  }
  PREV_DIR = DIR;

  digitalWrite(PIN_M1_DIR, DIR);
  digitalWrite(PIN_M2_DIR, DIR);
  SoftPWMServoPWMWrite(PIN_M1_PWM, thr); //these aren't servos use pwm
  SoftPWMServoPWMWrite(PIN_M2_PWM, thr);//these aren't servos use pwm
  Serial.printf("thr: dir: %d, pwm: %d\n ", DIR, thr);
  delay(25);
}



void doAutoCommands() {
  /*
     read entire input max length
     while more serial
     fill the comdBuf until a ','
     cmds
     Output:
     `accel_x, accel_y, accel_z, yaw, pitch, roll,time`.
     Input Commands:
     `steering 0-255, direction 0/1, gas amount 0-255, timestamp`
     1. steer 0-255 maps to
     2. direction
     3. gas
     4. time stamp
     When new line end
     execute the commands

     255,1,255,1110000
     255,0,0,1110001
  */

  //http://arduino.stackexchange.com/questions/1013/how-do-i-split-an-incoming-string
  int cmd_cnt = 0;
  char cmdBuf [MAX_CMD_BUF + 1];

  int str;
  int dir;
  int gas;
  unsigned int time;



  byte size = Serial.readBytes(cmdBuf, MAX_CMD_BUF);
  cmdBuf[size] = 0;
  char* command = strtok(cmdBuf, ",");
  while (command != 0) {
    //int cmdnum = atoi(command);
    //Serial.printf("%d, %s\n", cmd_cnt, command);
    switch (cmd_cnt) {
      case CMD_STR:
        str = atoi(command);
        if (str > 255 || str < 0) {
          return;
        }
        if (DEBUG_SERIAL) {
          Serial.printf("%d, %d\n", cmd_cnt, str);
        }
        break;
      case CMD_DIR:
        dir = atoi(command);
        if (dir  < 0  || dir > 1) {
          return;
        }
        if (DEBUG_SERIAL) {
          Serial.printf("%d, %d\n", cmd_cnt, dir );
        }
        break;
      case CMD_GAS:
        gas = atoi(command);
        if (gas > 255 || gas < 0) {
          return;
        }
        if (DEBUG_SERIAL) {
          Serial.printf("%d, %d\n", cmd_cnt, gas);
        }
        break;
      case CMD_TIME:
        time = atoi(command);
        if (time < last_time) {
          return;
        }
        last_time = time;
        if (DEBUG_SERIAL) {
          Serial.printf("%d, %lu\n", cmd_cnt, time);
        }
        break;
      default:
        if (DEBUG_SERIAL) {
          Serial.println("NOOP");
        }
        return; //return if there are too many commands or non matching
    }
    command = strtok(0, ",");
    cmd_cnt++;

    if (cmd_cnt == 4) {
      if (DEBUG_SERIAL) {
        Serial.printf("str: %d, dir: %d, gas: %d, time: %lu\n", str, dir, gas, time);
      }
      //do commands
      autoSteer(str);
      autoThrottle(dir, gas);
    }
  }

  delay(100);
  if (DEBUG_SERIAL) {
    Serial.printf("DONE COMMANDS: %lu\n", millis());
  }
}


void setup() {
  Serial.begin(9600);
  pinMode(PIN_M1_DIR, OUTPUT); // Motor1 DIR PIN
  pinMode(PIN_M2_DIR, OUTPUT); //Motor2 DIR PIN
  pinMode(PIN_M1_PWM, OUTPUT); // Motor1 PWM PIN
  pinMode(PIN_M2_PWM, OUTPUT); //Motor2 PWM PIN
  pinMode(PIN_KILL, OUTPUT);
  digitalWrite(PIN_KILL, LOW);
  digitalWrite(PIN_M1_DIR, LOW);
  digitalWrite(PIN_M2_DIR, LOW);
  SoftPWMServoServoWrite(PIN_M1_PWM, 0);
  SoftPWMServoServoWrite(PIN_M2_PWM, 0);

  /*
     initIMU: if not reachable stop
  */
  if (!initIMU()) {
    Serial.print("Could not connect to MPU9250: 0x");
    while (1);
  }
  /*
     Initialize the RC Controller data
  */
  getRCInfo();

}

void loop() {
  doAction();
  printIMU();
}


int getRCAction() {
  ch[0] = pulseIn(A1, HIGH, 25000); // Read the pulse width of
  ch[1] = pulseIn(A2, HIGH, 25000); // each channel
  ch[2] = pulseIn(A3, HIGH, 25000);
  ch[3] = pulseIn(A4, HIGH, 25000);
  return ACTION;
}


//Function: get The RC Control infomration
void getRCInfo() {
  ch[0] = pulseIn(A1, HIGH, 25000); // Read the pulse width of
  ch[1] = pulseIn(A2, HIGH, 25000); // each channel
  ch[2] = pulseIn(A3, HIGH, 25000);
  ch[3] = pulseIn(A4, HIGH, 25000);
  //check for out of range or controller off
  if (ch[0] == 0 )
  {
    Serial.printf("Out of Range or Powered Off\n");
  }
  else
  {
    for (int ii = 0; ii < channels; ii++)
    {
      if (ch[ii] > chmax[ii]) {
        chmax[ii] = ch[ii];
      }
      if (ch[ii] < chmin[ii]) {
        chmin[ii] = ch[ii];
      }
      chmid[ii] = (chmin[ii] + chmax[ii]) / 2;
      Serial.printf("Channel %d: %d, min: %d, mid: %d, max: %d\n", ii, ch[ii], chmin[ii], chmid[ii], chmax[ii]); // Print the value of
    }
  }
}
