#define MAX_CMD_BUF  20
#define CMD_STR 0
#define CMD_DIR 1
#define CMD_GAS 2
#define CMD_TIME 3

void setup() {
  Serial.begin(9600);
}

void loop() {
  if (Serial.available() > 0) {
    doAutoCommands();
  }
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
        Serial.printf("%d, %d\n", cmd_cnt, str);
        break;
      case CMD_DIR:
        dir = atoi(command);
        Serial.printf("%d, %d\n", cmd_cnt, dir );
        break;
      case CMD_GAS:
        gas = atoi(command);
        Serial.printf("%d, %d\n", cmd_cnt, gas);
        break;
      case CMD_TIME:
        time = atoi(command);
        Serial.printf("%d, %lu\n", cmd_cnt, time);
        break;
      default:
        Serial.println("NOOP");
    }
    command = strtok(0, ",");
    cmd_cnt++;
  }
  delay(100);
  Serial.printf("DONE COMMANDS: %lu\n", millis());
}






