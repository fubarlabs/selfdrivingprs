#define MAX_CMD_BUF  40

void setup() {
  Serial.begin(9600);
}

void loop() {
  doAutoCommands();
}

void doAutoCommands() {
  /*
     read entire input max length
     while more serial
     fill the comdBuf until a ','
     cmds
     1.
     2.
     3.
     4.
     5.
     When new line end
     execute the commands
  */

  //http://arduino.stackexchange.com/questions/1013/how-do-i-split-an-incoming-string
  char cmdBuf [MAX_CMD_BUF + 1];
  byte size = Serial.readBytes(cmdBuf, MAX_CMD_BUF);
  cmdBuf[size] = 0;
  char* command = strtok(cmdBuf, ",");
  while (command != 0) {
    int cmdnum = atoi(command);
    Serial.printf("%d\n", cmdnum);
    if (cmdnum == 123) {
      Serial.println("nnnnn");
    }
    command = strtok(NULL, ",");
  }
  Serial.println("no command");
}





