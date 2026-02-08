#include <printf.h>
#include <Servo.h>
#include <nRF24L01.h>
#include <SPI.h>
#include <RF24.h>

//radio and servo init
RF24 radio(9, 10);
Servo hServo, vServo;

//radio addresses setup
byte addresses[][6] = {"1Node", "2Node"};
typedef enum {transmitter = 1, receiver} radioMode;
radioMode mode = receiver;
const char* roleRemind[] = {"Not defined", "Transmitter (TX)", "Receiver (RX)"};

//position and message size (in bytes) constants, and laser pin
unsigned const int mSIZE = 35;
const int hSTART = 89;
const int vSTART = 101;
const int LASER_PIN = 6;

//offsets
int hOffset = 0;
int vOffset = 0;
int hPos_tmp = 0;  //used to not duplicate commands too much
int vPos_tmp = 0;

//a load of variables (i have no idea which ones i'm using)
char message[mSIZE] = "0 0"; //default starting position
String currAlgo = "manual"; //default algorithm
int messageUpdate = 0;
int ackIgnoreCount = 2;
bool newInput = false;
bool calculated = false;
bool initDegrees = false;
static char lastCommand[mSIZE] = "";
unsigned long lastCommandTime = 0;
char inputMessageGlobal[mSIZE] = "";

bool scanInProgress = false;
int scanStep = -40;
unsigned long lastScanTime = 0;
int currPhase = 0;

int defaultDelay = 3000; //change if you need servo commands to be done faster or slower (recommended not to change duh)
int delayMulti = 13; //don't change or you might be greeted with command duplication


//radio functions (made to shorten the code):
//abandoned ^


//servo commands (functions)
void startPos() {
    hServo.write(hSTART);
    vServo.write(vSTART);
    hOffset = 0;
    vOffset = 0;
}

void manual_turn(int hPos, int vPos) {
    if (hPos >= 0 && hPos <= 180) {
        hServo.write(hPos);
        Serial.print(". ");
        Serial.print("Current horizontal degrees: ");
        Serial.println(hPos-hSTART);
    }
    if (vPos >= 0 && vPos <= 180) {
        vServo.write(vPos);
        Serial.print("Current vertical degrees: ");
        Serial.println(vPos-vSTART);
        Serial.print('\n');
    }
}

void horizontalScan() {
  if (scanStep > 40) {
    scanStep = -40;
    scanInProgress = false;
    startPos();
    currAlgo = "manual";
    return;
  }
  hOffset = scanStep;
  hServo.write(hSTART + scanStep);
  scanStep += 10;
}

void verticalScan() {
  if (scanStep > 40) {
    scanStep = -40;
    scanInProgress = false;
    startPos();
    currAlgo = "manual";
    return;
  }
  vOffset = scanStep;
  vServo.write(vSTART + scanStep);
  scanStep += 10;
}

void diagonalScanInscrease() {
  if (scanStep > 40) {
    scanStep = -40;
    scanInProgress = false;
    startPos();
    currAlgo = "manual";
    return;
  }
  hOffset = scanStep;
  vOffset = scanStep;
  hServo.write(hSTART + scanStep);
  vServo.write(vSTART + scanStep);
  scanStep += 10;
}

void diagonalScanDecrease() {
  if (scanStep > 40) {
    scanStep = -40;
    scanInProgress = false;
    startPos();
    currAlgo = "manual";
    return;
  }
  hOffset = scanStep;
  vOffset = -scanStep;
  hServo.write(hSTART + scanStep);
  vServo.write(vSTART - scanStep);
  scanStep += 10;
}


//setup
void setup() {
  //general setup
    Serial.begin(9600);
    printf_begin();
    printf("Role: %s\n\r", roleRemind[mode]);

  //radio setup/settings (advised not to change)
    radio.begin();
    radio.setAutoAck(1);
    radio.enableAckPayload();
    radio.enableDynamicPayloads(); //dynamic payloads
    radio.setRetries(15, 15); //do NOT set to (0, 15), i spent 3 hours debugging
  //radio.setPayloadSize(mSIZE); //comment out if not static
    radio.openWritingPipe(addresses[1]);
    radio.openReadingPipe(1, addresses[0]);
    radio.startListening();
    radio.powerUp();
    radio.printDetails();
    
  //servo setup
    hServo.attach(5);
    vServo.attach(4);
  //starting position
    hServo.write(hSTART); //horizontal 89
    vServo.write(vSTART); //vertical 101

  //laser setup (turned on by default)
    pinMode(LASER_PIN, OUTPUT);
    digitalWrite(LASER_PIN, HIGH); 
}


// RX/TX control, input register, command execution
void loop() {
    //input handling
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');
        input.trim();

        if (input.length() == 0) return;

        int inputLen = input.length()+1;
        char inputMessage[inputLen];
        newInput = true;
        ackIgnoreCount = 2; // ignore first 2 ACKs
        input.toCharArray(inputMessage, inputLen);
        strcpy(inputMessageGlobal, inputMessage);

        printf("%s\n\r", inputMessage);

      //switching TX and RX, if not T or R - it's not a radio command
        char ch = toupper(inputMessage[0]);
        char ch2 = toupper(inputMessage[1]);
        if (ch == 'T' and mode == receiver) {
            printf("Switching to transmitter (TX) mode...\n\r");
            mode = transmitter;
            radio.openWritingPipe(addresses[0]);
            radio.openReadingPipe(1, addresses[1]);
        }
        else if (ch == 'R' and mode == transmitter) {
            printf("Switching to receiver (RX) mode...\n\r");
            mode = receiver;
            radio.openWritingPipe(addresses[1]);
            radio.openReadingPipe(1, addresses[0]);
            radio.startListening();
        } else {
            strcpy(message, inputMessage);
      }
    }
    
    unsigned int inputLen = mSIZE; //probably have to delete this variable but uhhh later
    //TX mode
    if (mode == transmitter && newInput) {
        char gotMessage[inputLen] = {0};
        radio.stopListening();
        printf("TX Log: Sending %s as a payload...\n\r", message);
        unsigned long timeSpent = micros();

     //sending messages
        if (radio.write(message, sizeof(message))) {
            if (!radio.available()) {
                printf("TX Log: Connection set up but the ack payload is blank... Delay: %lu microseconds\n\r", micros()-timeSpent);
                ackIgnoreCount--;
            } else {
                while (radio.available()) {
                    radio.read(gotMessage, sizeof(gotMessage));
                    if (ackIgnoreCount > 0) {
                        ackIgnoreCount--;
                        printf("RX Ack: (FALSE STATE DUE TO DELAY) %s\n\r", gotMessage);
                        continue;
                    }
                    printf("RX Ack: Got response (current state): %s, delay: %lu microseconds\n\r", gotMessage, micros()-timeSpent);
            } //calculating expected positions
            if (toupper(inputMessageGlobal[0]) == 'V' && !calculated) {
                verticalScan(); //any value here, it doesnt matter
                calculated = true;
            } else if (toupper(inputMessageGlobal[0]) == 'H' && !calculated) {
                horizontalScan(); //any value
                calculated = true;
            }  else if (toupper(inputMessageGlobal[0]) == 'D' && toupper(inputMessageGlobal[1]) ==  'D' && !calculated) {
                diagonalScanDecrease(); //any value
                calculated = true;
            } else if (toupper(inputMessageGlobal[0]) == 'D' && toupper(inputMessageGlobal[1]) ==  'I' && !calculated) {
                diagonalScanInscrease(); //any value
                calculated = true;
            }
          }
        } else {
          printf("TX Log: Failed to send and ack payload failed lol u suck dude\n\r");
          }
      
        messageUpdate++;
        delay(defaultDelay-1000); //needs to be set up 
        bool isManualCmd = (isDigit(inputMessageGlobal[0]) or isDigit(inputMessageGlobal[1])) or (inputMessageGlobal[0] == 'T');
        bool isALL = (inputMessageGlobal[0] == 'A' && inputMessageGlobal[1] == 'L');

      //all of those has to be set up
        if (messageUpdate == 3 && isManualCmd && !isALL) { //message amount for manual cmds
            newInput = false;
            calculated = false;
            messageUpdate = 0;
        } else if (messageUpdate == 16 && !isManualCmd && !isALL) { //message amount for automatic algorithms
            newInput = false;
            calculated = false;
            messageUpdate = 0;
        } else if (messageUpdate == 41 && !isManualCmd && isALL) { //need set up
            newInput = false;
            calculated = false;
            messageUpdate = 0;
        }
    }

    //RX mode
    if (mode == receiver) {  //Problem here: broken (working badly) ACK payload (RX's response to TX)
      //command delay
        if (lastCommand[0] != '\0' && millis() - lastCommandTime >= delayMulti*defaultDelay) {
            lastCommand[0] = '\0';
            currAlgo = "manual";
        }
        delayMulti = 13; //the variable of chaos and torture

      //rx mode (reading messages and sending ACK payloads (answers))
        char gotMessage[sizeof(message)] = {0};
        String ansStr = "cubesat " + String(hOffset) + " " + String(vOffset) + " " + currAlgo;
        char ans[mSIZE] = {0};
        byte pipeNo;
        ansStr.toCharArray(ans, mSIZE);
        while (radio.available(&pipeNo)) {
            radio.read(gotMessage, sizeof(message));
            radio.writeAckPayload(pipeNo, ans, sizeof(ans)); //can send empty payloads by commenting this
            printf("RX Log: Message read, sending back current state: %s\n\r", ans);
        }
      
      //deciding what TX message is and doing actions accordingly
        if (gotMessage[0] != '\0' && toupper(gotMessage[0]) != 'T' && toupper(gotMessage[0]) != 'R' && strcmp(gotMessage, lastCommand) != 0) {
            strcpy(lastCommand, gotMessage);
            String servoIns = String(gotMessage);
            lastCommandTime = millis();
            char ch1 = toupper(servoIns[0]);
            char ch2 = toupper(servoIns[1]);
            if (ch1 == 'H' or ch1 == 'V' or (ch1 == 'D' && (ch2 == 'I' or ch2 == 'D'))) {
                scanStep = -40;
                hOffset = 0;
                vOffset = 0;
            }
            if (isDigit(servoIns[0]) or isDigit(servoIns[1])) {
                hOffset = servoIns.substring(0, servoIns.indexOf(' ')).toInt();
                vOffset = servoIns.substring(servoIns.indexOf(' ')+1).toInt();
                if (hOffset != hPos_tmp or vOffset != vPos_tmp or !initDegrees) {
                    manual_turn(hSTART+hOffset, vSTART+vOffset);
                    hPos_tmp = hOffset;
                    vPos_tmp = vOffset;
                    initDegrees = true;
                }
          }

            scanStep = -40;
            if (ch1 == 'H') {
                currAlgo = "horizont";
                scanInProgress = true;
            }
            else if (ch1 == 'V') {
                currAlgo = "vertical";
                scanInProgress = true;
            }
            else if (ch1 == 'D' && ch2 == 'I') {
                currAlgo = "DI";
                scanInProgress = true;
            }
            else if (ch1 == 'D' && ch2 == 'D') {
                currAlgo = "DD";
                scanInProgress = true;
            } else if (ch1 == 'A' && ch2 == 'L') {
                currAlgo = "ALL";
                scanInProgress = true;
                currPhase = 0;
                scanStep = -40;
                delayMulti = 60;    //need setup
            }
            else if (ch1 == 'O' && ch2 == 'N') {
                digitalWrite(LASER_PIN, HIGH);
                printf("The laser: ON\n\r");
            }
            else if (ch1 == 'O' && ch2 == 'F') {
                digitalWrite(LASER_PIN, LOW);
                printf("The laser: OFF\n\r");
            }
       }
    }

    if (scanInProgress && millis() - lastScanTime >= defaultDelay) {
        lastScanTime = millis();

        if (currAlgo == "horizont") horizontalScan();
        else if (currAlgo == "vertical") verticalScan();
        else if (currAlgo == "DI") diagonalScanInscrease();
        else if (currAlgo == "DD") diagonalScanDecrease();
        else if (currAlgo == "ALL") {
            startPos();
            if (currPhase == 0) {
                horizontalScan();
                if (!scanInProgress) {
                    scanInProgress = true;
                    scanStep = -40;
                    currPhase = 1;
                    currAlgo = "ALL";
                }
            }
            else if (currPhase == 1) {
                verticalScan();
                if (!scanInProgress) {
                    scanInProgress = true;
                    scanStep = -40;
                    currPhase = 2;
                    currAlgo = "ALL";
                }
            }
            else if (currPhase == 2) {
                diagonalScanInscrease();
                if (!scanInProgress) {
                    scanInProgress = true;
                    scanStep = -40;
                    currPhase = 3;
                    currAlgo = "ALL";
                }
            }
            else if (currPhase == 3) {
                diagonalScanDecrease();
                if (!scanInProgress) {
                    currAlgo = "manual";
                    currPhase = 0;
                }
            }
        }
    }
}