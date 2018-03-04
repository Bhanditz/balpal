/*
  Tarve: MMA7455:n X+Z-akselin muutokset heijastuvat moottoriin siten että liike kompensoi asentoa siten että asento pysyy vakiona (=0).
*/
#include <Wire.h>

/*********************************/

#define CMD_ASD   0
#define CMD_START 1
#define CMD_STOP  2
#define CMD_PLUS  3
#define CMD_MINUS 4

/*********************************/

#define MOTOR_V_MAX 5
#define MOTOR_PIN 9

int motorPrs = 40;
int motorVal;
int MotorIn;

/*********************************/

#define INCOMING_BUFFER_LENGTH 5

unsigned int incomingBytesIdx = 0;
int *incomingBytesBuffer = (int*)malloc(INCOMING_BUFFER_LENGTH);

/*********************************/

#include <MMA_7455.h>

#define MMA_7455_gravity_offset 64
#define MMA_7455_sensitivity 2 // Good for "Tilt" measurements
#define MMA_7455_x_offset  2
#define MMA_7455_y_offset  18
#define MMA_7455_z_offset -6

MMA_7455 mma7455; //Make an instance of MMA_7455

struct AccStruct {
  char x;
  char y;
  char z;
  char zg;

  AccStruct *(*toSerial)( AccStruct * );
  AccStruct *(*fromMMA7455)( MMA_7455 * );
} accVal;
AccStruct *__AccStruct_toSerial( AccStruct *self ) {
  Serial.print("X:");
  if ( self->x >= 0 ) Serial.print( " " );
  Serial.print(  self->x, DEC);
  Serial.print(", Y:");
  if ( self->y >= 0 ) Serial.print( " " );
  Serial.print(  self->y, DEC);
  Serial.print(", Z:");
  if ( self->zg >= 0 ) Serial.print( " " );
  Serial.println(self->zg, DEC);
  return self;
}
AccStruct *__AccStruct_toSerialPlot( AccStruct *self ) {
  Serial.print(" ");
  Serial.print(  self->x, DEC);
  Serial.print(" ");
  Serial.print(  self->y, DEC);
  Serial.print(" ");
  Serial.print(self->zg, DEC);
  Serial.print(" ");
  int c = (self->x + self->y + self->z) / 3.0;
  Serial.println(c, DEC);
  return self;
}
AccStruct *__AccStruct_fromMMA7455( AccStruct *self, MMA_7455 *inst ) {
  self->x = inst->readAxis('x');
  self->y = inst->readAxis('y');
  self->z = inst->readAxis('z');
  return self;
}

/*********************************/

bool running = true;

void start() {
//  Serial.println("STARTING");
  running = true;
  digitalWrite(LED_BUILTIN, HIGH);
}
void stop() {
//  Serial.println("STOPPING");
  running = false;
  digitalWrite(LED_BUILTIN, LOW);
  analogWrite(MOTOR_PIN, 0);
}

void setup() {
  pinMode(MOTOR_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(9600);
  while (!Serial);

  //  TCCR1B = TCCR1B & 0b11111000 | 0x02;

  mma7455 = MMA_7455();

  mma7455.initSensitivity( MMA_7455_sensitivity );
  mma7455.calibrateOffset( MMA_7455_x_offset
                           , MMA_7455_y_offset
                           , MMA_7455_z_offset );
  stop();
}

int itr = 0;
void loop() {
  ++itr;
  if (Serial.available() > 0) {
    handleIncomingByte(Serial.read());
  }

  __AccStruct_fromMMA7455( &accVal, &mma7455 );
  accVal.zg = accVal.z - MMA_7455_gravity_offset;

  if ( itr % 200 == 0 ) {
    __AccStruct_toSerialPlot( &accVal );
    char ay  = abs(accVal.y);
    bool neg = accVal.y < 0;
    if ( ay > 10 ) {
//      Serial.println("Tilted");
    }

  }

  if ( running ) {
    analogWrite( MOTOR_PIN, motorVal );

    if ( itr % 10000 == 0 ) {
//      Serial.print(motorPrs);
//      Serial.print("% = ");
//      Serial.println(motorVal);
    }
  }
}

/*********************************/

void setMotorPrs(int prs) {
  if ( prs <   0 ) prs = 0;
  if ( prs > 100 ) prs = 100;
  motorPrs = prs;
  motorVal = ((MOTOR_V_MAX / 5.0) * 255.0) * (motorPrs / 100.0);
//  Serial.print("motorPrs: ");
//  Serial.println( motorPrs );
}

/*********************************/

void handleIncomingByte( int incomingByte ) {
  if (incomingByte == 10) {
    incomingBytesIdx = 0;
    switch (detectCmd()) {
      case CMD_ASD:
//        Serial.println("ASDING AROUND");
        break;
      case CMD_START:
        start();
        break;
      case CMD_STOP:
        stop();
        break;
      case CMD_PLUS:
        setMotorPrs(++motorPrs);
        break;
      case CMD_MINUS:
        setMotorPrs(--motorPrs);
        break;
      default:
//        Serial.println("Unknown command!");
        break;
    }
  } else if (incomingBytesIdx >= INCOMING_BUFFER_LENGTH) {
    incomingBytesIdx = 0;
//    Serial.println("Too long cmd");
  } else {
    incomingBytesBuffer[incomingBytesIdx++] = incomingByte;
  }
}

int   cmdAsd[3] = { 'a', 's', 'd' };
int cmdStart[5] = { 's', 't', 'a', 'r', 't' };
int  cmdStop[4] = { 's', 't', 'o', 'p' };

int  cmdPlus[1] = { '+' };
int cmdMinus[1] = { '-' };

int detectCmd() {
  if (detectCmdParser(cmdAsd,   3)) return CMD_ASD;
  if (detectCmdParser(cmdStart, 5)) return CMD_START;
  if (detectCmdParser(cmdStop,  4)) return CMD_STOP;
  if (detectCmdParser(cmdPlus,  1)) return CMD_PLUS;
  if (detectCmdParser(cmdMinus, 1)) return CMD_MINUS;
  return -1;
}
bool detectCmdParser( int *cmd, int len ) {
  for ( int i = 0; i < len; i++ ) {
    if ( incomingBytesBuffer[i] != cmd[i] ) {
      return false;
    }
  }
  return true;
}

