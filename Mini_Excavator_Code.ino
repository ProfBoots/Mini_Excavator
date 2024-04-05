#include "Adafruit_MCP23X17.h"
#include <Ps3Controller.h>
#include <ESP32Servo.h>  // by Kevin Harrington

// defines
#define clawServoPin 5
#define auxServoPin 18
#define cabLights 32
#define auxLights 33

#define pivot0 15
#define pivot1 14
#define mainBoom0 9
#define mainBoom1 8
#define secondBoom0 0
#define secondBoom1 1
#define tiltAttach0 3
#define tiltAttach1 2
#define thumb0 11
#define thumb1 10
#define auxAttach0 12
#define auxAttach1 13

#define leftMotor0 7
#define leftMotor1 6
#define rightMotor0 4
#define rightMotor1 5


Adafruit_MCP23X17 mcp;
Servo clawServo;
Servo auxServo;
int dly = 250;
int clawServoValue = 90;
int auxServoValue = 90;
int player = 0;
int battery = 0;
int servoDelay = 0;

bool cabLightsOn = false;
bool auxLightsOn = false;
bool moveClawServoUp = false;
bool moveClawServoDown = false;
bool moveAuxServoUp = false;
bool moveAuxServoDown = false;

void notify() {
  //--------------- Digital D-pad button events --------------
  if (Ps3.event.button_down.up) {

    mcp.digitalWrite(thumb0, HIGH);
    mcp.digitalWrite(thumb1, LOW);
    Serial.println("Started pressing the up button");
  }
  if (Ps3.event.button_up.up) {

    mcp.digitalWrite(thumb0, LOW);
    mcp.digitalWrite(thumb1, LOW);
    Serial.println("Released the up button");
  }
  if (Ps3.event.button_down.down) {
    Serial.println("Started pressing the down button");
    mcp.digitalWrite(thumb0, LOW);
    mcp.digitalWrite(thumb1, HIGH);
  }
  if (Ps3.event.button_up.down) {
    Serial.println("Released the down button");
    mcp.digitalWrite(thumb0, LOW);
    mcp.digitalWrite(thumb1, LOW);
  }
  if (Ps3.event.button_down.right) {
    mcp.digitalWrite(auxAttach0, HIGH);
    mcp.digitalWrite(auxAttach1, LOW);
    Serial.println("Started pressing the right button");
  }
  if (Ps3.event.button_up.right) {
    mcp.digitalWrite(auxAttach0, LOW);
    mcp.digitalWrite(auxAttach1, LOW);
    Serial.println("Released the right button");
  }
  if (Ps3.event.button_down.left) {
    mcp.digitalWrite(auxAttach0, LOW);
    mcp.digitalWrite(auxAttach1, HIGH);
    Serial.println("Started pressing the left button");
  }
  if (Ps3.event.button_up.left) {
    mcp.digitalWrite(auxAttach0, LOW);
    mcp.digitalWrite(auxAttach1, LOW);
    Serial.println("Released the left button");
  }
  //---------------- Analog stick value events ---------------
  if (abs(Ps3.event.analog_changed.stick.lx) + abs(Ps3.event.analog_changed.stick.ly) > 2) {
    Serial.print("Moved the left stick:");
    Serial.print(" x=");
    Serial.print(Ps3.data.analog.stick.lx, DEC);
    Serial.print(" y=");
    Serial.print(Ps3.data.analog.stick.ly, DEC);
    Serial.println();
    int LXValue = Ps3.data.analog.stick.lx;
    Serial.print("LXValue =");
    Serial.print(LXValue);
    if (LXValue > 115) {
      mcp.digitalWrite(pivot0, HIGH);
      mcp.digitalWrite(pivot1, LOW);
      delay(10);
      Serial.print("Made to into Positive");
    }
    if (LXValue < -115) {
      mcp.digitalWrite(pivot0, LOW);
      mcp.digitalWrite(pivot1, HIGH);
      delay(10);
      Serial.print("Made to into negative");
    }

    int LYValue = Ps3.data.analog.stick.ly;
    if (LYValue > 115) {
      mcp.digitalWrite(secondBoom0, HIGH);
      mcp.digitalWrite(secondBoom1, LOW);
      delay(10);
    }
    if (LYValue < -115) {
      mcp.digitalWrite(secondBoom0, LOW);
      mcp.digitalWrite(secondBoom1, HIGH);
      delay(10);
    }
    if (LXValue > -30 && LXValue < 30) {
      mcp.digitalWrite(pivot0, LOW);
      mcp.digitalWrite(pivot1, LOW);
    }
    if (LYValue > -30 && LYValue < 30) {
      mcp.digitalWrite(secondBoom0, LOW);
      mcp.digitalWrite(secondBoom1, LOW);
    }
  }

  if (abs(Ps3.event.analog_changed.stick.rx) + abs(Ps3.event.analog_changed.stick.ry) > 2) {
    Serial.print("Moved the right stick:");
    Serial.print(" x=");
    Serial.print(Ps3.data.analog.stick.rx, DEC);
    Serial.print(" y=");
    Serial.print(Ps3.data.analog.stick.ry, DEC);
    Serial.println();
    int RXValue = (Ps3.data.analog.stick.rx);
    if (RXValue > 115) {
      mcp.digitalWrite(tiltAttach0, HIGH);
      mcp.digitalWrite(tiltAttach1, LOW);
      delay(10);
      Serial.print("Made to into Positive");
    }
    if (RXValue < -115) {
      mcp.digitalWrite(tiltAttach0, LOW);
      mcp.digitalWrite(tiltAttach1, HIGH);
      delay(10);
      Serial.print("Made to into negative");
    }

    int RYValue = (Ps3.data.analog.stick.ry);
    if (RYValue > 115) {
      mcp.digitalWrite(mainBoom0, HIGH);
      mcp.digitalWrite(mainBoom1, LOW);
      delay(10);
    }
    if (RYValue < -115) {
      mcp.digitalWrite(mainBoom0, LOW);
      mcp.digitalWrite(mainBoom1, HIGH);
      delay(10);
    }
    if (RXValue > -30 && RXValue < 30) {
      mcp.digitalWrite(tiltAttach0, LOW);
      mcp.digitalWrite(tiltAttach1, LOW);
    }
    if (RYValue > -30 && RYValue < 30) {
      mcp.digitalWrite(mainBoom0, LOW);
      mcp.digitalWrite(mainBoom1, LOW);
    }
  }
  //------------- Digital shoulder button events -------------
  if (Ps3.event.button_down.l1) {
    mcp.digitalWrite(leftMotor0, HIGH);
    mcp.digitalWrite(leftMotor1, LOW);
    delay(10);
    Serial.println("Started pressing the left shoulder button");
  }
  if (Ps3.event.button_up.l1) {
    mcp.digitalWrite(leftMotor0, LOW);
    mcp.digitalWrite(leftMotor1, LOW);
    delay(10);
    Serial.println("Released the left shoulder button");
  }
  if (Ps3.event.button_down.r1) {
    mcp.digitalWrite(rightMotor0, HIGH);
    mcp.digitalWrite(rightMotor1, LOW);
    delay(10);
    Serial.println("Started pressing the right shoulder button");
  }
  if (Ps3.event.button_up.r1) {
    mcp.digitalWrite(rightMotor0, LOW);
    mcp.digitalWrite(rightMotor1, LOW);
    delay(10);
    Serial.println("Released the right shoulder button");
  }
  //-------------- Digital trigger button events -------------
  if (Ps3.event.button_down.l2) {
    mcp.digitalWrite(leftMotor0, LOW);
    mcp.digitalWrite(leftMotor1, HIGH);
    delay(10);
    Serial.println("Started pressing the left trigger button");
  }
  if (Ps3.event.button_up.l2) {
    mcp.digitalWrite(leftMotor0, LOW);
    mcp.digitalWrite(leftMotor1, LOW);
    delay(10);
    Serial.println("Released the left trigger button");
  }
  if (Ps3.event.button_down.r2) {
    mcp.digitalWrite(rightMotor0, LOW);
    mcp.digitalWrite(rightMotor1, HIGH);
    delay(10);
    Serial.println("Started pressing the right trigger button");
  }
  if (Ps3.event.button_up.r2) {
    mcp.digitalWrite(rightMotor0, LOW);
    mcp.digitalWrite(rightMotor1, LOW);
    delay(10);
    Serial.println("Released the right trigger button");
  }
  //--- Digital cross/square/triangle/circle button events ---
  if (Ps3.event.button_down.cross) {
    moveClawServoUp = true;
    Serial.println("Started pressing the cross button");
  }
  if (Ps3.event.button_up.cross) {
    moveClawServoUp = false;
    Serial.println("Released the cross button");
  }
  if (Ps3.event.button_down.square) {
    moveAuxServoUp = true;
    Serial.println("Started pressing the square button");
  }
  if (Ps3.event.button_up.square) {
    moveAuxServoUp = false;
    Serial.println("Released the square button");
  }
  if (Ps3.event.button_down.triangle) {
    moveClawServoDown = true;
    Serial.println("Started pressing the triangle button");
  }
  if (Ps3.event.button_up.triangle) {
    moveClawServoDown = false;
  }

  if (Ps3.event.button_down.circle) {
    moveAuxServoDown = true;
    Serial.println("Started pressing the circle button");
  }
  if (Ps3.event.button_up.circle) {
    moveAuxServoDown = false;
    Serial.println("Released the circle button");
  }
  //--------------- Digital stick button events --------------
  if (Ps3.event.button_down.l3) {
    if (!cabLightsOn) {
      digitalWrite(cabLights, HIGH);
      cabLightsOn = true;
    } else {
      digitalWrite(cabLights, LOW);
      cabLightsOn = false;
    }
    Serial.println("Started pressing the left stick button");
  }
  if (Ps3.event.button_up.l3)
    Serial.println("Released the left stick button");

  if (Ps3.event.button_down.r3) {
    if (!auxLightsOn) {
      digitalWrite(auxLights, HIGH);
      auxLightsOn = true;
    } else {
      digitalWrite(auxLights, LOW);
      auxLightsOn = false;
    }
    Serial.println("Started pressing the right stick button");
  }
  if (Ps3.event.button_up.r3) {
    Serial.println("Released the right stick button");
  }
  if (moveClawServoUp) {
    if (servoDelay == 3) {
      if (clawServoValue >= 10 && clawServoValue < 170) {
        clawServoValue = clawServoValue + 1;
        clawServo.write(clawServoValue);
      }
      servoDelay = 0;
    }
    servoDelay++;
  }
  if (moveClawServoDown) {
    if (servoDelay == 3) {
      if (clawServoValue <= 170 && clawServoValue > 10) {
        clawServoValue = clawServoValue - 1;
        clawServo.write(clawServoValue);
      }
      servoDelay = 0;
    }
    servoDelay++;
  }
  if (moveAuxServoUp) {
    if (servoDelay == 3) {
      if (auxServoValue >= 10 && auxServoValue < 170) {
        auxServoValue = auxServoValue + 1;
        auxServo.write(auxServoValue);
      }
      servoDelay = 0;
    }
    servoDelay++;
  }
  if (moveAuxServoDown) {
    if (servoDelay == 3) {
      if (auxServoValue <= 170 && auxServoValue > 10) {
        auxServoValue = auxServoValue - 1;
        auxServo.write(auxServoValue);
      }
      servoDelay = 0;
    }
    servoDelay++;
  }
}

void onConnect() {
  Serial.println("Connected.");
}

void setup() {

  Serial.begin(115200);

  mcp.begin_I2C();
  //   put your setup code here, to run once:


  for (int i = 0; i <= 15; i++) {
    mcp.pinMode(i, OUTPUT);
  }

  Ps3.attach(notify);
  Ps3.attachOnConnect(onConnect);
  Ps3.begin("a0:5a:5a:a0:0f:91");

  Serial.println("Ready.");

  pinMode(clawServoPin, OUTPUT);
  pinMode(auxServoPin, OUTPUT);

  pinMode(cabLights, OUTPUT);
  pinMode(auxLights, OUTPUT);

  clawServo.attach(clawServoPin);
  auxServo.attach(auxServoPin);
  clawServo.write(clawServoValue);
  auxServo.write(auxServoValue);
}



void loop() {
  if (!Ps3.isConnected())
    return;
  delay(500);
}
