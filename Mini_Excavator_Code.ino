#include "Adafruit_MCP23X17.h"
#include <ESP32Servo.h>  // by Kevin Harrington
#include <AsyncTimer.h>



#include <Bluepad32.h>

// defines
#define DEBUG true
#define LED_BUILTIN 2
#define BOOT_BUTTON 0

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

//joystick config
const int32_t fullzone = 250;
const int32_t midzone = 100;
const int32_t deadzone = 55;

// setup outputs
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

// Timer setup, see https://github.com/Aasim-A/AsyncTimer
AsyncTimer asynctimer;

unsigned short LEDintervalId;

void setup_LEDinterval() {
  LEDintervalId = asynctimer.setInterval([]() { 
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); 
    }, 500);
}

const unsigned int MAX_GAMEPADS = 1;
// ControllerPtr myControllers[BP32_MAX_GAMEPADS];
ControllerPtr myControllers[MAX_GAMEPADS];

// This callback gets called any time a new gamepad is connected.
// Up to 4 gamepads can be connected at the same time.
void onConnectedController(ControllerPtr ctl) {
    bool foundEmptySlot = false;
    for (int i = 0; i < MAX_GAMEPADS; i++) {
        if (myControllers[i] == nullptr) {
            Serial.printf("CALLBACK: Controller is connected, index=%d\n", i);
            // Additionally, you can get certain gamepad properties like:
            // Model, VID, PID, BTAddr, flags, etc.
            ControllerProperties properties = ctl->getProperties();
            if (DEBUG) {
              Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id,
                           properties.product_id);
            }
            
            myControllers[i] = ctl;
            foundEmptySlot = true;

            // cancel blinking LED and ensure it's on
            asynctimer.cancel(LEDintervalId);
            digitalWrite(LED_BUILTIN, HIGH);
            break;
        }
    }
    if (!foundEmptySlot && DEBUG) {
        Serial.println("CALLBACK: Controller connected, but could not find empty slot");
    }
}

void onDisconnectedController(ControllerPtr ctl) {
    bool foundController = false;

    for (int i = 0; i < MAX_GAMEPADS; i++) {
        if (myControllers[i] == ctl) {
            if (DEBUG) {
              Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
            }
            myControllers[i] = nullptr;
            foundController = true;
            break;
        }
    }

    if (!foundController) {
        Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");
    }

    // start blinking LED again
    setup_LEDinterval();
}

void processGamepad(ControllerPtr ctl) {
    //---------------- Analog stick value events ---------------
    
    const int32_t axisX = ctl->axisX();
    bool axisXMove = false;
    if (axisX > fullzone) {
      axisXMove = true;
      Serial.printf("Moved the left stick X: %4d\n", axisX);
      mcp.digitalWrite(pivot0, HIGH);
      mcp.digitalWrite(pivot1, LOW);
    }
    if (axisX < -fullzone) {
      axisXMove = true;
      mcp.digitalWrite(pivot0, LOW);
      mcp.digitalWrite(pivot1, HIGH);
    }
    if (!axisXMove) {
      mcp.digitalWrite(pivot0, LOW);
      mcp.digitalWrite(pivot1, LOW);
    }
    

    const int32_t axisY = ctl->axisY();
    bool axisYMove = false;
    if (axisY > fullzone) {
      axisYMove = true;
      Serial.printf("Moved the left stick Y: %4d\n", axisY);
      mcp.digitalWrite(secondBoom0, HIGH);
      mcp.digitalWrite(secondBoom1, LOW);
    }
    if (axisY < -fullzone) {
      axisYMove = true;
      mcp.digitalWrite(secondBoom0, LOW);
      mcp.digitalWrite(secondBoom1, HIGH);
    }
    if (!axisYMove) {
      mcp.digitalWrite(secondBoom0, LOW);
      mcp.digitalWrite(secondBoom1, LOW);
    }

    const int32_t axisRX = ctl->axisRX();
    bool axisRXmoved = false;
    if (axisRX > fullzone) {
      axisRXmoved = true;
      mcp.digitalWrite(tiltAttach0, HIGH);
      mcp.digitalWrite(tiltAttach1, LOW);
    }
    if (axisRX < -fullzone) {
      axisRXmoved = true;
      mcp.digitalWrite(tiltAttach0, LOW);
      mcp.digitalWrite(tiltAttach1, HIGH);
    }
    if (!axisRXmoved) {
      // Serial.printf("Dead right stick X: %4d \n", axisRX);
      mcp.digitalWrite(tiltAttach0, LOW);
      mcp.digitalWrite(tiltAttach1, LOW);
    } else {
      // Serial.printf("Moved the right stick X: %4d \n", axisRX);
    }

    const int32_t axisRY = ctl->axisRY();
    bool axisRYmoved = false;
    if (axisRY > fullzone) {
      axisRYmoved = true;
      // Serial.printf("Moved the right stick Y: %4d \n", axisRY);
      mcp.digitalWrite(mainBoom0, HIGH);
      mcp.digitalWrite(mainBoom1, LOW);
    }
    if (axisRY < -fullzone) {
      axisRYmoved = true;
      mcp.digitalWrite(mainBoom0, LOW);
      mcp.digitalWrite(mainBoom1, HIGH);
    }
    if (!axisRYmoved) {
      mcp.digitalWrite(mainBoom0, LOW);
      mcp.digitalWrite(mainBoom1, LOW);
    }


    // Dpad 
    uint8_t dpad = ctl->dpad();

    // dpad down
    bool thumb_pressed = false;
    if (dpad == 2) {
      thumb_pressed = true;
      // Serial.println("Started pressing dpad down");
      mcp.digitalWrite(thumb0, LOW);
      mcp.digitalWrite(thumb1, HIGH);
    } 
    // dpad up
    if (dpad == 1) {
      thumb_pressed = true;
      // Serial.println("Started pressing dpad up");
      mcp.digitalWrite(thumb0, HIGH);
      mcp.digitalWrite(thumb1, LOW);
    } 
    if (!thumb_pressed) {// release dpad up
      mcp.digitalWrite(thumb0, LOW);
      mcp.digitalWrite(thumb1, LOW);
      //Serial.println("Released dpad up");
    }
    
    // dpad right
    bool auxAttach_pressed = false;
    if (dpad == 8) {
      auxAttach_pressed = true;
      mcp.digitalWrite(auxAttach0, HIGH);
      mcp.digitalWrite(auxAttach1, LOW);
      // Serial.println("Started pressing dpad right");
    }
    // dpad left
    if (dpad == 4) {
      auxAttach_pressed = true;
      mcp.digitalWrite(auxAttach0, LOW);
      mcp.digitalWrite(auxAttach1, HIGH);
      // Serial.println("Started pressing the left button");
    } 
    if (!auxAttach_pressed) {
      mcp.digitalWrite(auxAttach0, LOW);
      mcp.digitalWrite(auxAttach1, LOW);
      // Serial.println("Released dpad left");
    }
    
    


    //------------- Digital shoulder/trigger button events -------------
    bool lpressed = false;
    if (ctl->l1()) {
      lpressed = true;
      mcp.digitalWrite(leftMotor0, HIGH);
      mcp.digitalWrite(leftMotor1, LOW);
      // Serial.println("Started pressing the left shoulder button");
    } 
    if (ctl->l2()) {
      lpressed = true;
      mcp.digitalWrite(leftMotor0, LOW);
      mcp.digitalWrite(leftMotor1, HIGH);
      // Serial.println("Started pressing the left trigger button");
    } 
    if (!lpressed) {
      mcp.digitalWrite(leftMotor0, LOW);
      mcp.digitalWrite(leftMotor1, LOW);
      // Serial.println("Released the left  button");
    }
    bool rpressed = false;
    if (ctl->r1()) {
      rpressed = true;
      mcp.digitalWrite(rightMotor0, HIGH);
      mcp.digitalWrite(rightMotor1, LOW);
      // Serial.println("Started pressing the right shoulder button");
    } if (ctl->r2()) {
      rpressed = true;
      mcp.digitalWrite(rightMotor0, LOW);
      mcp.digitalWrite(rightMotor1, HIGH);
      // Serial.println("Started pressing the right trigger button");
    } 
    if (!rpressed) {
      mcp.digitalWrite(rightMotor0, LOW);
      mcp.digitalWrite(rightMotor1, LOW);
      // Serial.println("Released the right button");
    }

    //--- Digital cross/square/triangle/circle or a/x/y/b button events ---
    if (ctl->a()) {
      moveClawServoUp = true;
      // Serial.println("Started pressing the a/cross button");
    } else {
      moveClawServoUp = false;
      // Serial.println("Released the a/cross button");
    }
    if (ctl->x()) {
      moveAuxServoUp = true;
      // Serial.println("Started pressing the x/square button");
    } else {
      moveAuxServoUp = false;
      // Serial.println("Released the x/square button");
    }
    if (ctl->y()) {
      moveClawServoDown = true;
      // Serial.println("Started pressing the y/triangle button");
    } else {
      moveClawServoDown = false;
      // Serial.println("Released the y/triangle button");
    }

    if (ctl->b()) {
      moveAuxServoDown = true;
      // Serial.println("Started pressing the b/circle button");
    } else {
      moveAuxServoDown = false;
      // Serial.println("Released the b/circle button");
    }


    //--------------- Digital stick button events --------------
    if (ctl->thumbL()) {
      if (!cabLightsOn) {
        digitalWrite(cabLights, HIGH);
        cabLightsOn = true;
      } else {
        digitalWrite(cabLights, LOW);
        cabLightsOn = false;
      }
      // Serial.println("Started pressing the left stick button");
    } else {
      // Serial.println("Released the left stick button");
    }
      
    if (ctl->thumbR()) {
      if (!auxLightsOn) {
        digitalWrite(auxLights, HIGH);
        auxLightsOn = true;
      } else {
        digitalWrite(auxLights, LOW);
        auxLightsOn = false;
      }
      // Serial.println("Started pressing the right stick button");
    } else {
      // Serial.println("Released the right stick button");
    }


    // servo control
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

void processControllers() {
    for (auto myController : myControllers) {
        if (myController && myController->isConnected() && myController->hasData()) {
            if (myController->isGamepad()) {
                processGamepad(myController);
            //} else if (myController->isMouse()) {
              //  processMouse(myController);
            //} else if (myController->isKeyboard()) {
              //  processKeyboard(myController);
            //} else if (myController->isBalanceBoard()) {
              //  processBalanceBoard(myController);
            } else {
                Serial.println("Unsupported controller");
            }
        }
    }
}



// Arduino setup function. Runs in CPU 1
void setup() {
    Serial.begin(115200);

    // mini excavator setup

    mcp.begin_I2C();

    for (int i = 0; i <= 15; i++) {
      mcp.pinMode(i, OUTPUT);
    }

    Serial.println("mcp Ready.");

    pinMode(clawServoPin, OUTPUT);
    pinMode(auxServoPin, OUTPUT);

    pinMode(cabLights, OUTPUT);
    pinMode(auxLights, OUTPUT);

    clawServo.attach(clawServoPin);
    auxServo.attach(auxServoPin);
    clawServo.write(clawServoValue);
    auxServo.write(auxServoValue);

    Serial.println("Servos Ready.");
    
    Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
    const uint8_t* addr = BP32.localBdAddress();
    Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

    // Setup the Bluepad32 callbacks
    BP32.setup(&onConnectedController, &onDisconnectedController);

    // "forgetBluetoothKeys()" should be called when the user performs
    // a "device factory reset", or similar.
    // Calling "forgetBluetoothKeys" in setup() just as an example.
    // Forgetting Bluetooth keys prevents "paired" gamepads to reconnect.
    // But it might also fix some connection / re-connection issues.
    // BP32.forgetBluetoothKeys();

    // Enables mouse / touchpad support for gamepads that support them.
    // When enabled, controllers like DualSense and DualShock4 generate two connected devices:
    // - First one: the gamepad
    // - Second one, which is a "virtual device", is a mouse.
    // By default, it is disabled.
    // BP32.enableVirtualDevice(false);

    // blink LED until connected
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
    setup_LEDinterval();
    
}

// Arduino loop function. Runs in CPU 1.
void loop() {
    // This call fetches all the controllers' data.
    // Call this function in your main loop.
    bool dataUpdated = BP32.update();
    if (dataUpdated)
        processControllers();

    // The main loop must have some kind of "yield to lower priority task" event.
    // Otherwise, the watchdog will get triggered.
    // If your main loop doesn't have one, just add a simple `vTaskDelay(1)`.
    // Detailed info here:
    // https://stackoverflow.com/questions/66278271/task-watchdog-got-triggered-the-tasks-did-not-reset-the-watchdog-in-time

    vTaskDelay(1);

    // forget bluetooth keys if boot button is pressed
    if (digitalRead(BOOT_BUTTON)) {
      BP32.forgetBluetoothKeys();
      delay(500);
    }
    // delay(150);

    asynctimer.handle();
}
