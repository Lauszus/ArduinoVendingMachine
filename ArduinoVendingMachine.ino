/*
 * The code is released under the GNU General Public License.
 * Developed by Kristian Lauszus, TKJ Electronics 2013
 * This code is used for an old vending machine where the main board did not work anymore
 */

#include "ArduinoVendingMachine.h"

// Change price of the items here:
const uint8_t priceArray[] = { 5, 5, 5, 5, 5, 5 };
// Change the name of the item here:
const uint8_t* nameArray[] = { FANTA, FANTA, COLA, COLA, FAXE, FAXE }; // See in ArduinoVendingMachine.h for the possible names. If the one you need is not present then type NULL instead


// Do not change anything else below this line!
const uint8_t coinPin = 2;  // Analog input pin that the coin validator is attached to
uint8_t lastButtonPressed;
uint32_t purchaseTimer;
bool waitAfterButtonPress;

const uint8_t clockPinLED = A1, dataPinLED = A2, latchPinLED = A0, resetPinLED = 13;
const uint8_t numbers[] = { 0xC0, 0xF9, 0xA4, 0xB0, 0x99, 0x92, 0x82, 0xF8, 0x80, 0x90 }; // Numbers for LED matrix

uint8_t displayBuffer[5];
uint8_t* pOutputString;
bool displayScrolling;
uint8_t scrollPosition, trailingSpaces;
uint32_t scrollTimer;

bool lastCoinInput;
volatile uint16_t counter;
uint16_t lastCounter;

const uint8_t clockPinOut = 3, dataPinOut = 5, latchPinOut = 4, resetPinOut = 6; // Pins for driving the motors
const uint8_t clockPinIn = 11, dataPinIn = 9, latchPinIn = 7; // Pins used to check the switches

const uint8_t motorToOutputMask[] = { 0x04, 0x08, 0x10, 0x20, 0x40, 0x80 };
const uint8_t motorToInputMask[] = { 0x02, 0x04, 0x08, 0x10, 0x20, 0x40 };
const uint8_t errorLedMask = 0x02; //, greenLedMask = 0x01; // The green LED does not work at the moment

uint8_t motorOutput, ledOutput, oldMotorOutput, oldLedOutput;
uint32_t motorTimer;

void setup() {
  Serial.begin(115200);

  // Setup coin input
  pinMode(coinPin, INPUT);
  counter = lastCounter = 0;
  attachInterrupt(0, cointInterrupt, CHANGE);

  // Pins for LED matrix
  pinMode(clockPinLED, OUTPUT);
  pinMode(latchPinLED, OUTPUT);
  pinMode(resetPinLED, OUTPUT);
  pinMode(dataPinLED, OUTPUT);

  digitalWrite(clockPinLED, LOW);
  digitalWrite(latchPinLED, HIGH);
  digitalWrite(resetPinLED, HIGH);

  // Pins used for shiftOut
  pinMode(clockPinOut, OUTPUT);
  pinMode(latchPinOut, OUTPUT);
  pinMode(resetPinOut, OUTPUT);
  pinMode(dataPinOut, OUTPUT);

  digitalWrite(clockPinOut, LOW);
  digitalWrite(latchPinOut, HIGH);
  digitalWrite(resetPinOut, HIGH);

  // Pins for shiftIn
  pinMode(clockPinIn, OUTPUT);
  pinMode(latchPinIn, OUTPUT);
  pinMode(dataPinIn, INPUT);

  digitalWrite(clockPinIn, LOW);
  digitalWrite(latchPinIn, HIGH);

  // Update display and set motors to the default position
  showBoot();
  resetMotors(); // Reset all motors to the default position
  showValue(counter); // Update display to show counter value
}

void loop() {
  if (Serial.available()) { // Only used for debugging
    int input = Serial.read();
    if (input >= '0' && input <= '5')
      spinMotor(input - '0');
    else if (input == 'C')
      scrollDisplay(COLA);
    else if (input == 'P')
      scrollDisplay(PEPSI);
    else if (input == 'F')
      scrollDisplay(FANTA);
    else if (input == 'X')
      scrollDisplay(FAXE);
    else if (input == 'B')
      scrollDisplay(BEER);
  }

  checkStopMotor(); // Check if a motor has turned a half revolution
  checkAllSlots(); // Check if any slot is empty
  updateMotorsLEDs(); // Send out the new values to the shift register

  purchaseChecker(); // Check if a button has been pressed

  if (displayScrolling)
    updateScroll();
  else if (counter != lastCounter || (waitAfterButtonPress && (millis() - purchaseTimer > 1000))) { // Only update the LED matrix if a coin has been inserted or 1s after purchaseChecker() has printed something to the LED matrix
    showValue(counter);
    lastCounter = counter;
    waitAfterButtonPress = false;
  }
}

void scrollDisplay(const uint8_t* output) {
  if (output == NULL)
    return;
  pOutputString = (uint8_t*)output;
  displayScrolling = true;
  scrollPosition = 0;
  trailingSpaces = 0;
  scrollTimer = 0;
  memset(displayBuffer, OFF, sizeof(displayBuffer)); // Initialize all to OFF
}

void updateScroll() { // This should be called regularly after scrollDisplay() is called
  uint32_t timer = millis();
  if (timer - scrollTimer < 300)
    return;
  scrollTimer = timer;

  for (uint8_t i = sizeof(displayBuffer) - 1; i > 0 ; i--) // Shift array one to the left
    displayBuffer[i] = displayBuffer[i - 1];

  if (trailingSpaces == 0) { // Check if it is still reading the array
    displayBuffer[0] = *(pOutputString + scrollPosition); // Read new value into array
    if (displayBuffer[0] == OFF) // End char found
      trailingSpaces++;
    else
      scrollPosition++;
  } else
    trailingSpaces++; // End char is found, so just add trailing spaces until text is fully scrolled out

  if (trailingSpaces == sizeof(displayBuffer)) {
    displayScrolling = false;
    showValue(counter); // Show counter value on display again after scrolling the text
  } else
    printDisplay(displayBuffer);
}

void checkAllSlots() { // Check if any of the slots are empty
  uint32_t input = readSwitches();
  for (uint8_t i = 0; i < sizeof(motorToOutputMask); i++) {
    if (checkSlot(input, i))
      ledOutput |= motorToOutputMask[i];
    else
      ledOutput &= ~motorToOutputMask[i];
  }
}

void spinMotor(uint8_t motor) { // You must call checkStopMotor() to stop the motor again after it has done the half revolution
  motorTimer = millis();
  motorOutput |= motorToOutputMask[motor];
  ledOutput |= errorLedMask;
  updateMotorsLEDs();
  while (!motorSwitchPressed(readSwitches(), motor)) // Wait until switch is pressed
    delay(10);
}

void checkStopMotor() { // Stops motors after is has done a half revolution
  uint32_t input = readSwitches();
  for (uint8_t i = 0; i < sizeof(motorToOutputMask); i++) {
    if (!motorSwitchPressed(input, i) && motorOutput & motorToOutputMask[i]) { // Switch is released and motor is running
      motorOutput &= ~motorToOutputMask[i];
      ledOutput &= ~errorLedMask;
    }
  }
  
  if (motorOutput && millis() - motorTimer > 10000) // If the motor has been turning more than 10s, then it must be stuck
    error();
}

// TODO: If there is not enough money blink price
void purchaseChecker() {
  uint8_t price = 0;
  uint8_t buttonPressed = 0xFF; // No button is pressed

  uint32_t switchInput = readSwitches();
  for (uint8_t i = 0; i < sizeof(motorToInputMask); i++) {
    if (buyButtonPressed(switchInput, i)) {
      price = priceArray[i];
      buttonPressed = i;
      break;
    }
  }

  if (buttonPressed != 0xFF && buttonPressed != lastButtonPressed) {
    if (ledOutput & motorToOutputMask[buttonPressed]) { // Check if the selected item is available
      if (counter >= price) { // Purchase item
        if (!motorOutput) { // Check if any motor is spinning
          counter -= price;
          spinMotor(buttonPressed);
          scrollDisplay(nameArray[buttonPressed]);
        }
      } else { // Not enough money to buy item
        showValue(price); // Show the price of the item
        purchaseTimer = millis(); // Set up timer, so it clears it after a set amount of time
        waitAfterButtonPress = true;
      }
    } else {
      errorDisplay();
      purchaseTimer = millis(); // Set up timer, so it clears it after a set amount of time
      waitAfterButtonPress = true;
    }
  }
  lastButtonPressed = buttonPressed;
}

void cointInterrupt() {
  bool input = PIND & (1 << PIND2); // Read pin 2 directly using the port registers
  if (input && !lastCoinInput)
    counter += 5;
  lastCoinInput = input;
}

void showBoot() {
  uint8_t output[5];
  output[4] = B;
  output[3] = O;
  output[2] = O;
  output[1] = T1;
  output[0] = T2;
  printDisplay(output);
}

void errorDisplay() {
  uint8_t output[5];
  output[4] = dash; // '-'
  output[3] = E;
  output[2] = r;
  output[1] = r;
  output[0] = dash; // '-'
  printDisplay(output);
}

void showValue(uint16_t input) {
  uint8_t output[5];

  output[0] = numbers[input % 10];
  output[1] = (uint16_t)floor(input / 10) % 10;
  output[2] = (uint16_t)floor(input / 100) % 10;
  output[3] = (uint16_t)floor(input / 1000) % 10;
  output[4] = (uint16_t)floor(input / 10000) % 10;

  if (input < 10)
    output[1] = OFF;
  else
    output[1] = numbers[output[1]];

  if (input < 100)
    output[2] = OFF;
  else
    output[2] = numbers[output[2]];

  if (input < 1000)
    output[3] = OFF;
  else
    output[3] = numbers[output[3]];

  if (input < 10000)
    output[4] = OFF;
  else
    output[4] = numbers[output[4]];

  printDisplay(output);
}

void printDisplay(uint8_t* output) {
  digitalWrite(latchPinLED, LOW);
  shiftOut(dataPinLED, clockPinLED, MSBFIRST, output[0]);
  shiftOut(dataPinLED, clockPinLED, MSBFIRST, output[1]);
  shiftOut(dataPinLED, clockPinLED, MSBFIRST, output[2]);
  shiftOut(dataPinLED, clockPinLED, MSBFIRST, output[3]);
  shiftOut(dataPinLED, clockPinLED, MSBFIRST, output[4]);
  digitalWrite(latchPinLED, HIGH);
}

void resetMotors() { // Set all motors to the default position
  for (uint8_t i = 0; i < sizeof(motorToOutputMask); i++)
    motorOutput |= motorToOutputMask[i]; // Set all motors on

  uint32_t timer = millis();
  while (motorOutput != 0x00) {
    uint32_t input = readSwitches();
    for (uint8_t i = 0; i < sizeof(motorToOutputMask); i++) {
      if (!motorSwitchPressed(input, i)) // If switch is released stop motor
        motorOutput &= ~motorToOutputMask[i];
    }
    if (millis() - timer > 10000)
      error();
    updateMotorsLEDs();
    delay(10);
  }
}

void error() {
  errorDisplay();
  motorOutput = 0;
  ledOutput = 0;
  updateMotorsLEDs();
  while (1);
}

bool checkSlot(uint32_t input, uint8_t motor) {
  return !(input & motorToInputMask[motor]);
}

bool motorSwitchPressed(uint32_t input, uint8_t motor) {
  return input & ((uint16_t)motorToInputMask[motor] << 8);
}

bool buyButtonPressed(uint32_t input, uint8_t button) {
  return !(input & ((uint32_t)motorToInputMask[button] << 16));
}

void updateMotorsLEDs() {
  if (ledOutput == oldLedOutput && motorOutput == oldMotorOutput)
    return;
  oldLedOutput = ledOutput;
  oldMotorOutput = motorOutput;

  digitalWrite(latchPinOut, LOW); // Ground latchPin and hold low for as long as you are transmitting
  shiftOut(dataPinOut, clockPinOut, LSBFIRST, ledOutput);
  shiftOut(dataPinOut, clockPinOut, LSBFIRST, motorOutput);
  digitalWrite(latchPinOut, HIGH); // Return the latch pin high to signal chip that it no longer needs to listen for information
}

uint32_t readSwitches() {
  digitalWrite(latchPinIn, LOW);
  delayMicroseconds(20);
  digitalWrite(latchPinIn, HIGH); // Ground latchPin and hold low for as long as you are transmitting
  uint32_t input = shiftIn(dataPinIn, clockPinIn, LSBFIRST);
  input |= (uint16_t)shiftIn(dataPinIn, clockPinIn, LSBFIRST) << 8;
  input |= (uint32_t)shiftIn(dataPinIn, clockPinIn, LSBFIRST) << 16;
  return input;
}
