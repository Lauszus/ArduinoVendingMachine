/*
 * The code is released under the GNU General Public License.
 * Developed by Kristian Lauszus, TKJ Electronics 2013
 * This code is used for an old vending machine where the main board did not work anymore
 */

#include "ArduinoVendingMachine.h"

// Change price of the items here:
const uint8_t priceArray[] = { 5, 5, 5, 5, 5, 5 };
// Change the name of the item here:
const uint8_t *nameArray[] = { LADDER, LADDER, LADDER, LADDER, LADDER, LADDER }; // See in ArduinoVendingMachine.h for the possible names. If the one you need is not present then type NULL instead
// Change value of the coin slots:
const uint8_t coinSlotValue[] = { 5, 0, 10 }; // Coin slots from right to left - note that the middle one is not connected at the moment
uint8_t coinSlotLeft[] = { 6, 0, 5 }; // Coins there is in the slot when it thinks it is empty - with safety margin of 1

// Do not change anything else below this line!
const uint8_t coinPin = 2; // Interrupt pin connected to the coin validator pulse pin
uint8_t lastButtonPressed;
uint32_t purchaseTimer;
bool waitAfterButtonPress;

const uint8_t clockPinLED = A1, dataPinLED = A2, latchPinLED = A0, resetPinLED = 13;
const uint8_t numbers[] = { 0xC0, 0xF9, 0xA4, 0xB0, 0x99, 0x92, 0x82, 0xF8, 0x80, 0x90 }; // Numbers for LED matrix

uint8_t displayBuffer[5];
uint8_t *pOutputString;
volatile bool displayScrolling;
uint8_t scrollPosition, trailingSpaces;
uint32_t scrollTimer;

bool lastCoinInput;
volatile uint8_t coinPulsesRecieved;
uint8_t lastCoinPulsesRecieved;
uint16_t counter, totalUnitsDispensed; // Counter for the credit currently in the machine and the total value of coins that have been put into the machine
uint16_t lastCounter;
uint32_t lastCoinPulseTime;

const uint8_t clockPinOut = 3, dataPinOut = 5, latchPinOut = 4, resetPinOut = 6; // Pins for driving the motors
const uint8_t clockPinIn = 11, dataPinIn = 9, latchPinIn = 7; // Pins used to check the switches

const uint8_t motorToOutputMask[] = { 0x04, 0x08, 0x10, 0x20, 0x40, 0x80 };
const uint8_t motorToInputMask[] = { 0x02, 0x04, 0x08, 0x10, 0x20, 0x40 };
const uint8_t errorLedMask = 0x02; //, greenLedMask = 0x01; // The green LED does not work at the moment

uint8_t motorOutput = 0, ledOutput = 0;
uint32_t motorTimer;
bool motorIsStuck[6];
//bool reportedDry[6];
uint32_t timeToNextTrapped;
uint32_t lastTrapped;

const uint8_t coinSolenoid[] = { 10, 0, A3 }; // Connected to the solenoids
const uint8_t coinSlot[] = { A6, 0, A7 }; // Analog input used to check if the coin slots are empty
const uint8_t coinReturn = A5; // Return button
const uint8_t COIN_EMPTY = 500; // If the ADC value gets below this value, then the coin slot is empty
uint32_t refundTimer;

void setup() {
  Serial.begin(19200); // Initialize serial communications with master vending arduino
  Serial.setTimeout(400);

  // Pins for LED matrix
  digitalWrite(clockPinLED, LOW);
  digitalWrite(latchPinLED, HIGH);
  digitalWrite(resetPinLED, HIGH);

  pinMode(clockPinLED, OUTPUT);
  pinMode(latchPinLED, OUTPUT);
  pinMode(resetPinLED, OUTPUT);
  pinMode(dataPinLED, OUTPUT);

  // Pins used for shiftOut
  digitalWrite(dataPinOut, LOW);
  digitalWrite(clockPinOut, LOW);
  digitalWrite(resetPinOut, HIGH); // Reset first
  digitalWrite(latchPinOut, HIGH); // Then latch

  pinMode(clockPinOut, OUTPUT);
  pinMode(latchPinOut, OUTPUT);
  pinMode(resetPinOut, OUTPUT);
  pinMode(dataPinOut, OUTPUT);

  // Pins for shiftIn
  pinMode(clockPinIn, OUTPUT);
  pinMode(latchPinIn, OUTPUT);
  pinMode(dataPinIn, INPUT);

  digitalWrite(clockPinIn, LOW);
  digitalWrite(latchPinIn, HIGH);

  // Setup outputs for solenoids
  for (uint8_t i = 0; i < sizeof(coinSolenoid); i++) {
    pinMode(coinSolenoid[i], OUTPUT);
    digitalWrite(coinSolenoid[i], LOW); // Make sure it is low by default
  }

  pinMode(coinReturn, INPUT_PULLUP);

  // Update display and set motors to the default position
  showBoot();
  delay(2000);
  resetMotors(); // Reset all motors to the default position
  if (!checkCoinSlots()) {
    scrollDisplay(NO_REFUND); // If there is no coins left show "No refund"
    refundTimer = millis();
  } 
  else
    showValue(counter); // Update display to show counter value

  pinMode(coinPin, INPUT); // Setup coin input
  counter = lastCounter = coinPulsesRecieved = lastCoinPulsesRecieved = 0;
  EEPROM_readAnything(0, totalUnitsDispensed); // Read value from EEPROM
  delay(300); // Make sure the voltage is stable
  //updateDryNoOutput();
  lastTrapped = 0;
  randomSeed(analogRead(A4)); // Use analog input as random seed
  timeToNextTrapped = random(1000000, 3000000);
  attachInterrupt(0, cointInterrupt, CHANGE);
}

#define TRANSMISSION_REPEATS 5
#define TRANSMISSION_SPACE 95
#define TRANSMISSION_ATOM_SIZE 3
uint8_t recieve_error;

uint16_t rfid_raw_read(){
  char parseBuffer[TRANSMISSION_REPEATS*TRANSMISSION_ATOM_SIZE];
  uint16_t number = 0;
  recieve_error = 1; // set default to error
  if(Serial.readBytes(parseBuffer, sizeof(parseBuffer)) == sizeof(parseBuffer)){
    // Recieved correct ammount of bytes - check message integrity
    if(memcmp(parseBuffer,parseBuffer+TRANSMISSION_ATOM_SIZE,sizeof(parseBuffer)-TRANSMISSION_ATOM_SIZE) == 0){
      if(parseBuffer[2] == TRANSMISSION_SPACE){
        // OK
        recieve_error = 0;
        // Let other party know we do not need a retransmission
        memcpy(&number,parseBuffer,sizeof(number));
      }
    }
  }
  return number;
}

char rfid_raw_transmit(uint16_t number){
  for (int i = 0; i < TRANSMISSION_REPEATS*TRANSMISSION_ATOM_SIZE; i = i+TRANSMISSION_ATOM_SIZE) {
    Serial.write(number & 0xFF);
    Serial.write(number >> 8);
    Serial.write(TRANSMISSION_SPACE);
  }
}

void loop() {
  if (Serial.available()) {
    int input = Serial.read();
    // Only used for debugging
    //if (input >= '0' && input <= '5') {
    //spinMotor(input - '0');
    //}
    //RFID functionality
    if(input == 'C'){ // Fetch current credits
      rfid_raw_transmit(counter);
    }
    else if(input == 'S'){ // Set current credits
      uint16_t temp_counter = rfid_raw_read();
      if(recieve_error == 0){ // If credits recieved correctly, update counter
        counter = temp_counter;
        Serial.write(0);
      } else {Serial.write(255);} // Did not recieve correctly - notify other party
      showValue(counter);
    }
    else if(input == 'Z'){ // Zero current credits
      counter = 0;
      showValue(counter);
    }
    else if (input == 'B')
      scrollDisplay(ERR_EEPROM_BAD);
    else if (input == 'O')
      scrollDisplay(ERR_OUT_OF_MEM);
    else if (input == 'N')
      scrollDisplay(ERR_NO_CREDIT);
    /*else if (input == 'C')
     scrollDisplay(COLA);
    else if (input == 'P')
     scrollDisplay(PEPSI);
     else if (input == 'F')
     scrollDisplay(FANTA);
     else if (input == 'X')
     scrollDisplay(FAXE);
     else if (input == 'B')
     scrollDisplay(BEER);
     else if (input == 'N')
     scrollDisplay(NO_REFUND);
     else if (input == 'T')
     scrollDisplay(TRAPPED);*/
    else if (input == 'E')
      Serial.println(totalUnitsDispensed);
    else if (input == 'R') {
      Serial.print("EEPROM was reset - old value: ");
      Serial.println(totalUnitsDispensed);
      totalUnitsDispensed = 0;
      EEPROM_updateAnything(0, totalUnitsDispensed);
    }/* else if (input == 'S') {
     tweetStatus();
     }*/
  }

  checkStopMotor(); // Check if a motor has turned a half revolution
  checkAllSlots(); // Check if any slot is empty
  updateMotorsLEDs(); // Send out the new values to the shift register
  coinChecker(); // Check if any coins have been inserted
  //updateDry(); // Check for empty slots, and tweet if any slots become empty

  purchaseChecker(); // Check if a button has been pressed
  coinReturnCheck(); // Check if the coin return button is pressed

  if (millis() - lastTrapped > timeToNextTrapped) {
    scrollDisplay(TRAPPED); // Show stuck in vendingmachine
    lastTrapped = millis();
    timeToNextTrapped = random(100000, 3000000);
  }

  if (displayScrolling)
    updateScroll();
  else if (!checkCoinSlots() && (millis() - refundTimer > 12000)) { // Scroll "No refund" every 12s
    scrollDisplay(NO_REFUND); // If there is no coins left show "No refund"
    refundTimer = millis();
  } 
  else if ((!waitAfterButtonPress && counter != lastCounter) || (waitAfterButtonPress && (millis() - purchaseTimer > 1000))) { // Only update the LED matrix if a coin has been inserted or 1s after purchaseChecker() has printed something to the LED matrix
    showValue(counter);
    lastCounter = counter;
    waitAfterButtonPress = false;
  }
}

bool checkCoinSlots() {
  bool output = true;
  for (uint8_t i = 0; i < sizeof(coinSlot); i++) {
    if (coinSlotValue[i] > 0 && analogRead(coinSlot[i]) < COIN_EMPTY){ // Check if coin slot is empty
      output = false;
    }
  }
  return output;
}

void coinChecker() {
  if (coinPulsesRecieved != lastCoinPulsesRecieved) { // Only run the check if pulses has changed
    if (coinPulsesRecieved > 1) { // Accept coin(s) and reset coin pulses
      cli(); // Disable interrupts to make sure we don't disregard any coins
      uint8_t coins = coinPulsesRecieved >> 1; // Get count of "whole" coins
      coinPulsesRecieved -= coins << 1; // Substract "whole" coins from pulses recieved (we could be between pulses)
      sei(); // Enable interrupts again
      counter += coins * 5;
      lastCoinPulseTime = 0;
    }
    lastCoinPulsesRecieved = coinPulsesRecieved;
  } 
  else if (coinPulsesRecieved == 1) { // If pulses is 1, and has not changed for 150ms, reset pulse count
    if (lastCoinPulseTime == 0) // If timer is not set, the pulse was just recieved
      lastCoinPulseTime = millis();
    else if (millis() - lastCoinPulseTime > 150) // Faux pulse - reset everything
      lastCoinPulseTime = coinPulsesRecieved = lastCoinPulsesRecieved = 0;
  }
}

void coinReturnCheck() {
  if (counter && analogRead(coinReturn) < 50) { // The button is normally high
    uint8_t sortedArray[sizeof(coinSlotValue)];
    memcpy(sortedArray, coinSlotValue, sizeof(coinSlotValue)); // Copy array
    sortArray(sortedArray, sizeof(sortedArray)); // Sort the array in descending order

    for (uint8_t i = 0; i < sizeof(sortedArray); i++) {
      for (uint8_t j = 0; j < sizeof(coinSlotValue); j++) {
        if (sortedArray[i] > 0 && coinSlotValue[j] == sortedArray[i]) {
          while (counter >= coinSlotValue[j]) { // Keep releasing coins until the counter is lower than the value
            if (analogRead(coinSlot[j]) < COIN_EMPTY && coinSlotLeft[j] == 0) // Check if coin slot is empty
              break;
            else {
              digitalWrite(coinSolenoid[j], HIGH); // Pulse solenoid
              delayNew(250); // Turn on solenoid
              digitalWrite(coinSolenoid[j], LOW);
              delayNew(250); // Make sure coin is released

              if (analogRead(coinSlot[j]) < COIN_EMPTY)
                coinSlotLeft[j]--;

              counter -= coinSlotValue[j];
              showValue(counter);
            }
          }
        }
      }
    }
    if (counter != 0)
      refundTimer = 0;
  }
}

void sortArray(uint8_t *input, uint8_t size) { // Inspired by: http://www.tenouk.com/cpluscodesnippet/sortarrayelementasc.html
  for (uint8_t i = 1; i < size; i++) {
    for (uint8_t j = 0; j < size - 1; j++) {
      if (input[j] < input[j + 1]) {
        uint8_t temp = input[j];
        input[j] = input[j + 1];
        input[j + 1] = temp;
      }
    }
  }
}

void scrollDisplay(const uint8_t *output) {
  if (output == NULL)
    return;
  pOutputString = (uint8_t*)output;
  displayScrolling = true;
  scrollPosition = 0;
  trailingSpaces = 0;
  scrollTimer = 0;
  memset(displayBuffer, SPACE, sizeof(displayBuffer)); // Initialize all to off
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
    if (displayBuffer[0] == OFF) { // End char found
      displayBuffer[0] = SPACE; // Set LEDs off
      trailingSpaces++;
    } 
    else
      scrollPosition++;
  } 
  else
    trailingSpaces++; // End char is found, so just add trailing spaces until text is fully scrolled out

  if (trailingSpaces == sizeof(displayBuffer))
    showValue(counter); // Show counter value on display again after scrolling the text
  else {
    printDisplay(displayBuffer);
    displayScrolling = true;
  }
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
      totalUnitsDispensed++;
      EEPROM_updateAnything(0, totalUnitsDispensed);
    }
  }

  if (motorOutput && millis() - motorTimer > 10000) { // If the motor has been turning more than 10s, then it must be stuck
    for (uint8_t i = 0; i < sizeof(motorToOutputMask); i++) {
      if (motorOutput & motorToOutputMask[i]) { // Motor is running
        counter += priceArray[i]; // Give back credit
        motorStuck(i);
        showErrorJam(); // Show error for 1s
      }
    }
  }
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
      } 
      else { // Not enough money to buy item
        showValue(price); // Show the price of the item
        purchaseTimer = millis(); // Set up timer, so it clears it after a set amount of time
        waitAfterButtonPress = true;
      }
    } 
    else {
      if (motorIsStuck[buttonPressed] == true)
        showErrorJam(); // Show error for 1s
      else
        showErrorDry(); // Show error for 1s
    }
  }
  lastButtonPressed = buttonPressed;
}

void showError() {
  errorDisplay();
  purchaseTimer = millis(); // Set up timer, so it clears it after a set amount of time
  waitAfterButtonPress = true;
}

void showErrorJam() {
  uint8_t output[5];
  output[4] = SPACE;
  output[3] = j;
  output[2] = A;
  output[1] = r;
  output[0] = n;
  printDisplay(output);

  purchaseTimer = millis(); // Set up timer, so it clears it after a set amount of time
  waitAfterButtonPress = true;
}

void showErrorDry() {
  uint8_t output[5];
  output[4] = SPACE;
  output[3] = d;
  output[2] = r;
  output[1] = Y;
  output[0] = SPACE;
  printDisplay(output);

  purchaseTimer = millis(); // Set up timer, so it clears it after a set amount of time
  waitAfterButtonPress = true;
}

void cointInterrupt() {
  bool input = PIND & (1 << PIND2); // Read pin 2 directly using the port registers
  if (!input && lastCoinInput)
    coinPulsesRecieved++;
  lastCoinInput = input;
  displayScrolling = false;
}

void showBoot() {
  uint8_t output[5];
  output[4] = B;
  output[3] = O;
  output[2] = O;
  randomSeed(analogRead(A4)); // Use analog input as random seed
  if(random(1, 5) == 1){
    output[1] = T1;
    output[0] = T2;
  } 
  else {
    output[1] = B;
    output[0] = S;
  }
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

  memset(output + 1, SPACE, sizeof(output) - 1); // Initialize the rest to off
  if (input >= 10) {
    output[1] = numbers[(input / 10) % 10];
    if (input >= 100) {
      output[2] = numbers[(input / 100) % 10];
      if (input >= 1000) {
        output[3] = numbers[(input / 1000) % 10];
        if (input >= 10000)
          output[4] = numbers[(input / 10000) % 10];
      }
    }
  }
  printDisplay(output);
}

void printDisplay(uint8_t *output) {
  displayScrolling = false; // Stop scrolling by default
  digitalWrite(latchPinLED, LOW);
  for (uint8_t i = 0; i < 5; i++)
    shiftOut(dataPinLED, clockPinLED, MSBFIRST, output[i]);
  digitalWrite(latchPinLED, HIGH);
}

bool checkMotors(){
  uint8_t motorsDone = 0;
  uint32_t input = readSwitches();
  for (uint8_t i = 0; i < sizeof(motorToOutputMask); i++) {
    if (!motorSwitchPressed(input, i)) // If switch is released stop motor
      motorsDone++;
  }
  if(motorsDone == sizeof(motorToOutputMask))
    return true;
  return false;
}

void resetMotors() { // Set all motors to the default position
  if(checkMotors()){ // If all motors are in correct position, write motorOutput to zero and return
    updateMotorsLEDs();
    return;
  }

  for (uint8_t i = 0; i < sizeof(motorToOutputMask); i++)
    motorOutput |= motorToOutputMask[i]; // Set all motors on in the buffer

  uint32_t timer = millis();
  while (motorOutput != 0x00) {
    uint32_t input = readSwitches();
    for (uint8_t i = 0; i < sizeof(motorToOutputMask); i++) {
      if (!motorSwitchPressed(input, i)) // If switch is released stop motor
        motorOutput &= ~motorToOutputMask[i];
    }
    if (millis() - timer > 10000) { // Motors running now must be stuck
      for (uint8_t i = 0; i < sizeof(motorToOutputMask); i++) {
        if (motorOutput & motorToOutputMask[i]) // Motor is running
          motorStuck(i);
      }
    }
    updateMotorsLEDs();
    delay(2);
  }
}

void motorStuck(uint8_t motor) {
  motorOutput &= ~motorToOutputMask[motor]; // Turn off motor
  motorIsStuck[motor] = true;
}

bool checkDry(uint32_t input, uint8_t motor) {
  return (input & motorToInputMask[motor]);
}

bool checkSlot(uint32_t input, uint8_t motor) {
  return !(input & motorToInputMask[motor]) && !motorIsStuck[motor];
}

bool motorSwitchPressed(uint32_t input, uint8_t motor) {
  return input & ((uint16_t)motorToInputMask[motor] << 8);
}

bool buyButtonPressed(uint32_t input, uint8_t button) {
  return !(input & ((uint32_t)motorToInputMask[button] << 16));
}

void updateMotorsLEDs() {
  digitalWrite(latchPinOut, LOW); // Ground latchPin and hold low for as long as you are transmitting
  shiftOut(dataPinOut, clockPinOut, LSBFIRST, ledOutput);
  shiftOut(dataPinOut, clockPinOut, LSBFIRST, motorOutput);
  digitalWrite(latchPinOut, HIGH); // Return the latch pin high to signal chip that it no longer needs to listen for information
}

uint32_t readSwitches() {
  digitalWrite(latchPinIn, LOW);
  delayMicroseconds(20);
  digitalWrite(latchPinIn, HIGH);
  uint32_t input = shiftIn(dataPinIn, clockPinIn, LSBFIRST);
  input |= (uint16_t)shiftIn(dataPinIn, clockPinIn, LSBFIRST) << 8;
  input |= (uint32_t)shiftIn(dataPinIn, clockPinIn, LSBFIRST) << 16;
  return input;
}

void delayNew(unsigned long ms) { // Just a copy of the normal delay(), but also checks if motor should be stopped
  uint16_t start = (uint16_t)micros();
  uint8_t oldMotorOutput;

  while (ms > 0) {
    if (((uint16_t)micros() - start) >= 1000) {
      ms--;
      start += 1000;

      oldMotorOutput = motorOutput;
      checkStopMotor(); // Check if motor should be stopped
      if (motorOutput != oldMotorOutput) // Check if motor output is updated
        updateMotorsLEDs(); // Update motor and LED output
    }
  }
}
/*
void updateDry() { // Check if any of the slots are empty, and updates the Dry information
  uint32_t input = readSwitches();
  for (uint8_t i = 0; i < sizeof(motorToOutputMask); i++) {
    if (checkDry(input, i)) {
      if (!reportedDry[i])
        reportedDry[i] = true;
    }
    else {
      if (reportedDry[i])
        reportedDry[i] = false;
    }
  }
}

void updateDryNoOutput() {
  // updateDry without printing
  uint32_t input = readSwitches();
  for (uint8_t i = 0; i < sizeof(motorToOutputMask); i++)
    if (checkDry(input, i))
      if (!reportedDry[i])
        reportedDry[i] = true;
}

void tweetStatus() {
  // Show beverages dispensed (sold)
  Serial.print("S");
  Serial.print(totalUnitsDispensed);

  // Print all jammed slots
  Serial.print(",J");
  for (uint8_t i = 0; i < sizeof(motorIsStuck); i++)
    if (motorIsStuck[i])
      Serial.print(i);

  // Print all empty slots
  Serial.print(",D");
  for (uint8_t i = 0; i < sizeof(reportedDry); i++)
    if (reportedDry[i])
      Serial.print(i);

  // Print all empty coin slots
  Serial.print(",C");
  if (coinSlotLeft[0] == 0)
    Serial.print("0");
  if (coinSlotLeft[2] == 0)
    Serial.print("2");

  Serial.println();
}
*/
