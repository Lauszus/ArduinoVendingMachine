#ifndef _ArduinoVendingMachine_h_
#define _ArduinoVendingMachine_h_

#include <Arduino.h>
#include "EEPROMAnything.h"

enum Letter {
  OFF = 0x00,
  SPACE = 0xFF,

  C = 0xC6,
  O = 0xC0,
  L = 0xC7,
  A = 0x88,

  E = 0x86,
  r = 0xAF,
  dash = 0xBF, // '-'

  P = 0x8C,
  S = 0x92,
  I = 0xCF,

  F = 0x8E,
  n = 0xAB,
  T1 = 0xF8,
  T2 = 0xFE,

  j = 0xF1,
  J = 0xE0,
  a = 0xA0,
  Y = 0x91,

  X = 0x89,

  B = 0x80,
  R = A,

  V = 0xC1,
  d = 0xA1,
  u = 0xE3,
  o = 0xA3,

  i = 0xEF,
  g = 0x90,

  K = X,
};

const uint8_t COLA[] = { C, O, L, A, OFF };
const uint8_t PEPSI[] = { P, E, P, S, I, OFF };
const uint8_t FANTA[] = { F, A, n, T1, T2 , A, OFF };
const uint8_t FAXE[] = { F, A, X, E, OFF };
const uint8_t BEER[] = { B, E, E, r, OFF };
const uint8_t NO_REFUND[] = { n, o, SPACE, r, E, F, u, n, d, OFF };

// Function prototypes
bool checkCoinSlots();
void coinReturnCheck();
void sortArray(uint8_t *input, uint8_t size);
void scrollDisplay(const uint8_t *output);
void updateScroll();
void checkAllSlots();
void spinMotor(uint8_t motor);
void checkStopMotor();
void purchaseChecker();
void showError();
void showErrorJam();
void showErrorDry();
void cointInterrupt();
void showBoot();
void errorDisplay();
void showValue(uint16_t input);
void printDisplay(uint8_t *output);
void resetMotors();
void motorStuck(uint8_t motor);
bool checkSlot(uint32_t input, uint8_t motor);
bool motorSwitchPressed(uint32_t input, uint8_t motor);
bool buyButtonPressed(uint32_t input, uint8_t button);
void updateMotorsLEDs();
uint32_t readSwitches();

#endif
