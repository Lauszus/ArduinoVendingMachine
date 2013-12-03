#ifndef _ArduinoVendingMachine_h_
#define _ArduinoVendingMachine_h_

#include <Arduino.h>
#include "EEPROMAnything.h"

enum Letter {
  OFF = 0x00,
  SPACE = 0xFF,
  mdash = dash = 0xBF, // '-'
  ldash = 0xF7;
  hdash = 0xFE;
  A = 0x88, a = 0xA0,
  B = 0x80, b = 0x83,
  C = 0xC6, c = 0xA7,
  d = 0xA1,
  E = 0x86,
  F = 0x8E,
  g = 0x90,
  H = 0x89, h = 0x8B,
  I = 0xCF, i = 0xEF,
  J = 0xE0, j = 0xF1,
  K = 0x89,
  L = 0xC7,
  m1= 0xAF, m2= 0xAB,
  n = 0xAB,  
  O = 0xC0, o = 0xA3,
  P = 0x8C,
  Q = 0x40,
  R = 0x88, r = 0xAF,
  S = 0x92,
  T1= 0xF8, T2= 0xFE,
  u = 0xE3,
  V = 0xC1,
  // w
  X = 0x89,
  Y = 0x91,
  // z
};

const uint8_t COLA[] = { C, O, L, A, OFF };
const uint8_t PEPSI[] = { P, E, P, S, I, OFF };
const uint8_t FANTA[] = { F, A, n, T1, T2 , A, OFF };
const uint8_t FAXE[] = { F, A, X, E, OFF };
const uint8_t BEER[] = { B, E, E, r, OFF };
const uint8_t NO_REFUND[] = { n, o, SPACE, r, E, F, u, n, d, OFF };
const uint8_t TRAPPED[] = { H, E, L, P, SPACE, I, SPACE, A, m1, m2, SPACE, T1, T2, r, A, P, P, E, d, SPACE, i, n, SPACE, A, SPACE, V, E, n, d, i, n, g, SPACE , m1, m2, A, c, h, i, n, E, OFF };
const uint8_t LADDER[] = { ldash, mdash, hdash, mdash, ldash, mdash, hdash, mdash, ldash, OFF };

// Function prototypes
bool checkCoinSlots();
void coinChecker();
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
void delayNew(unsigned long ms);
void updateDry();
void tweetBoot();
void tweetStatus();

#endif
