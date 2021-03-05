// ===============================
// AUTHOR     	: SUKRU GORGULU
// CREATE DATE  : June 8th, 2020
// PURPOSE      : Heat Testing Unit Functions Implemented
// 
//==================================

#include <msp430g2553.h>
#include "lcd.h"
#include "mcp9600.h"
#include "easydriver.h"
#include "debug.h"

#define NUM_MOTOR3_POS 16 // number of angle positions for motor-3 to stop

void LimitSwitch_Init(void);
uint8_t LimitSwitch_Read(void);
void StartButton_Init(void);
uint8_t StartButton_Read(void);
void Solenoid_Init(void);
void Solenoid_ON(void);
void Solenoid_OFF(void);
void VacuumPump_Init(void);
void VacuumPump_ON(void);
void VacuumPump_OFF(void);
void HeatingUnit_ON(void);
void HeatingUnit_OFF(void);
void Init_Peripherals(void);
void Check_Dial(void);
unsigned int Find_Max(int *Array, unsigned int ArrayLength);
void Run_Program(void);
void StartButton_test(void);
void LimitSwitch_test(void);
void Solenoid_test(void);
void VacuumPump_test(void);
void HeatingUnit_test(void);
void LCD_message_test(void);
