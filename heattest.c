// ===============================
// AUTHOR     	: SUKRU GORGULU
// CREATE DATE  : June 8th, 2020
// PURPOSE      : Heat Testing Unit Functions Implemented
// 
//==================================


#include <msp430g2553.h>
#include "heattest.h"

const uint8_t STATIC_LABEL[]      = {'T','e','m','p','e','r', 'a', 't', 'u', 'r', 'e'};
const uint8_t WAITBUTTON_LABEL[]  = {'W', 'A', 'I', 'T', 'S', ' ', 'B', 'T', 'T', 'N'};
const uint8_t SUCCESS_LABEL[]     = {'S', 'U', 'C', 'C', 'E', 'S', 'S', '.', ' ', ' '};
const uint8_t NOPOWER_LABEL[]     = {'N', 'O', ' ', 'P', 'O', 'W', 'E', 'R', '.', ' '};
const uint8_t DIALPROBLEM_LABEL[] = {'D', 'I', 'A', 'L', ' ', 'P', 'R', 'O', 'B', '.'};
const uint8_t REDIAL_LABEL[]      = {'D', 'I', 'A', 'L', ' ', 'P', 'R', 'O', 'B', '?'};
const uint8_t TOOLOW_LABEL[]      = {'T', 'M', 'P', ' ', 'T', 'O', 'O', ' ', 'L', 'O'};
const uint8_t TOOHIGH_LABEL[]     = {'T', 'M', 'P', ' ', 'T', 'O', 'O', ' ', 'H', 'I'};

const uint8_t wait_text[]         = "Waits for bttn ";
const uint8_t success_text[]      = "Success!       ";
const uint8_t nopower_text[]      = "No power inHeat";
const uint8_t dialproblem_text[]  = "Dial problem!  ";
const uint8_t redial_text[]       = "Dial problem?  ";
const uint8_t toolow_text[]       = "Temp. too low  ";
const uint8_t toohigh_text[]      = "Temp. too high ";
const uint8_t movingin_text[]     = "Moving intoUnit";
const uint8_t movingout_text[]    = "Moving out...  ";
const uint8_t findmax_text[]      = "Find maxT point";
const uint8_t foundmax_text[]     = "Found max point";
const uint8_t calibrate_text[]    = "Calibrating... ";
const uint8_t savecalibrate_text[]= "Saved Calibratd";
const uint8_t vacuumpumpon_text[] = "Vacuum pump ON ";
const uint8_t vacuumpumpoff_text[]= "Vacuum pump OFF";
const uint8_t heatingon_text[]    = "Heating ON     ";
const uint8_t heatingoff_text[]   = "Heating OFF    ";

void LimitSwitch_Init(void)
{
    /* Port Output Register. 1 makes output default high */
     P1OUT |= BIT4;

    /* Port 1 Direction Register. 0 makes it input */
    P1DIR &= ~BIT4;

    /* Port 1 Resistor Enable Register. pull-up */
    P1REN |= BIT4;
}

uint8_t LimitSwitch_Read(void)
{
    P1OUT |= BIT4;
    P1REN |= BIT4;
    wait();
    // the start button
  // when it gets low, it is activated.
    if ((P1IN & BIT4) != 0)
    {   wait(); // handle debouncing to be sure of false detection
       // if (P1IN & BIT3 == 0)
            return 1; // switch is activated
    }
    else
        {
        return 0; // switch is not pushed.
        }
}


void StartButton_Init(void)
{
    /* Port Output Register. 1 makes output default high */
     P1OUT |= BIT3;

    /* Port 1 Direction Register. 0 makes it input */
    P1DIR &= ~BIT3;

    /* Port Resistor Enable Register. pull-up */
    P1REN |= BIT3;
}

uint8_t StartButton_Read(void)
{
    P1OUT |= BIT3;
    P1REN |= BIT3;
    wait();
    // the start button
  // when it gets low, it is activated.
    if ((P1IN & BIT3) == 0)
    {
        wait(); // handle debouncing to be sure of false detection
        while((P1IN & BIT3) == 0) // waits until the button is released
            wait(); // handle debouncing to be sure of false detection
        return 1; //
    }
    else
        {
        return 0; // button is not pushed.
        }
}

void Solenoid_Init(void)
{
    /* Port Output Register. 1 makes output default high */
    // P3OUT = BIT1;

    /* Port Direction Register. 0 makes it input */
    P3DIR |= BIT1;
    P3OUT &=~BIT1;

    /* Port 1 Resistor Enable Register. pull-up */
    //YP3REN = BIT1;
}
void Solenoid_ON(void)
{
    P3OUT |= BIT1;
}
void Solenoid_OFF(void)
{
    P3OUT &= ~BIT1;
}

void VacuumPump_Init(void)
{
    /* Port 1 Output Register. 1 makes output high */
    // P3OUT = BIT0;

    /* Port 1 Direction Register. 0 makes it input */
    P3DIR |= BIT0;
    P3OUT &= ~BIT0;
    /* Port 1 Resistor Enable Register. pull-up */
    // P3REN = BIT0;
}
void VacuumPump_ON(void)
{
    P3OUT |= BIT0;
}

void VacuumPump_OFF(void)
{
    P3OUT &= ~BIT0;
}

void Init_Peripherals(void)
{
    // STEP-0. START AND MOVE TO ZERO POSITION
    StartButton_Init();
    LimitSwitch_Init();
    Solenoid_Init();
    VacuumPump_Init();
    MOTORS_INIT();
    //// Print_on_LCD(FIRST_LINE, 16, "INIT'S OK");
    DEBUG("Peripherals init'd\r\n")
    // Move_Motor1_ZEROPOS();
    //// Print_on_LCD(FIRST_LINE, 16,"ZEROPOS OK");
    // DEBUG("Moved Motor1 ZEROPOS");
    delayMiliSecond(500);
}

void Check_Dial(void)
{
    // [dial_problem] function: print on LCD: “dial problem?"
    // Print_on_LCD(FIRST_LINE, 13,"DIAL PROBLEM?");
    update_LCD(FIRST_LINE, COL_0, 16, redial_text);

    // Set LCD Background color YELLOW
    setRGB(0,255,255);

    // move motor-3 to mid-pos.
    // Move_Motor3_MIDPOS();
    // Print_on_LCD(FIRST_LINE, 15,"WAITS FOR START");
    update_LCD(FIRST_LINE, COL_0, 16, wait_text);
    while(StartButton_Read() != 1); // wait for human op. START button press.
    clear_LCD();
    setRGB(255,255,255);
}

void HeatingUnit_ON(void)
{ // Assumes Heating Unit is OFF
    // 3x (Activate solenoid and wait for 0.1 secs, then deactivate solenoid and wait for 0.1 secs)
    unsigned int i;
    for(i=0;i<3;i++)
    {
        Solenoid_ON();
        delayMiliSecond(100);
        Solenoid_OFF();
        delayMiliSecond(100);
    }
}

void HeatingUnit_OFF(void)
{ // Assumes heating unit is ON
    HeatingUnit_ON();
}

/*void Reset_Flags(void)
{
    flag_NOPOWER = 0;
    flag_DIALPROBLEM = 0;
    flag_TOOHIGH = 0;
    flag_TOOLOW = 0;
}*/

unsigned int Find_Max(int *Array, unsigned int ArrayLength)
{
    unsigned int i, Max_Index = 0;
    for(i=1;i<ArrayLength;i++)
        if(Array[i] > Array[Max_Index])
            Max_Index = i;
    return Max_Index;
}

void Run_Program(void)
{
    uint8_t flag_NOPOWER = 0;
    uint8_t flag_DIALPROBLEM = 0;
    uint8_t flag_TOOHIGH = 0;
    uint8_t flag_TOOLOW = 0;

    int AngleStop;
    const int min_Temp_Allowed = 10;
    const int Targeted_Temp = 22, Tolerance = 4;
    int TempValue,  MaxValue, NewValue, Array_TempValues[NUM_MOTOR3_POS];
    unsigned int MaxValuePos;
    int Delta;
    int motor2_currentPOS = 0; // Assumes motor-2 is initially in its starting position.
    int motor3_currentPOS = 0; // Assumes motor-3 is initially in its middle position.

    Print_on_LCD(FIRST_LINE, wait_text);
    while(StartButton_Read() != 1); // waits until start button is pressed

    // STEP-1. MOVE INTO THE UNIT TO PUSH THE MEASUREMENT TUBE INSIDE IT

    // Move Motor-1 forward to push the measurement tube inside the unit to be tested.
    // DONE: Motor-2 should rotate in conjunction with motor-1 to prevent any damage on measurement tube.
    Print_on_LCD(FIRST_LINE, movingin_text);
    motor2_currentPOS += Move_Motor1_Motor2_into_Unit(motor2_currentPOS);

    // STEP-2. TURN ON THE HEATING UNIT
    DEBUG("HeatingUnit_ON\r\n");
    HeatingUnit_ON();
    Print_on_LCD(FIRST_LINE, heatingon_text);

    // STEP-3. WAIT 6 SECONDS FOR UNIT TO HEAT UP
    delayMiliSecond(6000);

    // STEP-4. ACTIVATE VACUUM PUMP
    DEBUG("VacuumPump_ON\r\n");
    VacuumPump_ON();
    // Activate relay to turn on the vacuum pump
    Print_on_LCD(FIRST_LINE, vacuumpumpon_text);

    // STEP-5. WAIT 7 SECONDS FOR STABLE AIR MEASUREMENT
    delayMiliSecond(7000);

    //Read temperature
    TempValue = GetTemperatureValue();//GetTemperatureValue();
    delay_ms(200); // should wait mcp9600 to get ready for next call
    DEBUG(" Temperature: %d\r\n", TempValue);
    Temperature_on_LCD(FIRST_LINE, TempValue);

    // if value is below minimum temp. allowed (degrees celsius), then set NOPOWER flag and jump to STEP-7
    // #define MIN_TEMP_ALLOWED  in header file
    if (TempValue < min_Temp_Allowed)
    {
        DEBUG("NO POWER\r\n");
        flag_NOPOWER = 1;
    }
    else
    {
        // STEP-6. LOOK FOR THE POINT OF HIGHEST TEMPERATURE INSIDE THE UNIT

        // motor2_currentPOS +=
        Move_Motor2_MIDPOS(motor2_currentPOS); motor2_currentPOS = 0;
        motor2_currentPOS += motor2GotoPosition(-180);
        Print_on_LCD(FIRST_LINE, findmax_text);

        // Repeat for each angle-stops per 22.5 degrees: (360/22.5 = 16 angle-stops)
        for (AngleStop=0; AngleStop<16; AngleStop++)
        {
            // Move motor-2 to next angle-pos.
            motor2_currentPOS += motor2GotoPosition((360 / NUM_MOTOR3_POS));
            // Wait 3 secs.
            delayMiliSecond(3000);
            // Read temperature
            TempValue = GetTemperatureValue();
            // Save the value to values array
            Temperature_on_LCD(SECOND_LINE, TempValue);

            Array_TempValues[AngleStop] = TempValue;
            // Save the position to positions array
            DEBUG("Motor 2 in position: %d\r\n", motor2_currentPOS);
            DEBUG("Angle position: %d\r\n", (int)(AngleStop*(360/NUM_MOTOR3_POS))-180);
            DEBUG("Measured temperature: %d\r\n", TempValue);
            // TODO: Display angle and temp on lcd
        }

        // init_value = Find max. value in the values array.
        MaxValuePos = Find_Max(Array_TempValues, NUM_MOTOR3_POS);
        MaxValue = Array_TempValues[MaxValuePos];

        Move_Motor2_MIDPOS(motor2_currentPOS); motor2_currentPOS = 0;
        // DEBUG("Motor2 moved to start position\r\n");
        // Move motor-2 to the angle-pos of max. value.
        motor2_currentPOS += motor2GotoPosition(-180);
        motor2_currentPOS += motor2GotoPosition(MaxValuePos * (360 / NUM_MOTOR3_POS));

        Print_on_LCD(FIRST_LINE, foundmax_text);
        Temperature_on_LCD(SECOND_LINE, TempValue);

        DEBUG("Selected max. temperature: %d, %d \r\n", MaxValue, (int)(MaxValuePos*(360/NUM_MOTOR3_POS))-180);
        DEBUG("Motor2 moved to MaxValue position: %d, (%d degrees) \r\n", motor2_currentPOS, (int)(MaxValuePos*(360/NUM_MOTOR3_POS))-180);
        // TODO: display selected temp. and angle on lcd
        // and wait a moment for user to read

        // Repeat until init_value is below 180-4 or above 180+4:
        while((MaxValue < Targeted_Temp - Tolerance) || (MaxValue > Targeted_Temp + Tolerance))
        {
            Print_on_LCD(FIRST_LINE, calibrate_text);
            // if the value is below 180-4 degrees Celsius then rotate motor-3 CW 2 degrees, else anti-CW.
            if(MaxValue < Targeted_Temp - Tolerance)
                motor3_currentPOS += motor3GotoPosition(2);
            else if(MaxValue > Targeted_Temp - Tolerance)
                motor3_currentPOS += motor3GotoPosition(-2);

            DEBUG("Moved motor3 to %d steps\r\n", motor3_currentPOS);

            // Wait 3 secs. for adjustment registration.
            delayMiliSecond(3000);
            // Wait 7 secs. for temperature stabilization.
            delayMiliSecond(7000);

            // Read temperature again.
            NewValue = GetTemperatureValue();
            delay_ms(200); // should wait mcp9600 to get ready for next call
            //TODO: display new temperature value on lcd
            Temperature_on_LCD(FIRST_LINE, MaxValue);
            Temperature_on_LCD(SECOND_LINE, NewValue);

            // This delta variable will be used to adjust motor-3 movement later.
            Delta = NewValue - MaxValue;
            DEBUG("Delta is %d\r\n", Delta);

            if(Delta == 0)
            {
                MOTOR_3_SLEEP(); // release motor3 for human operator
                Check_Dial(); // then call [dial_problem] function here to wait for manual adjust by human operator.
                MOTOR_3_WAKEUP(); // hold motor3
                motor3_currentPOS = 0; // Assumes motor3 is brought to MIDPOS manually by human operator.
                delayMiliSecond(3000); // Wait for adjustment registration.
                delayMiliSecond(7000); // Wait for temperature stabilization.

                NewValue = GetTemperatureValue(); // Read temperature again.
                delay_ms(200); // should wait mcp9600 to get ready for next call
                Temperature_on_LCD(SECOND_LINE, NewValue);

                Delta = NewValue - MaxValue;
                if(Delta == 0)
                { // If delta is still zero, then set DIALPROBLEM flag, and jump to STEP-7.
                    flag_DIALPROBLEM = 1;
                    break;
                }
            }
            MaxValue = NewValue; // shift values for next while loop
        }

        // If init_value is still below, then set TOOLOW flag and jump to STEP-7
        if(MaxValue > Targeted_Temp + Tolerance)
        {
            flag_TOOHIGH = 1;
            // JUMP TO STEP-7
        }
        // If init_value is still above, then set TOOHIGH flag and jump to STEP-7
        if(MaxValue < Targeted_Temp - Tolerance)
        {
            flag_TOOLOW = 1;
            // JUMP TO STEP-7
        }
    }

    // STEP-7. SAVE THE MEASUREMENT DATA TO FLASH RAM AND TURN OFF THE PUMP

    //If NOPOWER or TOOLOW or TOOHIGH flags are set then skip storing and calibration ops.
    if ((flag_NOPOWER != 1) && (flag_TOOLOW != 1) && (flag_TOOHIGH != 1))
    {
        //TODO: Store the init_value and other necessary data to Flash ram.

        // Save the calibration to the unit:
        // Activate solenoid and wait for 0.1 secs, then deactivate solenoid and wait for 2 secs
        Print_on_LCD(FIRST_LINE, savecalibrate_text);
        Solenoid_ON();
        delayMiliSecond(100);
        Solenoid_OFF();
        delayMiliSecond(2000);
    }

    // Turn heating unit off:
    HeatingUnit_OFF();
    Print_on_LCD(FIRST_LINE, heatingoff_text);

    // turn off the vacuum pump
    VacuumPump_OFF();
    Print_on_LCD(FIRST_LINE, vacuumpumpoff_text);

    // STEP-8. MOVE BACK AND GET OUT OF UNIT TAKING CARE OF O-RINGS

    // Move motor-1 backwards until limit switch.
    // DONE: Move motor-2 in conjunction with motor to prevent any damage to o-rings
    Move_Motor2_MIDPOS(motor2_currentPOS);
    motor2_currentPOS = 0;

    Print_on_LCD(FIRST_LINE, movingout_text);
    Move_Motor1_Motor2_outof_Unit(motor2_currentPOS);

    // STEP-9. MAKE THE TEST MACHINE READY FOR THE NEXT UNIT TO BE INSERTED

    // Move motor-3 position to its initial position (middle position)
    Move_Motor3_MIDPOS(motor3_currentPOS);
    motor3_currentPOS = 0;

    // Move motor-2 back start position.
    // Move_Motor2_MIDPOS(motor2_currentPOS);
    // motor2_currentPOS = 0;

    // STEP-10. DISPLAY THE RESULT

    if (flag_NOPOWER == 1)
    {
        clear_LCD();
        Print_on_LCD(FIRST_LINE, nopower_text);
        // update_LCD(FIRST_LINE, COL_0, 10, &NOPOWER_LABEL);
        setRGB(255,0,0);// TODO: Set LCD Background color RED
    }
    else if (flag_TOOHIGH == 1)
    {
        clear_LCD();
        Print_on_LCD(FIRST_LINE, toohigh_text);
        // update_LCD(FIRST_LINE, COL_0, 10, &TOOHIGH_LABEL);
        setRGB(255,0,0);// TODO: Set LCD Background color RED
    }
    else if (flag_TOOLOW == 1)
    {
        clear_LCD();
        Print_on_LCD(FIRST_LINE, toolow_text);
        // update_LCD(FIRST_LINE, COL_0, 10, &TOOLOW_LABEL);
        setRGB(255,0,0);// TODO: Set LCD Background color RED
    }
    else if (flag_DIALPROBLEM == 1)
    {
        clear_LCD();
        Print_on_LCD(FIRST_LINE, dialproblem_text);
        // update_LCD(FIRST_LINE, COL_0, 10, &DIALPROBLEM_LABEL);
        setRGB(255,0,0);// TODO: Set LCD Background color RED
    }
    else
    {
        clear_LCD();
        Print_on_LCD(FIRST_LINE, success_text);
        // update_LCD(FIRST_LINE, COL_0, 10, &SUCCESS_LABEL);
        setRGB(0,255,0); //TODO: Set LCD Background color GREEN
    }
}

void StartButton_test(void)
{
    DEBUG("Push the StartButton\r\n");
    while(StartButton_Read() == 0);
    DEBUG("StartButton pushed\r\n");
}

void LimitSwitch_test(void)
{
    DEBUG("Waiting Limit Switch\r\n");
    while(LimitSwitch_Read() == 0);
    DEBUG("Limit Switch is activated\r\n");
}

void Solenoid_test(void)
{
    // to test io functions working
    DEBUG("Push Start to Solenoid ON:\r\n");

    while(StartButton_Read() == 0);
    DEBUG("StartButton pushed\r\n");

    Solenoid_ON();
    DEBUG("Solenoid ON\r\n");

    delayMiliSecond(2000);

    DEBUG("Push Start to Solenoid OFF:\r\n");
    while(StartButton_Read() == 0);
    DEBUG("StartButton pushed\r\n");

    Solenoid_OFF();
    DEBUG("Solenoid OFF\r\n");
}

void VacuumPump_test(void)
{
    // to test io functions working
    DEBUG("Push Start to V-Pump ON:\r\n");

    while(StartButton_Read() == 0);
    DEBUG("StartButton pushed\r\n");

    VacuumPump_ON();
    DEBUG("VacuumPump ON\r\n");

    delayMiliSecond(2000);

    DEBUG("Push Start to V-Pump OFF:\r\n");
    while(StartButton_Read() == 0);
    DEBUG("StartButton pushed\r\n");

    VacuumPump_OFF();
    DEBUG("VacuumPump OFF\r\n");
}

void HeatingUnit_test(void)
{
    // to test io functions working
    DEBUG("Push Start for HeatingUnit ON:\r\n");

    while(StartButton_Read() == 0);
    DEBUG("StartButton pushed\r\n");

    HeatingUnit_ON();
    DEBUG("HeatingUnit ON\r\n");

    delayMiliSecond(2000);

    DEBUG("Push Start for HeatingUnit OFF:\r\n");
    while(StartButton_Read() == 0);
    DEBUG("StartButton pushed\r\n");

    HeatingUnit_OFF();
    DEBUG("HeatingUnit OFF\r\n");
}

void io_test_all(void)
{
    StartButton_test();
    LimitSwitch_test();
    Solenoid_test();
    VacuumPump_test();
    HeatingUnit_test();
}

void LCD_message_test(void)
{
    // to test lcd working
    DEBUG("Push Start for LCD test 1:\r\n");
    while(StartButton_Read() == 0);
    DEBUG("StartButton pushed\r\n");
    clear_LCD();

    LCD_test();
    setRGB(255,0,0);
    delayMiliSecond(1000);

    DEBUG("Push Start for LCD test 2:\r\n");
    while(StartButton_Read() == 0);
    DEBUG("StartButton pushed\r\n");
    DEBUG("You see SUCCESS?\r\n")
    clear_LCD();
    Print_on_LCD(FIRST_LINE, (uint8_t *)&"SUCCESS");
    setRGB(0,255,0);
    delayMiliSecond(1000);

    DEBUG("Push Start for LCD test 3:\r\n");
    while(StartButton_Read() == 0);
    DEBUG("StartButton pushed\r\n");
    DEBUG("You see SUCCESS?\r\n");
    clear_LCD();

    update_LCD(FIRST_LINE, COL_0, 16, success_text);
    setRGB(0,0,255);
}
