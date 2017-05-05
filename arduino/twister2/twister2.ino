/*
 * ----------------------------------------------------------------------------
 * "THE BEER-WARE LICENSE" (Revision 42):
 * <jpnewman snail mit dot edu> wrote this stuff. As long as you retain this
 * notice you can do whatever you want with this stuff. If we meet some day,
 * and you think this stuff is worth it, you can buy me a beer in return.
 * Jon
 * ----------------------------------------------------------------------------
 */

// This script is used to control the Twister2 device. It is very shitty.

#include <AccelStepper.h>
#include <LiquidCrystal.h>

#define POSGREY(sum)                                                           \
    (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011)
#define NEGGREY(sum)                                                           \
    (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)

// Button read parameters
#define HOLD_MSEC 500

// Stepper parameters
#define USTEPS_PER_REV (200 * 16)
#define MAX_RPM     500
#define MOT_DIR     19
#define MOT_STEP    18
#define MOT_EN      17

// Rotary encoder pins
#define ENC_0 22
#define ENC_1 23
#define ENC_B 21

//#define DEBUG

const int encoder_p0 = 22;
const int encoder_p1 = 23;
const int encoder_but = 21;

volatile bool encoder_updated = false;
volatile int encoder_dir = 0;

float forward_turns = 80;
float back_turns = 40;
float turn_speed_rpm = 240;
volatile bool start_requested = false;

enum SelectedMode { fwd_turns, bck_turns, turn_spd, num_modes };
volatile SelectedMode selected_mode = fwd_turns;

long last_complete_turn = 0;
AccelStepper stepper(1, MOT_STEP, MOT_DIR);

volatile bool disp_update_requested = false;
LiquidCrystal lcd(0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10);

elapsedMillis sinceButtonChange;

// Return code
// -2 : Wrong value compared to desired_value
// -1 : Did not pass debounce. Should be ignored.
//  1 : Normal button press.
//  2 : Hold value maintained for longer than wait period.
// NOTE: Need interrupts enabled for this to work properly.
static int readButton(int pin,
                      int *value,
                      int min_hold_usec = 0 ,
                      size_t min_interval_msec = 0,
                      bool hold_value = false,
                      int desired_value = -1)
{
    if (sinceButtonChange < min_interval_msec)
        return -1;

    // Make sure button state equals desired value for at least min_hold_usec
    *value = digitalRead(pin);

    if (desired_value != 1 && desired_value != *value)
        return -2;

    for (int i = 0; i < min_hold_usec; i++) {
        if (*value != digitalRead(pin))
            return -1;
        delayMicroseconds(1);
    }

    // Log button change time
    sinceButtonChange = 0;

    if (!hold_value)
        return 1;

    // See if the user keeps button held for HOLD_MSEC
    while (sinceButtonChange < HOLD_MSEC) {
        if (*value != digitalRead(pin))
            return 1;
    }

    return 2;
}

static void showTurns(long current_step)
{
    if (current_step % USTEPS_PER_REV == 0) {
        lcd.clear();
        lcd.print("Turns:");
        lcd.setCursor(0,1);
        lcd.print( current_step / USTEPS_PER_REV);
    }
}

int executeTwist(void)
{
    // Unsleep the driver
    digitalWrite(MOT_EN, HIGH);

    // Set speed
    auto max_speed = (float)USTEPS_PER_REV * turn_speed_rpm / 60.0;
    stepper.setMaxSpeed(max_speed);
    stepper.setAcceleration(2000); //max_speed b/ 5);

    lcd.clear();
    lcd.print("Twisting...");

    // Forward motion
    stepper.move((long)forward_turns * (long)USTEPS_PER_REV);
    auto distance_to_go = stepper.distanceToGo();

    while (distance_to_go != 0 && start_requested) {
        stepper.run();
        distance_to_go = stepper.distanceToGo();
        //showTurns(stepper.currentPosition());
    }

    lcd.clear();
    lcd.print("Untwisting...");

    // Forward motion
    stepper.move(-(long)back_turns * (long)USTEPS_PER_REV);
    distance_to_go = stepper.distanceToGo();

    while (distance_to_go != 0 && start_requested) {
        stepper.run();
        distance_to_go = stepper.distanceToGo();
        //showTurns(stepper.currentPosition());
    }

    if (!start_requested) {
        stepper.stop();
        lcd.clear();
        lcd.print("Twist stopped.");
    }else {
        lcd.clear();
        lcd.print("Twist complete.");
    }

    stepper.setCurrentPosition(0);

    // Unsleep the driver
    digitalWrite(MOT_EN, LOW);

    // 0 = successful move
    // 1 = emergency stop
    return 0;
}

void setup()
{
    pinMode(encoder_p0, INPUT);
    pinMode(encoder_p1, INPUT);
    pinMode(encoder_but, INPUT);

    pinMode(MOT_EN, OUTPUT);

    attachInterrupt(encoder_p0, updateEncoder, CHANGE);
    attachInterrupt(encoder_p1, updateEncoder, CHANGE);
    attachInterrupt(encoder_but, toggleMode, FALLING);

    stepper.setMinPulseWidth(3);

    lcd.begin(16, 2);
    lcd.print("Twister 2");
    lcd.setCursor(0,1);
    lcd.print("JPN MWL MIT");

    delay(2000);
    disp_update_requested = true;
}

void loop()
{
    if (encoder_updated) {

        switch (selected_mode) {

            case fwd_turns: {
                forward_turns += (float)encoder_dir/5;
                if (forward_turns > 200)
                    forward_turns = 200;
                else if (forward_turns < 0)
                    forward_turns = 0;

                break;
            }
            case bck_turns: {
                back_turns += (float)encoder_dir/5;
                if (back_turns > 200)
                    back_turns = 200;
                else if (back_turns < 0)
                    back_turns = 0;

                break;
            }
            case turn_spd: {
                turn_speed_rpm += 2 * (float)encoder_dir;
                if (turn_speed_rpm > 1000)
                    turn_speed_rpm = 1000;
                else if (turn_speed_rpm < 0)
                    turn_speed_rpm = 0;

                break;
            }
        }

        encoder_updated = false;
    }

    if (disp_update_requested) {

        lcd.clear();

        switch (selected_mode) {

            case fwd_turns: {
                lcd.print("Forward turns:");
                lcd.setCursor(0,1);
                lcd.print((int)forward_turns);
                break;
            }
            case bck_turns: {
                lcd.print("Backward turns:");
                lcd.setCursor(0,1);
                lcd.print((int)back_turns);
                break;
            }
            case turn_spd: {
                lcd.print("Speed (RPM):");
                lcd.setCursor(0,1);
                lcd.print((int)turn_speed_rpm);
                break;
            }
        }

        disp_update_requested = false;
    }

    if (start_requested) {
        executeTwist();
        start_requested = false;
    }


}

volatile int last_enc = 0;
void updateEncoder()
{
    noInterrupts();

    int msb;
    if (readButton(encoder_p0, &msb) == -1)
        return;

    int lsb;
    if (readButton(encoder_p1, &lsb) == -1)
        return;

    // converting the 2 pin value to single number
    int encoded = (msb << 1) | lsb;

    // adding it to the previous encoded value
    int sum = (last_enc << 2) | encoded;
    last_enc = encoded; // store this value for next time

    // Get direction
    if (POSGREY(sum))
        encoder_dir = 1;
    else if (NEGGREY(sum))
        encoder_dir = -1;
    else
        encoder_dir = 0;

    // Set flags
    encoder_updated = true;
    disp_update_requested = true;
    interrupts();
    return;
}

void toggleMode()
{
    int val;
    auto hold = readButton(encoder_but, &val, 1000, 10, true, 0);

    noInterrupts();
    switch (hold) {
        case 2:{
            start_requested = true;
            break;
        }
        case 1: {
            if (start_requested) {
                start_requested = false;
            } else {
                int sm = (int)selected_mode;
                sm++;
                if (sm >= num_modes)
                    sm = 0;
                selected_mode = (SelectedMode)sm;
            }
            break;
        }
    }

    disp_update_requested = true;

    interrupts();
    return;
}
