#include <Arduino.h>
#include <Buttonbox.h>

LED TeensyPowerLed(42);

ButtonInfo DriveToCone1ButtonInfo = {0, 1}; // button id on teensy, driverstation joystick id
ButtonInfo DriveToCube2ButtonInfo = {1, 2};
ButtonInfo DriveToCone3ButtonInfo = {2, 3};
ButtonInfo DriveToCone4ButtonInfo = {3, 4};
ButtonInfo DriveToCube5ButtonInfo = {4, 5};
ButtonInfo DriveToCone6ButtonInfo = {5, 6};
ButtonInfo DriveToCone7ButtonInfo = {43, 7};
ButtonInfo DriveToCube8ButtonInfo = {7, 8};
ButtonInfo DriveToCone9ButtonInfo = {8, 9};
ButtonInfo CancelButtonInfo = {9, 10};
ButtonInfo HumanPlayerLeftButtonInfo = {10, 11};
ButtonInfo HumanPlayerRightButtonInfo = {11, 12};

Button driveToButtons[] = {
    Button(DriveToCone1ButtonInfo),
    Button(DriveToCube2ButtonInfo),
    Button(DriveToCone3ButtonInfo),
    Button(DriveToCone4ButtonInfo),
    Button(DriveToCube5ButtonInfo),
    Button(DriveToCone6ButtonInfo),
    Button(DriveToCone7ButtonInfo),
    Button(DriveToCube8ButtonInfo),
    Button(DriveToCone9ButtonInfo),
    Button(CancelButtonInfo),
    Button(HumanPlayerLeftButtonInfo),
    Button(HumanPlayerRightButtonInfo),
};

size_t driveToButtonArraySize = sizeof(driveToButtons) / sizeof(driveToButtons[0]);
ToggleButtonGroup DriveToToggleGroup = ToggleButtonGroup(driveToButtons, driveToButtonArraySize, driveToButtons[9]);

ToggleButton ConeToggleButton(12, 13, 14);

ButtonInfo InjestButtonInfo = {15, 15};
ButtonInfo EjectButtonInfo = {16, 16};

Button HeldButtonArray[] = {
    Button(InjestButtonInfo),
    Button(EjectButtonInfo),
};
size_t HeldButtonArraySize = sizeof(HeldButtonArray) / sizeof(HeldButtonArray[0]);
HeldButtonGroup IntakeHeldGroup = HeldButtonGroup(HeldButtonArray, HeldButtonArraySize);

ButtonInfo ElevatorStowButtonInfo = {13, 17};
ButtonInfo ElevatorHumanButtonInfo = {14, 18};
ButtonInfo ElevatorLowButtonInfo = {18, 19};
ButtonInfo ElevatorMiddleButtonInfo = {38, 20};
ButtonInfo ElevatorHighButtonInfo = {20, 21};

Button ElevatorButtonArray[] = {
    Button(ElevatorStowButtonInfo),
    Button(ElevatorHumanButtonInfo),
    Button(ElevatorLowButtonInfo),
    Button(ElevatorMiddleButtonInfo),
    Button(ElevatorHighButtonInfo),
};
size_t ElevatorButtonArraySize = sizeof(ElevatorButtonArray) / sizeof(ElevatorButtonArray[0]);
ToggleButtonGroup ElevatorButtonGroup(ElevatorButtonArray, ElevatorButtonArraySize, ElevatorButtonArray[0]);

SingleToggleButton JoystickEnableToggle(17, 22);

PovInfo povInfo = {21, 24, 23, 22};
POV pov = POV(povInfo);

void setup()
{
  DriveToToggleGroup.ActiveDefaultButton();
  ElevatorButtonGroup.ActiveDefaultButton();
  ConeToggleButton.Enable();

  Serial.println("starting");
  TeensyPowerLed.enable();
}

void loop()
{
  DriveToToggleGroup.PollButtons();
  ConeToggleButton.PollButton();
  IntakeHeldGroup.PollButtons();
  JoystickEnableToggle.PollButton();
  ElevatorButtonGroup.PollButtons();
  pov.Poll();
  delay(20);
}
