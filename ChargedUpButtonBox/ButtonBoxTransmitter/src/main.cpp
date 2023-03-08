#include <Arduino.h>
#include <Buttonbox.h>

ButtonInfo DriveToCone1ButtonInfo = {0, 1};
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

Button CargoShipButtonRight = Button(12, 4);
Button CargoShipButtonBottom = Button(11, 5);

Button HeldButtonArray[] = {
    CargoShipButtonRight,
    CargoShipButtonBottom,
};
size_t HeldButtonArraySize = sizeof(HeldButtonArray) / sizeof(HeldButtonArray[0]);

// ToggleButtonGroup CenterToggleGroup = ToggleButtonGroup(ToggleButtonArray, ToggleButtonArraySize, CenterTopButton);
HeldButtonGroup CargoHeldGroup = HeldButtonGroup(HeldButtonArray, HeldButtonArraySize);

ToggleButton ConeToggleButton(14, 6, 7);
int x = 0;
void setup()
{
  // CenterToggleGroup.ActiveDefaultButton();
  ConeToggleButton.Enable();
}

void loop()
{
  // x = Serial.parseInt();
  // Serial.println(x);
  // usb_rawhid_recv(0, &x, 1, 1);
  // CenterToggleGroup.PollButtons();
  DriveToToggleGroup.PollButtons();
  CargoHeldGroup.PollButtons();
  ConeToggleButton.PollButton();
  delay(50);
}
