#include <Arduino.h>
#include <Buttonbox.h>

Button CenterTopButton = Button(8, 1);
Button CenterMiddleButton = Button(5, 2);
Button CenterBottomButton = Button(2, 3);

Button ToggleButtonArray[] = {
    CenterTopButton,
    CenterMiddleButton,
    CenterBottomButton,
};
size_t ToggleButtonArraySize = sizeof(ToggleButtonArray) / sizeof(ToggleButtonArray[0]);

Button CargoShipButtonRight = Button(12, 4);
Button CargoShipButtonBottom = Button(11, 5);

Button HeldButtonArray[] = {
    CargoShipButtonRight,
    CargoShipButtonBottom,
};
size_t HeldButtonArraySize = sizeof(HeldButtonArray) / sizeof(HeldButtonArray[0]);

ToggleButtonGroup CenterToggleGroup = ToggleButtonGroup(ToggleButtonArray, ToggleButtonArraySize, CenterTopButton);
HeldButtonGroup CargoHeldGroup = HeldButtonGroup(HeldButtonArray, HeldButtonArraySize);

ToggleButton MohidToggleButton(14, 6, 7);
int x = 0;
void setup()
{
  CenterToggleGroup.ActiveDefaultButton();
  MohidToggleButton.Enable();
}

void loop()
{
  // x = Serial.parseInt();
  // Serial.println(x);
  // usb_rawhid_recv(0, &x, 1, 1);
  CenterToggleGroup.PollButtons();
  CargoHeldGroup.PollButtons();
  MohidToggleButton.PollButton();
  delay(50);
}
