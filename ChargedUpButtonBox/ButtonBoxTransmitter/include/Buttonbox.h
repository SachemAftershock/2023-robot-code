#ifndef BUTTON_BOX_H_
#define BUTTON_BOX_H_

#include <Arduino.h>

constexpr uint8_t ARCADE_BUTTON_PRESSED = 0;
constexpr uint16_t INVALID_BUTTON_ID = 9999;
constexpr uint8_t JOYSTICK_BUTTON_PRESSED = 1;
constexpr uint8_t JOYSTICK_BUTTON_RELEASED = 0;

struct ButtonInfo
{
  uint8_t buttonId;
  uint8_t joyStickId;

  uint8_t secondaryJoystickId;
};

class Button
{
  uint8_t m_Id;
  uint8_t m_JoystickId;

public:
  Button(ButtonInfo buttonInfo) : Button(buttonInfo.buttonId, buttonInfo.joyStickId) {}

  Button(uint8_t buttonId, uint8_t joystickId)
  {
    m_Id = buttonId;
    m_JoystickId = joystickId;
    pinMode(buttonId, INPUT_PULLUP);
  }

  void Enable()
  {
    Joystick.button(m_JoystickId, JOYSTICK_BUTTON_PRESSED);
  }

  void Disable()
  {
    Joystick.button(m_JoystickId, JOYSTICK_BUTTON_RELEASED);
  }

  bool IsEnabled()
  {
    return digitalRead(m_Id) == ARCADE_BUTTON_PRESSED;
  }

  bool operator==(const Button &other)
  {
    return this->m_Id == other.m_Id;
  }

  bool operator!=(const Button &other)
  {
    return this->m_Id != other.m_Id;
  }
};

class ToggleButton : public Button
{
  bool IsPrimaryActive = true;
  bool isHeld = false;

  uint8_t m_SecondaryJoystickId;

public:
  ToggleButton(ButtonInfo buttonInfo) : ToggleButton(buttonInfo.buttonId, buttonInfo.joyStickId, buttonInfo.secondaryJoystickId) {}
  ToggleButton(uint8_t buttonId,
               uint8_t primaryJoystickId,
               uint8_t secondaryJoystickId) : Button(buttonId, primaryJoystickId)
  {
    // Enable();
    m_SecondaryJoystickId = secondaryJoystickId;
  }

  void PollButton()
  {
    if (IsEnabled() && !isHeld)
    {
      Toggle();
      isHeld = true;
    }
    else if (!IsEnabled())
    {
      isHeld = false;
    }
  }

private:
  void Toggle()
  {
    if (IsPrimaryActive)
    {
      Disable();
      Joystick.button(m_SecondaryJoystickId, JOYSTICK_BUTTON_PRESSED);
      IsPrimaryActive = false;
    }
    else
    {
      Enable();
      Joystick.button(m_SecondaryJoystickId, JOYSTICK_BUTTON_RELEASED);
      IsPrimaryActive = true;
    }
  }
};

class ButtonGroup
{
protected:
  Button *m_Buttons;
  size_t m_Count;

public:
  ButtonGroup(Button *buttons, size_t length)
  {
    m_Buttons = buttons;
    m_Count = length;
  }

  void EnableAll()
  {
    for (size_t i = 0; i < m_Count; i++)
    {
      (m_Buttons + i)->Enable();
    }
  }

  void DisableAll()
  {
    for (size_t i = 0; i < m_Count; i++)
    {
      (m_Buttons + i)->Disable();
    }
  }

  virtual void PollButtons() = 0;
};

class ToggleButtonGroup : public ButtonGroup
{
  Button m_Active;

public:
  ToggleButtonGroup(Button *buttons, size_t length, const Button &defaultActive) : ButtonGroup(buttons, length), m_Active(defaultActive)
  {
  }

  void ActiveDefaultButton()
  {
    m_Active.Enable();
  }

  void PollButtons()
  {
    if (m_Active.IsEnabled())
      return;

    for (size_t i = 0; i < m_Count; i++)
    {
      if ((m_Buttons + i)->IsEnabled())
      {
        if (*(m_Buttons + i) == m_Active)
          continue;
        DisableAll();
        (m_Buttons + i)->Enable();
        m_Active = *(m_Buttons + i);
      }
    }
  }
};

class HeldButtonGroup : public ButtonGroup
{

  size_t m_Active = INVALID_BUTTON_ID;

public:
  HeldButtonGroup(Button *buttons, size_t length) : ButtonGroup(buttons, length) {}

  void PollButtons()
  {

    if (m_Active != INVALID_BUTTON_ID && (m_Buttons + m_Active)->IsEnabled())
      return;

    for (size_t i = 0; i < m_Count; i++)
    {
      if ((m_Buttons + i)->IsEnabled())
      {
        if (i == m_Active)
          continue;
        m_Active = i;
        (m_Buttons + m_Active)->Enable();
      }
      else
      {
        (m_Buttons + i)->Disable();
        if (i == m_Active)
          m_Active = INVALID_BUTTON_ID;
      }
    }
  }
};

#endif /* BUTTON_BOX_H_ */