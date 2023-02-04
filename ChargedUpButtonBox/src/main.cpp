#include <Arduino.h>

constexpr int ARCADE_BUTTON_PRESSED = 0;
constexpr int INVALID_BUTTON_ID = 9999;
constexpr int JOYSTICK_BUTTON_PRESSED = 1;
constexpr int JOYSTICK_BUTTON_RELEASED = 0;

typedef struct ButtonInfo {
    uint8_t buttonId;
    uint8_t joyStickId;
    uint8_t ledId;
};

class LED {
  uint8_t m_Id;

  public:
    LED(uint8_t ledId) {
      m_Id = ledId;
      pinMode(m_Id, OUTPUT);
    }

    void Enable() {
      digitalWrite(m_Id, HIGH);
    }

    void Disable() {
      digitalWrite(m_Id, LOW);
    }
};

class Button {
  uint8_t m_Id;
  uint8_t m_JoystickId;
  LED m_Led;

  public:
    Button(ButtonInfo buttonInfo): Button(buttonInfo.buttonId, buttonInfo.ledId, buttonInfo.joyStickId) {}
    
    Button(uint8_t buttonId, uint8_t ledId, uint8_t joystickId): m_Led(LED(ledId)) {
      m_Id = buttonId;
      m_JoystickId = joystickId;
      pinMode(buttonId, INPUT_PULLUP);
    }

    void Enable() {
      m_Led.Enable();
      Joystick.button(m_JoystickId, JOYSTICK_BUTTON_PRESSED);
    }

    void Disable() {
      m_Led.Disable();
      Joystick.button(m_JoystickId, JOYSTICK_BUTTON_RELEASED);
    }

    bool IsEnabled() {
      return digitalRead(m_Id) == ARCADE_BUTTON_PRESSED;
    }

    bool operator==(const Button& other) {
      return this->m_Id == other.m_Id;
    }

    bool operator!=(const Button& other) {
      return this->m_Id != other.m_Id;
    }

};

class ToggleButton: public Button {
  LED m_SecondaryLed;
  bool IsPrimaryActive = true;
  bool isHeld = false;

  uint8_t m_SecondaryJoystickId;

  public:
    ToggleButton(uint8_t buttonId, uint8_t defaultLedId, uint8_t secondaryLedId, 
    uint8_t primaryJoystickId, uint8_t secondaryJoystickId): 
    Button(buttonId, defaultLedId, primaryJoystickId), m_SecondaryLed(secondaryLedId) {
      //Enable();
      m_SecondaryJoystickId = secondaryJoystickId;
    }

    void PollButton() {
      if (IsEnabled() && !isHeld) {
        Toggle();
        isHeld = true;
      } else if (!IsEnabled()) {
        isHeld = false;
      }
    }
    
  private:  
    void Toggle() {
      if (IsPrimaryActive) {
        Disable();
        m_SecondaryLed.Enable();
        Joystick.button(m_SecondaryJoystickId, JOYSTICK_BUTTON_PRESSED);
        IsPrimaryActive = false;
      } else {
        Enable();
        m_SecondaryLed.Disable();
        Joystick.button(m_SecondaryJoystickId, JOYSTICK_BUTTON_RELEASED);
        IsPrimaryActive = true;
      }
    }
};

class ButtonGroup {
  protected:
    Button *m_Buttons;
    size_t m_Count;

  public:
    ButtonGroup(Button* buttons, size_t length) {
      m_Buttons = buttons;
      m_Count = length;
    }

    void EnableAll() {
      for (size_t i = 0; i < m_Count; i++) {
        (m_Buttons + i)->Enable();
      }
    }

    void DisableAll() {
      for (size_t i = 0; i < m_Count; i++) {
        (m_Buttons + i)->Disable();
      }
    }

    virtual void PollButtons() = 0;
};

class ToggleButtonGroup: public ButtonGroup {
  Button m_Active;

  public:

    ToggleButtonGroup(Button* buttons, size_t length, const Button& defaultActive): 
    ButtonGroup(buttons, length), m_Active(defaultActive) {
    }

    void ActiveDefaultButton() {
      m_Active.Enable();
    }

    void PollButtons() {
      if (m_Active.IsEnabled()) return;

      for (size_t i = 0; i < m_Count; i++) {
        if ( (m_Buttons + i)->IsEnabled() ) {
          if (*(m_Buttons + i) == m_Active) continue;
          DisableAll();
          (m_Buttons + i)->Enable();
          m_Active = *(m_Buttons + i);
        }
      }
    }
};

class HeldButtonGroup: public ButtonGroup {
    
  size_t m_Active = INVALID_BUTTON_ID;

  public:

    HeldButtonGroup(Button* buttons, size_t length): ButtonGroup(buttons, length) {}

    void PollButtons() {

      if (m_Active != INVALID_BUTTON_ID  && (m_Buttons + m_Active)->IsEnabled()) return;

      for (size_t i = 0; i < m_Count; i++) {
        if ( (m_Buttons + i)->IsEnabled() ) {
          if (i == m_Active) continue;
          m_Active = i;
          (m_Buttons + m_Active)->Enable();
        } 
        else {
          (m_Buttons + i)->Disable();
          if (i == m_Active) m_Active = INVALID_BUTTON_ID;
        }
      }

    }
};

Button CenterTopButton = Button(8, 23, 1);
Button CenterMiddleButton = Button(5, 20, 2);
Button CenterBottomButton = Button(2, 17, 3);

Button ToggleButtonArray[] = {
  CenterTopButton, 
  CenterMiddleButton, 
  CenterBottomButton,
};
size_t ToggleButtonArraySize = sizeof(ToggleButtonArray) / sizeof(ToggleButtonArray[0]);

Button CargoShipButtonRight = Button(12, 39, 4);
Button CargoShipButtonBottom = Button(11, 38, 5);

Button HeldButtonArray[] = {
  CargoShipButtonRight,
  CargoShipButtonBottom,
};
size_t HeldButtonArraySize = sizeof(HeldButtonArray) / sizeof(HeldButtonArray[0]);

ToggleButtonGroup CenterToggleGroup = ToggleButtonGroup(ToggleButtonArray, ToggleButtonArraySize, CenterTopButton);
HeldButtonGroup CargoHeldGroup = HeldButtonGroup(HeldButtonArray, HeldButtonArraySize);

ToggleButton MohidToggleButton(14, 41, 42, 6, 7);

void setup() {
  CenterToggleGroup.ActiveDefaultButton();
  MohidToggleButton.Enable();
}

void loop() {
  CenterToggleGroup.PollButtons();
  CargoHeldGroup.PollButtons();
  MohidToggleButton.PollButton();
  delay(50);
}

