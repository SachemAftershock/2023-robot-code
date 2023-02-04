#include <Arduino.h>

#define ARCADE_BUTTON_PRESSED 0

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
  bool m_Pressed = false;
  uint8_t m_Id;
  LED m_Led;

  public:
    Button(uint8_t buttonId, uint8_t ledId): m_Led(LED(ledId)) {
      m_Id = buttonId;
      pinMode(buttonId, INPUT_PULLUP);
    }

    void Enable() {
      m_Pressed = true;
      m_Led.Enable();
    }

    void Disable() {
      m_Pressed = false;
      m_Led.Disable();
    }

    bool IsEnabled() {
      return digitalRead(m_Id) == ARCADE_BUTTON_PRESSED;
    }
};

class ButtonGroup {
  Button *m_Buttons;
  size_t m_Count;

  uint8_t m_Active;

  public:
    ButtonGroup(Button* button, size_t length, uint8_t defaultActive) {
      m_Buttons = button;
      m_Count = length;
      m_Active = defaultActive;

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

    void PollButtons() {
      for (size_t i = 0; i < m_Count; i++) {
        if ( ((m_Buttons + i)->IsEnabled()) && (i != m_Active) ) {
          DisableAll();
          (m_Buttons + i)->Enable();
          (m_Buttons + m_Active)->Disable();
          m_Active = i;
          break;
        }
      }
    }
};

Button SuperCoolMohidButton = Button(14, 41);
Button LessCoolMohidButton = Button(5, 20);

Button ButtonArray[] = {SuperCoolMohidButton, LessCoolMohidButton};
ButtonGroup MohidButtonGroup(ButtonArray, sizeof(ButtonArray) / sizeof(ButtonArray[0]), 20);

//size_t buttonLength = sizeof(buttons) / sizeof(buttons[0]);

void setup() {
}

void loop() {
  MohidButtonGroup.PollButtons();
}

