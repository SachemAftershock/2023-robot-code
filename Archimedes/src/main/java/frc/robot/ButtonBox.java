package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.enums.ButtonBoxLedInfo.LedPosition;
import java.util.ArrayList;
import frc.robot.enums.ButtonBoxLedInfo.ButtonPosition;

public class ButtonBox extends CommandGenericHID {

    public ButtonBox(int port) {
        super(port);
    }

    private static int cone1Id = ButtonPosition.eDriveTo1.getId();
    private static int cube2Id = ButtonPosition.eDriveTo2.getId();
    private static int cone3Id = ButtonPosition.eDriveTo3.getId();

    private static int cone4Id = ButtonPosition.eDriveTo4.getId();
    private static int cube5Id = ButtonPosition.eDriveTo5.getId();
    private static int cone6Id = ButtonPosition.eDriveTo6.getId();

    private static int cone7Id = ButtonPosition.eDriveTo7.getId();
    private static int cube8Id = ButtonPosition.eDriveTo8.getId();
    private static int cone9Id = ButtonPosition.eDriveTo9.getId();

    private static int cancelId = ButtonPosition.eCancel.getId();
    private static int leftHumanStationId = ButtonPosition.eHumanPlayerLeft.getId();
    private static int rightHumanStationId = ButtonPosition.eHumanPlayerRight.getId();

    private static int cubeToggleId = ButtonPosition.eCubeActive.getId();
    private static int coneToggleId = ButtonPosition.eConeActive.getId();

    private static int ingestIntakeButtonId = ButtonPosition.eIngest.getId();
    private static int ejectIntakeButtonId = ButtonPosition.eEject.getId();

    private static int stowPostionId = ButtonPosition.eStow.getId();
    private static int humanPlayerPostionId = ButtonPosition.ePlayerStation.getId();
    private static int floorPostionButtonId = ButtonPosition.eLow.getId();
    private static int mediumPostionButtonId = ButtonPosition.eMid.getId();
    private static int highPositionButtonId = ButtonPosition.eHigh.getId();

    private static int enableJoystickId = ButtonPosition.eJoystickEnable.getId();

    private boolean mIsJoysticEnabled = false;

    // 1 button toggling Cube or Cone state Button
    public Trigger cubeToggle() {
        return this.button(cubeToggleId);
    }

    public Trigger coneToggle() {
        return this.button(coneToggleId);
    }

    // 2 buttons intake or output buttons
    public Trigger ingestIntake() {
        return this.button(ingestIntakeButtonId);
    }

    public Trigger ejectIntake() {
        return this.button(ejectIntakeButtonId);
    }

    // 5 buttons elevator drop off position Buttons
    public Trigger highPosition() {
        return this.button(highPositionButtonId);
    }

    public Trigger mediumPosition() {
        return this.button(mediumPostionButtonId);
    }

    public Trigger floorPosition() {
        return this.button(floorPostionButtonId);
    }

    public Trigger humanPlayerPostion() {
        return this.button(humanPlayerPostionId);
    }

    public Trigger stowPostion() {
        return this.button(stowPostionId);
    }

    // 12 buttons drive to scoring columns Buttons
    public Trigger cone1() {
        return this.button(cone1Id);
    }

    public Trigger cube2() {
        return this.button(cube2Id);
    }

    public Trigger cone3() {
        return this.button(cone3Id);
    }

    public Trigger cone4() {
        return this.button(cone4Id);
    }

    public Trigger cube5() {
        return this.button(cube5Id);
    }

    public Trigger cone6() {
        return this.button(cone6Id);
    }

    public Trigger cone7() {
        return this.button(cone7Id);
    }

    public Trigger cube8() {
        return this.button(cube8Id);
    }

    public Trigger cone9() {
        return this.button(cone9Id);
    }

    // #region Human Station Buttons
    public Trigger leftHumanStation() {
        return this.button(leftHumanStationId);
    }

    public Trigger rightHumanStation() {
        return this.button(rightHumanStationId);
    }
    // #endregion

    // Joystick buttons
    // public Trigger leftJoystickButton() {
    // return this.button(leftJoystickButtonId);
    // }

    // public Trigger rightJoystickButton() {
    // return this.button(rightJoystickButtonId);
    // }

    // public Trigger upJoystickButton() {
    // return this.button(upJoystickButtonId);
    // }

    // public Trigger downJoystickButton() {
    // return this.button(downJoystickButtonId);
    // }

    public Trigger enableJoystick() {
        return this.button(enableJoystickId);
    }

    public Trigger cancel() {
        return this.button(cancelId);
    }

    public boolean isJoystickEnabled() {
        return mIsJoysticEnabled;
    }

    public void toggleJoystick() {
        mIsJoysticEnabled = !mIsJoysticEnabled;
    }

    public void setJoystickEnabled() {
        mIsJoysticEnabled = true;
        ButtonBoxPublisher.enableLed(LedPosition.eJoystickEnable);
    }

    public void setJoystickDisabled() {
        mIsJoysticEnabled = false;
        ButtonBoxPublisher.disableLed(LedPosition.eJoystickEnable);
    }

    public int[] getPressedButtons() {
        ArrayList<Integer> pressedButtons = new ArrayList<Integer>();
        for (int i = 1; i <= 22; i++) {
            if (this.getHID().getRawButton(i)) {
                pressedButtons.add(i);
            }
        }

        int[] pressedButtonsArray = new int[pressedButtons.size()];
        for (int i = 0; i < pressedButtons.size(); i++) {
            pressedButtonsArray[i] = pressedButtons.get(i);
        }
        return pressedButtonsArray;
    }
}