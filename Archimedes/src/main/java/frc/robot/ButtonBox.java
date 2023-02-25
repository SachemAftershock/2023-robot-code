package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ButtonBox extends CommandGenericHID {

    public ButtonBox(int port) {
        super(port);
    }

    private static int ingestIntakeButtonId = 1;
    private static int ejectIntakeButtonId = 2;
    private static int highPositionButtonId = 20;
    private static int mediumPostionButtonId = 21;
    private static int floorPostionButtonId = 5;
    private static int humanPlayerPostionId = 6;
    private static int stowPostionId = 7;
    private static int cone1Id = 8;
    private static int cube2Id = 9;
    private static int cone3Id = 10;

    private static int cone4Id = 11;
    private static int cube5Id = 12;
    private static int cone6Id = 13;

    private static int cone7Id = 14;
    private static int cube8Id = 15;
    private static int cone9Id = 16;

    private static int leftHumanStationId = 17;
    private static int rightHumanStationId = 18;
    private static int cancelId = 19;
    private static int cubeToggleId = 3;
    private static int coneToggleId = 4;

    private static int leftJoystickButtonId = 22;
    private static int rightJoystickButtonId = 23;
    private static int upJoystickButtonId = 24;
    private static int downJoystickButtonId = 25;
    private static int enableJoystickId = 26;

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
    public Trigger leftJoystickButton() {
        return this.button(leftJoystickButtonId);
    }

    public Trigger rightJoystickButton() {
        return this.button(rightJoystickButtonId);
    }

    public Trigger upJoystickButton() {
        return this.button(upJoystickButtonId);
    }

    public Trigger downJoystickButton() {
        return this.button(downJoystickButtonId);
    }

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
}