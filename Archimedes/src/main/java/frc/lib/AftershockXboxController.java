package frc.lib;

import edu.wpi.first.wpilibj.XboxController;

/**
 * Wrapper for XboxControllers
 * 
 * @author Shreyas Prasad
 */
public class AftershockXboxController extends XboxController {

    private final double kJoystickDeadbandToleranceX = 0.15;
    private final double kJoystickDeadbandToleranceY = 0.15;
    private final double kTriggerDeadbandTolerance =  0.05;

    private LatchedBoolean mLeftTriggerPressed;
    private LatchedBoolean mRightTriggerPressed;
    private LatchedBoolean mRightTriggerReleased;
    private LatchedBoolean mLeftTriggerReleased;

    /**
     * Constructor for Afterhock Wrapper Xbox Controllers
     * 
     * @param port Driver Station port the controller is connected to
     */
    public AftershockXboxController(final int port) {
        super(port);
        mLeftTriggerPressed = new LatchedBoolean();
        mRightTriggerPressed = new LatchedBoolean();
        mRightTriggerReleased = new LatchedBoolean();
        mLeftTriggerReleased = new LatchedBoolean();
    }

    public double getkJoystickDeadbandToleranceX() {
        return kJoystickDeadbandToleranceX;
    }

    public double getkJoystickDeadbandToleranceY() {
        return kJoystickDeadbandToleranceY;
    }

    /**
     * Gets X axis value on left Joystick with deadband applied
     * 
     * @return X Axis Value of Selected Joystick with deadband applied
     */
    public double getLeftDeadbandX() {
        return Util.deadband(this.getLeftX(), kJoystickDeadbandToleranceX);
    }

    /**
     * Gets Y axis value on right Joystick with deadband applied
     * 
     * @return Y Axis Value of Selected Joystick with deadband applied
     */
    public double getRightDeadbandY() {
        return Util.deadband(this.getRightY(), kJoystickDeadbandToleranceY);
    }

    /**
     * Gets Y axis value on left Joystick with deadband applied
     * 
     * @return Y Axis Value of Selected Joystick with deadband applied
     */
    public double getLeftDeadbandY() {
        return Util.deadband(this.getLeftY(), kJoystickDeadbandToleranceY);
    }

    /**
     * Gets X axis value on right Joystick with deadband applied
     * 
     * @return X Axis Value of Selected Joystick with deadband applied
     */
    public double getRightDeadbandX() {
        return Util.deadband(this.getRightX(), kJoystickDeadbandToleranceX);
    }

    /**
     * Whether the Left Trigger was pressed since the last check
     *
     * @return Whether the Trigger was pressed since the last check
     */
    public boolean getLeftTriggerPressed(){
        return mLeftTriggerPressed.update(getLeftTriggerAxis() > kTriggerDeadbandTolerance);
    }
    
    /**
     * Whether the Right Trigger was pressed since the last check
     *
     * @return Whether the Trigger was pressed since the last check
     */
    public boolean getRightTriggerPressed(){
        return mRightTriggerPressed.update(getRightTriggerAxis() > kTriggerDeadbandTolerance);
    }

    public boolean getLeftTriggerReleased(){
        return mLeftTriggerReleased.update(getRightTriggerAxis() < kTriggerDeadbandTolerance);
    }

    public boolean getRightTriggerReleased(){
        return mRightTriggerReleased.update(getRightTriggerAxis() < kTriggerDeadbandTolerance);
    }

    public boolean getLeftTriggerHeld() {
        return getLeftTriggerAxis() > kTriggerDeadbandTolerance;
    }
    public boolean getRightTriggerHeld() {
        return getRightTriggerAxis() > kTriggerDeadbandTolerance;
    }

    /**
     * Gets if D-Pad is currently being pressed
     * 
     * @return whether D-Pad value is not -1
     */
    public boolean getDPadPressed() {
        return this.getPOV() != -1;
    }

    /**
     * Gets the currently pressed angle on the D-Pad
     * 
     * @return degree measure of the D-Pad Button Pressed (-1 if not pressed)
     */
    public int getDPadAngle() {
        return this.getPOV();
    }

    /**
     * Gets if Up on the D-Pad is pressed
     * 
     * @return whether the D-Pad angle is 0deg
     */
    public boolean getDPadUp() {
        return this.getPOV() == DPadDirection.eUp.getAngle();
    }

    /**
     * Gets if Up-Right on the D-Pad is pressed
     * 
     * @return whether the D-Pad angle is 45deg
     */
    public boolean getDPadUpRight() {
        return this.getPOV() == DPadDirection.eUpRight.getAngle();
    }

    /**
     * Gets if Right on the D-Pad is pressed
     * 
     * @return whether the D-Pad angle is 90deg
     */
    public boolean getDPadRight() {
        return this.getPOV() == DPadDirection.eRight.getAngle();
    }

    private boolean pressed = false;
    public boolean getDPadRightPressed(){
        if(getDPadRight()){
            if(pressed){
                pressed = false;
            }
            else{
                pressed = true;
            }
        }
        return pressed;
    }

    /**
     * Gets if Down-Right on the D-Pad is pressed
     * 
     * @return whether the D-Pad angle is 135deg
     */
    public boolean getDPadDownRight() {
        return this.getPOV() == DPadDirection.eDownRight.getAngle();
    }

    /**
     * Gets if Down on the D-Pad is pressed
     * 
     * @return whether the D-Pad angle is 180deg
     */
    public boolean getDPadDown() {
        return this.getPOV() == DPadDirection.eDown.getAngle();
    }

    /**
     * Gets if Down-Left on the D-Pad is pressed
     * 
     * @return whether the D-Pad angle is 225deg
     */
    public boolean getDPadDownLeft() {
        return this.getPOV() == DPadDirection.eDownLeft.getAngle();
    }

    /**
     * Gets if Left on the D-Pad is pressed
     * 
     * @return whether the D-Pad angle is 270deg
     */
    public boolean getDPadLeft() {
        return this.getPOV() == DPadDirection.eLeft.getAngle();
    }

    /**
     * Gets if Up-Left on the D-Pad is pressed
     * 
     * @return whether the D-Pad angle is 315deg
     */
    public boolean getDPadUpLeft() {
        return this.getPOV() == DPadDirection.eUpLeft.getAngle();
    }

    /**
     * Lookup Table for matching direction of D-Pad on Xbox Controller pressed to an angle measure
     * 
     * @author Shreyas Prasad
     */
    private enum DPadDirection {
        eUp(0), eUpRight(45), eRight(90), eDownRight(135), eDown(180),
        eDownLeft(225), eLeft(270), eUpLeft(315);

        private final int angle;

        private DPadDirection(int angle) {
            this.angle = angle;
        }

        /**
         * Gets angle for D-Pad Direction Pressed
         * 
         * @return angle [0,360) corresponding to the appropriate 45deg interval on the D-Pad
         */
        private int getAngle() {
            return this.angle;
        }
    }
}