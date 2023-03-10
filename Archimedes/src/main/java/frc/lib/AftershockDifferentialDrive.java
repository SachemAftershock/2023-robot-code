package frc.lib;

import com.ctre.phoenix.motorcontrol.NeutralMode;

/**
 * Implementation of Differential Drive
 * <p>
 * Created due to unexplained issues with DifferentialDrive WPILib Class
 * 
 * @author Shreyas Prasad
 */
public class AftershockDifferentialDrive {
    private TalonFXGroup mLeft, mRight;
    private boolean mInvertRight;
    private NeutralMode mNeutralMode;

    private double mXSpeedDeadband = 0.02;
    private double mZRotationDeadband = 0.02;

    /**
     * Constructor for Aftershock Differential Drive
     * 
     * @param left Left Controlling Motors, as TalonFXGroup
     * @param right Right Controlling Motors, as TalonFXGroup
     */
    public AftershockDifferentialDrive(TalonFXGroup left, TalonFXGroup right) {
        mLeft = left;
        mRight = right;

        mInvertRight = false;
    }

    /**
     * Set Neutral Mode of all motors
     * <p>
     * Default is Coast
     * 
     * @param mode Neutral mode
     */
    public void setNeutralMode(NeutralMode mode) {
        mNeutralMode = mode;
        mLeft.setNeutralMode(mNeutralMode);
        mRight.setNeutralMode(mNeutralMode);
    }

    public void setRampRate(double value) {
        mLeft.setRampRate(value);
        mRight.setRampRate(value);
    }

    /**
     * Inverts right set of motors
     * <p>
     * Due to right side being reversed
     * @param invert Inverted State
     */
    public void invertRight(boolean invert) {
        mInvertRight = invert;
        mRight.setInverted(mInvertRight);
    }

    /**
     * Sets the deadband to ignore values below
     * 
     * @param xSpeedDeadband Deadband for input forward speed
     * @param zRotationDeadband Deadband for input rotational speed
     */
    public void setDeadband(double xSpeedDeadband, double zRotationDeadband) {
        mXSpeedDeadband = xSpeedDeadband;
        mZRotationDeadband = zRotationDeadband;
    }

    /**
     * Arcade Drive Implementation, default Square inputs
     * 
     * @param xSpeed Forward Input Speed
     * @param zRotation Rotational Input Speed
     */
    public void arcadeDrive(double xSpeed, double zRotation) {
        arcadeDrive(xSpeed, zRotation, true);
    }

    /**
     * Arcade Drive Implementation
     * 
     * @param xSpeed Forward Input Speed
     * @param zRotation Rotational Input Speed
     * @param squareInputs Square Inputs to increase fine control
     */
    public void arcadeDrive(double xSpeed, double zRotation, boolean squareInputs) {
        xSpeed = Util.deadband(xSpeed, mXSpeedDeadband);
        zRotation = Util.deadband(zRotation, mZRotationDeadband);

        // Square the inputs (while preserving the sign) to increase fine control
        // while permitting full power.
        if (squareInputs) {
            xSpeed = Math.copySign(xSpeed * xSpeed, xSpeed);
            zRotation = Math.copySign(zRotation * zRotation, zRotation);
        }

        double leftMotorOutput;
        double rightMotorOutput;

        double maxInput = Math.copySign(Math.max(Math.abs(xSpeed), Math.abs(zRotation)), xSpeed);

        if (xSpeed >= 0.0) {
            // First quadrant, else second quadrant
            if (zRotation >= 0.0) {
                leftMotorOutput = maxInput;
                rightMotorOutput = xSpeed - zRotation;
            } else {
                leftMotorOutput = xSpeed + zRotation;
                rightMotorOutput = maxInput;
            }
          } else {
            // Third quadrant, else fourth quadrant
            if (zRotation >= 0.0) {
                leftMotorOutput = xSpeed + zRotation;
                rightMotorOutput = maxInput;
            } else {
                leftMotorOutput = maxInput;
                rightMotorOutput = xSpeed - zRotation;
            }
        }
        //System.out.println("left --> " + leftMotorOutput + " right --> " + rightMotorOutput);
        // if(Math.abs(leftMotorOutput - rightMotorOutput) > 0.0) {
        //     System.out.println("ERROR : motor offset --> " + Math.abs(leftMotorOutput - rightMotorOutput));
        // }
        set(leftMotorOutput, rightMotorOutput);
    }

    /**
     * Tank Drive
     * 
     * @param leftSpeed Speed to drive Left Motors
     * @param rightSpeed Speed to drive Right Motors
     */
    public void set(double leftSpeed, double rightSpeed) {
        if(leftSpeed > 1.0) {
            leftSpeed = 1.0;
        } 
        if(rightSpeed > 1.0) {
            rightSpeed = 1.0;
        }
        if(leftSpeed < -1.0) {
            leftSpeed = -1.0;
        }
        if(rightSpeed < -1.0) {
            rightSpeed = -1.0;
        }
        mLeft.set(leftSpeed);
        mRight.set(rightSpeed);
    }

    /**
     * Stop All Drive Motors
     */
    public void stop() {
        set(0,0);
    }
}
