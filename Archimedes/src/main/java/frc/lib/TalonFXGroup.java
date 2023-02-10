package frc.lib;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.MathUtil;

/**
 * Class to simutaneously control multiple TalonFX Motor Controllers
 * <p>
 * Created due to unexplained issues with DifferentialDrive WPILib Class
 * 
 * @author Shreyas Prasad
 */
public class TalonFXGroup {
    private TalonFX[] mTalons;
    private boolean mIsInverted;

    /**
     * Constructor for TalonFXGroup
     * 
     * @param talon1 First TalonFX motor
     * @param talons Additional TalonFX motors
     */
    public TalonFXGroup(TalonFX talon1, TalonFX... talons) {
        mTalons = new TalonFX[talons.length + 1];
        mTalons[0] = talon1;
        for(int i=0;i<talons.length;i++) {
            mTalons[i+1] = talons[i];
        }

        mIsInverted = false;
        setNeutralMode(NeutralMode.Coast);
    }

    /**
     * Set Motor Inversion
     * @param inverted Motor inverted
     */
    public void setInverted(boolean inverted) {
        mIsInverted = inverted;
        for(TalonFX talon : mTalons) 
            talon.setInverted(mIsInverted);
    }

    /**
     * Motor Neutral Mode
     * 
     * @param mode Neutral Mode
     */
    public void setNeutralMode(NeutralMode mode) {
        for(TalonFX talon : mTalons)
            talon.setNeutralMode(mode);
    }

    /**
     * Drive all motors
     * <p>
     * Limited to [-1.0, 1.0]
     * @param speed Speed to drive motors
     */
    public void set(double speed) {
        speed = MathUtil.clamp(speed, -1.0, 1.0);

        for(TalonFX talon : mTalons)
            talon.set(ControlMode.PercentOutput, speed);
    }
    
    /**
     * Stop all motors
     */
    public void stop() {
        for(TalonFX talon : mTalons) {
            talon.set(ControlMode.PercentOutput, 0.0);
        }
    }

    public void setRampRate(double value) {
        for(TalonFX talon : mTalons) {
            talon.configOpenloopRamp(value);
        }
    }
    
}
