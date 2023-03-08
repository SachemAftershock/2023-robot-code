package frc.robot.enums;

import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;

public enum ArmState {
    eStowEmpty(18, 18), 
    eStowLoaded(0,0),
    eLow(0, 0), 
    eMid(13, 13), 
    eHigh(0, 0), 
    ePlayerStation(0, 0);

    private double mCubeLength;
    private double mConeLength;

    ArmState(double cubeLength, double coneLength) {
        this.mCubeLength = cubeLength;
        this.mConeLength = coneLength;
    }

    public double getLength() {
        return RobotContainer.isCone() ? mConeLength : mCubeLength + ArmConstants.kCubeOffset;
    }
}