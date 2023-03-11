package frc.robot.enums;

import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;

public enum ArmState {

    //These are going to be raw lidar values until we learn how to interpolate correctly
    eStowEmpty(17.5,17.5), 
    eStowLoaded(0,0),
    eFloor(15.8, 15.8),
    eLow(14.0, 14.0), 
    eMid(40.0, 40.0), 
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