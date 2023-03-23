package frc.robot.enums;

import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;

public enum ArmState {

    eUnknown(-1, -1),
    // These are going to be raw lidar values until we learn how to interpolate
    // correctly
    //eStowEmpty was 17.5;
    eStowEmpty(17.5, 17.5), eStowLoaded(0, 0), eFloor(16.2, 16.2), eLow(15.9, 15.9), eMid(16.4, 15.3),
    eHigh(12.75, 13.4), ePlayerStation(16.9, 16.9);// (17.0, 15.5/*16.5*/);

    //high was 13.17 in cone
    private double mCubeLength; 
    private double mConeLength;

    ArmState(double coneLength, double cubeLength) {
        this.mConeLength = coneLength;
        this.mCubeLength = cubeLength;
    }

    public double getLength() {
        // System.out.println("Cone --> " + RobotContainer.isCone());
        // return RobotContainer.isCone() ? mConeLength : mCubeLength +
        // ArmConstants.kCubeOffset;

        if (RobotContainer.isCone()) {
            return mConeLength;
        }
        else {
            return mCubeLength;
        }
    }
}