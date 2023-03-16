package frc.robot.enums;

import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;

public enum ArmState {

    eUnknown(-1, -1),
    // These are going to be raw lidar values until we learn how to interpolate
    // correctly
    eStowEmpty(17.5, 17.5), eStowLoaded(0, 0), eFloor(15.8, 15.8), eLow(16.22, 14.0), eMid(15.3, 15.3),
    eHigh(13.0, 13.0), ePlayerStation(16.0, 16.0);// (17.0, 15.5/*16.5*/);

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