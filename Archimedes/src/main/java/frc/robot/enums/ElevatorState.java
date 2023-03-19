package frc.robot.enums;

import frc.robot.RobotContainer;

public enum ElevatorState {
    eStowEmpty(25.0, 25.0), // height from floor
    eStowLoaded(28.0, 28.0), 
    eClearBumper(26.0, 26.0),
    eFloor(22.5, 22.5), eRaised(22.0, 22.0), // For raising the
                                                                      // elevator before
                                                                      // lowering
    eLow(36.0, 26.0), eMid(51.0, 51.0), 
    eHigh(56.0, 62.0),
    ePlayerStation(57.75, 60.0);//was +0.5// was 64

    private double mCubeHeight;
    private double mConeHeight;

    ElevatorState(double cubeHeight, double coneHeight) {
        this.mCubeHeight = cubeHeight;
        this.mConeHeight = coneHeight;
    }

    public double getHeight() {
        if(RobotContainer.isCone()) {
            return mConeHeight;
        } else {
            return mCubeHeight;
        }
    }
}