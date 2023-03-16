package frc.robot.enums;

import frc.robot.RobotContainer;

public enum ElevatorState {
    eStowEmpty(25.0, 25.0), // height from floor
    eStowLoaded(28.0, 28.0), eFloor(18.5, 18.5), eRaised(22.0, 22.0), // For raising the
                                                                      // elevator before
                                                                      // lowering
    eLow(36.0, 26.0), eMid(51.0, 51.0), 
    eHigh(56.0, 64.0),
    ePlayerStation(57.75, 60.5);// was 64

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