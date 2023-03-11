package frc.robot.enums;

import frc.robot.RobotContainer;

public enum ElevatorState {
    eStowEmpty(19.0, 19.0), 
    eStowLoaded(28.0,28.0),
    eFloor(18.5, 18.5),
    eRaised(22.0, 22.0), //For raising the elevator before lowering
    eLow(36.0, 36.0), 
    eMid(0, 36.0), 
    eHigh(0, 48), 
    ePlayerStation(0, 0);

    private double mCubeHeight;
    private double mConeHeight;

    ElevatorState(double cubeHeight, double coneHeight) {
        this.mCubeHeight = cubeHeight;
        this.mConeHeight = coneHeight;
    }

    public double getHeight() {
        return RobotContainer.isCone() ? mConeHeight : mCubeHeight;
    }
}