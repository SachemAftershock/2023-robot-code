package frc.robot.enums;

import frc.robot.RobotContainer;

public enum ElevatorState {
    eStowEmpty(19.0, 19.0), 
    eStowLoaded(0,0),
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