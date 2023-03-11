package frc.robot.enums;

import frc.robot.RobotContainer;

public enum ElevatorState {
    eStowEmpty(20.0, 20.0), eStowLoaded(28.0, 28.0), eFloor(18.5, 18.5), eRaised(22.0, 22.0), // For raising the elevator before lowering
    eLow(36.0, 36.0), eMid(51.0, 51.0), eHigh(64.0, 64.0), ePlayerStation(60.0, 60.0);

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