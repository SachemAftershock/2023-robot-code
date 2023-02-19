package frc.robot.enums;

public enum ElevatorState {
    eStow(0, 0), eLow(0, 0),
    eMid(0, 0), eHigh(0, 0),
    ePlayerStation(0, 0);

    private double mCubeHeight;
    private double mConeHeight;

    ElevatorState(double cubeHeight, double coneHeight) {
        this.mCubeHeight = cubeHeight;
        this.mConeHeight = coneHeight;
    }

    public double getCubeHeight() {
        return mCubeHeight;
    }

    public double getConeHeight() {
        return mConeHeight;
    }

}