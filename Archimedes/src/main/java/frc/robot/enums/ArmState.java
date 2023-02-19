package frc.robot.enums;

public enum ArmState {
    eStow(0, 0), eLow(0, 0),
    eMid(0, 0), eHigh(0, 0),
    ePlayerStation(0, 0);

    private double mCubeLength;
    private double mConeLength;

    ArmState(double cubeLength, double coneLength) {
        this.mCubeLength = cubeLength;
        this.mConeLength = coneLength;
    }

    public double getCubeLength() {
        return mCubeLength;
    }

    public double getConeLength() {
        return mConeLength;
    }

}