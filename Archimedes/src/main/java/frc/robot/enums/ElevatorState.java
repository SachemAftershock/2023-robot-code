package frc.robot.enums;

import frc.robot.RobotContainer;

public enum ElevatorState {
    // eStowEmpty(25.0, 25.0), 
    // eStowLoaded(28.0, 28.0), 
    // eClearBumper(30.0, 30.0),
    // eRaised(22.0, 22.0),                                                                                    
    // eLow(22.0, 22.0), 
    // eMid(51.0, 52.5), 
    // eHigh(56.0, 62.0), 
    // ePlayerStation(57.75, 60.0),
    // eUnknown(-1, -1);


    eStowEmpty(0.0, 0.0), 
    eStowLoaded(0.0, 0.0), 
    eClearBumper(30.0, 30.0),
    eRaised(22.0, 22.0),                                                                                    
    eLow(22.0, 22.0), 
    eMid(90.38, 94.95), 
    eHigh(102.67, 118.16), 
    ePlayerStation(107.38, 113.73),
    eUnknown(-1, -1);

    private double mCubeHeight;
    private double mConeHeight;

    ElevatorState(double cubeHeight, double coneHeight) {
        this.mCubeHeight = cubeHeight;
        this.mConeHeight = coneHeight;
    }

    public double getHeight() {
        if (RobotContainer.isCone()) {
            return mConeHeight;
        }
        else {
            return mCubeHeight;
        }
    }
}