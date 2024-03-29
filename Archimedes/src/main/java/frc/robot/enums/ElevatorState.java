package frc.robot.enums;

import frc.robot.RobotContainer;

public enum ElevatorState {
    eStowEmpty(26.0, 26.0),
    eStowLoaded(28.0, 28.0), 
    eClearBumper(30.0, 30.0),
    //Raised(22.0, 22.0),                                                                                    
    eLow(24.5, 24.5), 
    //eLow(30.0, 30.0), 
    //eMid(49.0, 50.5 - 5.0),
    //values were 51, and 52.5
    eMid(45, 45), 
    eHigh(69.0, 62.0), 
    //ePlayerStation(57.75, 60.0),
    ePlayerStation(59.0, 61.5 + 1.5),
    eUnknown(-1, -1);


    // eStowEmpty(1.0, 1.0), 
    // eStowLoaded(0.0, 0.0), 
    // eClearBumper(12.15, 12.15),
    // eRaised(22.0, 22.0),                                                                                    
    // eLow(-3.0, -3.0), 
    // eMid(54.0, 94.95), 
    // eHigh(102.67, 118.16), 
    // ePlayerStation(98.4, 98.4),
    // eUnknown(-1, -1);

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