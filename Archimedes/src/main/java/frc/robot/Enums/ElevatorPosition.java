package frc.robot.enums;

import frc.robot.subsystems.ElevatorSubsystem;

public enum ElevatorPosition {
    STOW(0,0,0,0),
    FLOOR(0,37,0,37),
    MID(0,86,0,132),
    HUMAN(0,0,0,0), 
    HIGH(0,160,0,183);


    public final int elevatorHeightCube;
    public final int armExtensionCube;
    public final int elevatorHeightCone;
    public final int armExtensionCone;

    private ElevatorPosition(int elevatorDistanceCube, int armDistanceCube, int elevatorDistanceCone, int armDistanceCone ) {
        elevatorHeightCube = elevatorDistanceCube;
        armExtensionCube = armDistanceCube;
        elevatorHeightCone = elevatorDistanceCone;
        armExtensionCone = armDistanceCone;
    }
    

}