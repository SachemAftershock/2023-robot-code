package frc.robot.enums;

public enum SuperState {
    eStow(ElevatorState.eStow, ArmState.eStow),
    eLow(ElevatorState.eLow, ArmState.eLow),
    eMid(ElevatorState.eMid, ArmState.eMid), 
    eHigh(ElevatorState.eHigh, ArmState.eHigh),
    ePlayerStation(ElevatorState.ePlayerStation, ArmState.ePlayerStation);
    

    private ElevatorState mElevatorState;
    private ArmState mArmState;

    SuperState(ElevatorState elevatorState, ArmState armState) {
        mElevatorState = elevatorState;
        mArmState = armState;
    }
    
    public ElevatorState getElevatorState() {
        return mElevatorState;
    }

    public ArmState getArmState() {
        return mArmState;
    }

}
