package frc.robot;
import frc.robot.Enums.ElevatorPosition;
import frc.robot.Enums.ArmDistance;


public class ElevatorInfo {
    private ElevatorPosition targetPosition;
    private ArmDistance targetIntakePosition;

    public ElevatorInfo(ElevatorPosition targetPosition, ArmDistance targetIntakePosition) {
        this.targetPosition = targetPosition;
        this.targetIntakePosition = targetIntakePosition;
    }
    public ElevatorPosition getElevatorPosition() {
        return targetPosition;
    }
    public ArmDistance getTargetIntakePosition() {
        return targetIntakePosition;
    }
}