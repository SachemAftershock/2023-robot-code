package frc.robot.subsystems;
import frc.robot.enums.ArmDistance;
import frc.robot.enums.ElevatorPosition;


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