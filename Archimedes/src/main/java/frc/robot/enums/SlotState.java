package frc.robot.enums;

import edu.wpi.first.math.geometry.Pose2d;

public enum SlotState {
    ePosition1(new Pose2d()), ePosition2(new Pose2d()), ePosition3(new Pose2d()),
    ePosition4(new Pose2d()), ePosition5(new Pose2d()), ePosition6(new Pose2d()), 
    ePosition7(new Pose2d()), ePosition8(new Pose2d()), ePosition9(new Pose2d());

    private Pose2d mPosition;

    SlotState(Pose2d position) {
        mPosition = position;
    }

    public Pose2d getPosition() {
        return mPosition;
    }

}
