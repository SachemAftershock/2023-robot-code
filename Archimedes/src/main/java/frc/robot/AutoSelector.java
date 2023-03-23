package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.auto.fieldOrientedTrajectoryAuto.*;
import frc.robot.auto.linearAuto.AutoPathConeLinear;
import frc.robot.auto.linearAuto.AutoPathCubeLinear;
import frc.robot.commands.drive.LinearDriveCommand;
import frc.robot.commands.drive.RotateDriveCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;



/**
 * Class to select desired Autonomous Configuration at Startup
 * 
 * @author Shreyas Prasad
 */
public class AutoSelector {

    /**
     * Autonomous Paths that map to a Command Sequence
     * 
     * @author Shreyas Prasad
     */
    enum AutoPath {
        eNothing, ePath0, eCone, ePath0NC, ePath1, eCube, ePath1NC, ePath2, ePath2NC,
         ePath3, ePath3NC, ePath4, ePath4NC, ePath5, ePath5NC,
         eCubeTaxi, eConeTaxi;
    }

    private AutoPath mSelectedAutoScenario, mPrevAutoScenario;

    private SendableChooser<AutoPath> mAutoChooser;

    /**
     * Constructor for AutoSelector Class
     */
    public AutoSelector() {
        mPrevAutoScenario = null;

        mAutoChooser = new SendableChooser<>();
        mAutoChooser.setDefaultOption("No Path", AutoPath.eNothing);
        mAutoChooser.addOption("Path 0" , AutoPath.ePath0);
        mAutoChooser.addOption("Cone Path" , AutoPath.eCone);
        mAutoChooser.addOption("Path 0 No Charge" , AutoPath.ePath0NC);
        mAutoChooser.addOption("Path 1", AutoPath.ePath1);
        mAutoChooser.addOption("Cube Path", AutoPath.eCube);
        mAutoChooser.addOption("Path 1 No Charge", AutoPath.ePath1NC);
        mAutoChooser.addOption("Path 2", AutoPath.ePath2);
        mAutoChooser.addOption("Path 2 No Charge", AutoPath.ePath2NC);
        mAutoChooser.addOption("Path 3", AutoPath.ePath3);
        mAutoChooser.addOption("Path 3 No Charge", AutoPath.ePath3NC);
        mAutoChooser.addOption("Path 4", AutoPath.ePath4);
        mAutoChooser.addOption("Path 4 No Charge", AutoPath.ePath4NC);
        mAutoChooser.addOption("Path 5", AutoPath.ePath5);
        mAutoChooser.addOption("Path 5 No Charge", AutoPath.ePath5NC);
        mAutoChooser.addOption("Cube Taxi", AutoPath.eCubeTaxi);
        mAutoChooser.addOption("Cone Taxi", AutoPath.eConeTaxi);
        SmartDashboard.putData("Auto Path", mAutoChooser);
    }

    /**
     * Allows Operators to select one of several designed Autonomous Routines via SmartDashboard/Shuffleboard
     */
    public void selectAuto() {
        mSelectedAutoScenario = mAutoChooser.getSelected();
        if(mPrevAutoScenario != mSelectedAutoScenario) {
            System.out.println("Changing Auto Path: " + mSelectedAutoScenario.name());
        }
        mPrevAutoScenario = mSelectedAutoScenario;
    }

    /**
     * Decodes AutoPath Enum to a Command Sequence
     * 
     * @return Command Sequence for Autonomous
     */
    public Command getSelectedAutoCommand() {
        switch(mSelectedAutoScenario) {
            case ePath0:
                return new AutoPathZero(DriveSubsystem.getInstance(), ElevatorSubsystem.getInstance(), ArmSubsystem.getInstance(), IntakeSubsystem.getInstance());
            case ePath1:
                return new AutoPathOne(DriveSubsystem.getInstance(), ElevatorSubsystem.getInstance(), ArmSubsystem.getInstance(), IntakeSubsystem.getInstance());
            case ePath2:
                return new AutoPathTwo(DriveSubsystem.getInstance(), ElevatorSubsystem.getInstance(), ArmSubsystem.getInstance(), IntakeSubsystem.getInstance());
            case ePath3:
                return new AutoPathThree(DriveSubsystem.getInstance(), ElevatorSubsystem.getInstance(), ArmSubsystem.getInstance(), IntakeSubsystem.getInstance());
            case ePath4:
                return new AutoPathFour(DriveSubsystem.getInstance(), ElevatorSubsystem.getInstance(), ArmSubsystem.getInstance(), IntakeSubsystem.getInstance());
            case ePath5:
                return new AutoPathFive(DriveSubsystem.getInstance(), ElevatorSubsystem.getInstance(), ArmSubsystem.getInstance(), IntakeSubsystem.getInstance());
            case ePath0NC:
                return new AutoPathZeroNoCharge(DriveSubsystem.getInstance(), ElevatorSubsystem.getInstance(), ArmSubsystem.getInstance(), IntakeSubsystem.getInstance());
            case ePath1NC:
                return new AutoPathOneNoCharge(DriveSubsystem.getInstance(), ElevatorSubsystem.getInstance(), ArmSubsystem.getInstance(), IntakeSubsystem.getInstance());
            case ePath2NC:
                return new AutoPathTwoNoCharge(DriveSubsystem.getInstance(), ElevatorSubsystem.getInstance(), ArmSubsystem.getInstance(), IntakeSubsystem.getInstance());
            case ePath3NC:
                return new AutoPathThreeNoCharge(DriveSubsystem.getInstance(), ElevatorSubsystem.getInstance(), ArmSubsystem.getInstance(), IntakeSubsystem.getInstance());
            case ePath4NC:
                return new AutoPathFourNoCharge(DriveSubsystem.getInstance(), ElevatorSubsystem.getInstance(), ArmSubsystem.getInstance(), IntakeSubsystem.getInstance());
            case ePath5NC:
                return new AutoPathFiveNoCharge(DriveSubsystem.getInstance(), ElevatorSubsystem.getInstance(), ArmSubsystem.getInstance(), IntakeSubsystem.getInstance());
            case eCone:
                return new AutoPathCone(DriveSubsystem.getInstance(), ElevatorSubsystem.getInstance(), ArmSubsystem.getInstance(), IntakeSubsystem.getInstance());
            case eCube:
                return new AutoPathCube(DriveSubsystem.getInstance(), ElevatorSubsystem.getInstance(), ArmSubsystem.getInstance(), IntakeSubsystem.getInstance());
            case eCubeTaxi:
                return new AutoPathCubeLinear(DriveSubsystem.getInstance(), ElevatorSubsystem.getInstance(), ArmSubsystem.getInstance(), IntakeSubsystem.getInstance());
            case eConeTaxi:
                return new AutoPathConeLinear(DriveSubsystem.getInstance(), ElevatorSubsystem.getInstance(), ArmSubsystem.getInstance(), IntakeSubsystem.getInstance());
            case eNothing:
            default: 
                return null;
        }
    }
}