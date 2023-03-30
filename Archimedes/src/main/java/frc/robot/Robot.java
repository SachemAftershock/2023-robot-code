// Copyright (c) FIRST and other WPILib contributors. : )
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.hal.simulation.RoboRioDataJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.SubsystemManager;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class Robot extends TimedRobot {
    private Command mAutonomousCommand;

    private RobotContainer mRobotContainer;
    private AutoSelector mAutoSelector;
    private DriveSubsystem mDrive;

    /**
     * This function is run when the robot is first started up and should be used
     * for any initialization code.
     */
    @Override
    public void robotInit() {
        mRobotContainer = RobotContainer.getInstance();
        mRobotContainer.initialize();

        mDrive = DriveSubsystem.getInstance();

        mAutoSelector = new AutoSelector();

        CameraServer.startAutomaticCapture();

        mRobotContainer.syncLeds();
        DriverStation.silenceJoystickConnectionWarning(true);
    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items
     * like diagnostics that you want ran during disabled, autonomous, teleoperated
     * and test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        mRobotContainer.periodic();
    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
        mAutoSelector.selectAuto();
    }

    /**
     * This autonomous runs the autonomous command selected by your
     * {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        //mRobotContainer.initializeSubsystems();
        System.out.println("Time 1 --> " + Timer.getFPGATimestamp());
        mRobotContainer.initialize();
        System.out.println("Time 2 --> " + Timer.getFPGATimestamp());
        // CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().schedule(mRobotContainer.getAutonomousCommand());
        System.out.println("Time 3 --> " + Timer.getFPGATimestamp());
        //mAutonomousCommand = mRobotContainer.getAutonomousCommand();
        // CommandScheduler.getInstance().schedule(mAutoSelector.getSelectedAutoCommand());
        // if (mAutonomousCommand != null) {
        //     mAutonomousCommand.schedule();
        // }
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        CommandScheduler.getInstance().cancelAll();

        boolean TEST = false;
        if (TEST) {
            mRobotContainer.initialize();
            System.out.println("YOU ARE IN TEST MODE");

            ButtonBoxPublisher.sendMessage("TEST_MODE");
        }

        mRobotContainer.syncLeds();

        if (mAutonomousCommand != null) {
            mAutonomousCommand.cancel();
        }
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {

    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
        mRobotContainer.initialize();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {

        SubsystemManager mSubsystemManager = mRobotContainer.getSubsystemManager();

        if (!mSubsystemManager.checkSystems()) {
            mSubsystemManager.checkSystems();
        }

        mRobotContainer.test();
        mRobotContainer.testPeriodic();
    }

    /** This function is called once when the robot is first started up. */
    @Override
    public void simulationInit() {
    }

    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {
    }
}
