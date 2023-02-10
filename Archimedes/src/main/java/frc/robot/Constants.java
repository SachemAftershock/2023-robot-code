// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  public static class PortConstants{
    public static final int kSparkMaxPort = 0;
    public static final int kLidarPort = 0;

  }
  public static class ControllerConstants{
    public static final int kPrimaryControllerPort = 0;
  
  }
  public static class IntakeConstants{
    /**public static final double intakeConeSpeed = -0.75;
    public static final double intakeCubeSpeed = 0.75;
    public static final double outputConeSpeed = 0.75;
    public static final double outputCubeSpeed = -0.75;**/

    public static final double kIngestConeSpeed = 0.5;
    public static final double kIngestCubeSpeed = 0.5;
  }
  public static class MotorConstants{
    public static final int kCANSparkMaxID = 3; 
  }
  public static class PIDvalues{
    public static final double[] kPIDvalue = {0, 0, 0};
    public static final double kElevatorEpsilon = 0.0;
  }
  public static class Lidarportvalues{
    public static final int y = 0;
  }
  }
