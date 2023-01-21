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

  // MOTOR IDS for the SWERVE
  public static int AXIS_FL_ID = 0;
  public static int AXIS_BL_ID = 1;
  public static int AXIS_FR_ID = 3;
  public static int AXIS_BR_ID = 7;

  public static int DRIVE_FL_ID = 8;
  public static int DRIVE_BL_ID = 9;
  public static int DRIVE_FR_ID = 4;
  public static int DRIVE_BR_ID = 5;

  // CANCODER IDS
  public static int CODER_FL_ID = 12;
  public static int CODER_BL_ID = 2;
  public static int CODER_FR_ID = 13;
  public static int CODER_BR_ID = 11;

  // Indexer Output
  public static double INDEXER_TARGET = 0.8;

  // Drivebase Facts
  public static double TRACK_WIDTH_METERS = 0.7112;
  public static double TRACK_LENGTH_METERS = 0.7112;

  public static double SWERVE_FORWARD_SPEED_MAX = 4.2;
  public static double SWERVE_STRAFE_SPEED_MAX = 4.2;
  public static double SWERVE_ROT_SPEED_MAX = 6.0;

  public static double MAX_SPEED = 9.2;

  // PIDs (potentially very wack) (olds can be found in 2022 code)
  public static double AXIS_kF = 0.0;
  public static double AXIS_kP = 0.2;
  public static double AXIS_kI = 0.0;
  public static double AXIS_kD = 0.1;

  // note SDS default 0 0.2 0 0.1

  // PIDs (potentially very wack)
  public static double DRIVE_kF = 0.0;
  public static double DRIVE_kP = 0.07;
  public static double DRIVE_kI = 0.0;
  public static double DRIVE_kD = 0.00;

  public static double STEERING_RATIO = 12.8;
  public static double DRIVING_RATIO = 6.75;

  public static double WHEEL_RADIUS_METERS = 0.0508;

  public static double CHARGE_STATION_BALANCE_ANGLE_GOAL = 3.25;


  public static double CHARGE_STATION_STABILIZE_SECONDS = 1;
}
