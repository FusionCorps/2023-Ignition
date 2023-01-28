// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static java.lang.Math.PI;

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

  public static class ArmConstants {
    public static final int BASE_ID = 20;
    public static final int WRIST_ID = 21;

    public static final int BASE_FOLLOWER_ID = 22;

    public static final double BASE_kF = 0;
    public static final double BASE_kP = 0.1;
    public static final double BASE_kI = 0;
    public static final double BASE_kD = 0;

    public static final double BASE_MAX_V = 256000;
    public static final double BASE_MAX_A = 19200;
    public static final int BASE_CURVE_STR = 1;

    public static final double WRIST_kF = 0;
    public static final double WRIST_kP = 0.07;
    public static final double WRIST_kI = 0;
    public static final double WRIST_kD = 0;

    public static final double WRIST_MAX_V = 128000;
    public static final double WRIST_MAX_A = 51200;
    public static final int WRIST_CURVE_STR = 1;

    public static final double BASE_FF = 0;
    public static final double WRIST_FF = 0.0;

    public static final double BASE_GEAR_RATIO = 58.286*3;
    public static final double WRIST_GEAR_RATIO = 60;

    public static final int BASE_START_POS = 0;
    public static final int WRIST_START_POS = 0;

    public static final int BASE_ERROR_THRESHOLD = 500;
    public static final int WRIST_STOWED_POS = 0;

    public static final double MID_BASE_POS = -127*PI/180/(PI/1024/BASE_GEAR_RATIO);
    public static final double MID_WRIST_POS = -90*PI/180/(PI/1024/WRIST_GEAR_RATIO);

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

  public static double SWERVE_FORWARD_SPEED_MAX = 5.2;
  public static double SWERVE_STRAFE_SPEED_MAX = 5.2;
  public static double SWERVE_ROT_SPEED_MAX = 6.0;

  public static double MAX_SPEED = 9.2;

  // PIDs (potentially very wack) (olds can be found in 2022 code)
  public static double AXIS_kF = 0.0;
  public static double AXIS_kP = 0.15;
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

  // AutoBalance alt constants
  public static double CHARGE_STATION_BALANCE_ANGLE_GOAL = 3.25;

  public static double AUTON_DRIVE_kP = 0.6;

  public static double CHARGE_STATION_STABILIZE_SECONDS = .1;
}
