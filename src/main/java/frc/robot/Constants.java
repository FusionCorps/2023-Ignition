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
  // dt for updating odometry
  public static final double kLooperDt = .01;
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class IntakeConstants {
    // INTAKE MOTOR ID
    public static final int INTAKE_ID = 25;

    public static final double INTAKE_PCT = -0.4;

    public static final double OUTTAKE_VOLTS = 6.3;
    public static final double OUTTAKE_VOLTS_CUBE = 4.2;
  }

  public static class ArmConstants {
    public static final int BASE_ID = 20;
    public static final int WRIST_ID = 21;

    public static final int BASE_FOLLOWER_ID = 22;

    public static final double BASE_kF = 0;
    public static final double BASE_kP = 0.2;
    public static final double BASE_kI = 0;
    public static final double BASE_kD = 0;

    public static final double BASE_MAX_V = 19500; // ticks / 100ms
    public static final double BASE_MAX_A = 48000; // ticks / 100ms / s
    public static final int BASE_CURVE_STR = 2; // smoothness

    // slomo testing
//    public static final double BASE_MAX_V = 2000;
//    public static final double BASE_MAX_A = 4000;
//    public static final int BASE_CURVE_STR = 1;

    public static final double WRIST_kF = 0;
    public static final double WRIST_kP = 0.4;
    public static final double WRIST_kI = 0;
    public static final double WRIST_kD = 0;

    public static final double WRIST_MAX_V = 19500;
    public static final double WRIST_MAX_A = 52000; // could be up to 102400 with good enough intake
    public static final int WRIST_CURVE_STR = 2;

    // slomo testing
//    public static final double WRIST_MAX_V = 2000;
//    public static final double WRIST_MAX_A = 4000;
//    public static final int WRIST_CURVE_STR = 1;

    public static final double BASE_FF = 0;
    public static final double WRIST_FF = 0.0;

    public static final double BASE_GEAR_RATIO = 58.286*3/36*20;
    public static final double WRIST_GEAR_RATIO = 54;

    public static final int BASE_START_POS = 0;
    public static final int WRIST_START_POS = 0;

    public static final int BASE_ERROR_THRESHOLD = 8000;
    public static final int WRIST_ERROR_THRESHOLD = 20000;

    public static final double BASE_SAFETY_THRESHOLD = 45*PI/180/(PI/1024/BASE_GEAR_RATIO);

    public static final int WRIST_STOWED_POS = 0;

    // degrees to motor ticks
    public static final double BASE_CONVERSION_FACTOR = PI/180/(PI/1024/BASE_GEAR_RATIO);
    public static final double WRIST_CONVERSION_FACTOR = PI/180/(PI/1024/WRIST_GEAR_RATIO);

    public static final double MID_BASE_POS = -127*PI/180/(PI/1024/BASE_GEAR_RATIO);
    public static final double MID_WRIST_POS = -90*PI/180/(PI/1024/WRIST_GEAR_RATIO);
    public static final double MID_WRIST_POS_TELE = -95*PI/180/(PI/1024/WRIST_GEAR_RATIO);

    public static final double MID_BASE_POS_CUBE = -15*PI/180/(PI/1024/BASE_GEAR_RATIO);
    public static final double MID_WRIST_POS_CUBE = 55*PI/180/(PI/1024/WRIST_GEAR_RATIO);

    public static final double HIGH_BASE_POS = -131.5*PI/180/(PI/1024/BASE_GEAR_RATIO);
    public static final double HIGH_WRIST_POS = -129*PI/180/(PI/1024/WRIST_GEAR_RATIO);

    public static final double HIGH_BASE_POS_VIKES = -131.5*PI/180/(PI/1024/BASE_GEAR_RATIO);
    public static final double HIGH_WRIST_POS_VIKES = -129*PI/180/(PI/1024/WRIST_GEAR_RATIO);


    public static final double HIGH_WRIST_POS_AUTO = -113.5*PI/180/(PI/1024/WRIST_GEAR_RATIO);

    // testing to get cone very close to high
    public static final double HIGH_BASE_POS_ALT_PREP = -134.5*PI/180/(PI/1024/BASE_GEAR_RATIO);
    public static final double HIGH_BASE_POS_ALT = -122*PI/180/(PI/1024/BASE_GEAR_RATIO);
    public static final double HIGH_WRIST_POS_ALT = -146.5*PI/180/(PI/1024/WRIST_GEAR_RATIO);

//    public static final double HIGH_BASE_POS_ALT = -114*PI/180/(PI/1024/BASE_GEAR_RATIO);
//    public static final double HIGH_WRIST_POS_ALT = -161.5*PI/180/(PI/1024/WRIST_GEAR_RATIO);

//    public static final double HIGH_BASE_POS_ALT = -121*PI/180/(PI/1024/BASE_GEAR_RATIO);
//    public static final double HIGH_WRIST_POS_ALT = -148.5*PI/180/(PI/1024/WRIST_GEAR_RATIO);

    public static final double HIGH_BASE_POS_ALT_FTW = -121.5*PI/180/(PI/1024/BASE_GEAR_RATIO);
    public static final double HIGH_WRIST_POS_ALT_FTW = -148.5*PI/180/(PI/1024/WRIST_GEAR_RATIO);
    public static final double HIGH_BASE_POS_ALT_WACO = -118*PI/180/(PI/1024/BASE_GEAR_RATIO);
    public static final double HIGH_WRIST_POS_ALT_WACO = -152*PI/180/(PI/1024/WRIST_GEAR_RATIO);

    public static final double HIGH_BASE_POS_ALT_AUTO = -120.5*PI/180/(PI/1024/BASE_GEAR_RATIO);
    public static final double HIGH_WRIST_POS_ALT_AUTO = -152*PI/180/(PI/1024/WRIST_GEAR_RATIO);

    public static final double INTAKE_BASE_POS_CONE = 23.5*PI/180/(PI/1024/BASE_GEAR_RATIO);
    public static final double INTAKE_WRIST_POS_CONE = -155*PI/180/(PI/1024/WRIST_GEAR_RATIO);

    public static final double INTAKE_BASE_POS_CUBE = 45.5*PI/180/(PI/1024/BASE_GEAR_RATIO);
    public static final double INTAKE_WRIST_POS_CUBE = -175*PI/180/(PI/1024/WRIST_GEAR_RATIO);

    public static final double CHUTE_BASE_POS = 15*PI/180/(PI/1024/BASE_GEAR_RATIO);
    public static final double CHUTE_WRIST_POS = -58
              *PI/180/(PI/1024/WRIST_GEAR_RATIO);

    public static final double SHELF_BASE_POS = 90*PI/180/(PI/1024/BASE_GEAR_RATIO);
    public static final double SHELF_WRIST_POS = -180*PI/180/(PI/1024/WRIST_GEAR_RATIO);

//    public static final double LOW_BASE_POS_CUBE = INTAKE_BASE_POS_CUBE;
//    public static final double LOW_WRIST_POS_CUBE = 45*PI/180/(PI/1024/WRIST_GEAR_RATIO);

    public static final double LOW_BASE_POS_CUBE = 0;
    public static final double LOW_WRIST_POS_CUBE = 70*PI/180/(PI/1024/WRIST_GEAR_RATIO);

    public static final double FLING_BASE_POS = 0;
    public static final double FLING_INIT_WRIST_POS = -50*PI/180/(PI/1024/WRIST_GEAR_RATIO);
    public static final double FLING_FINAL_WRIST_POS = 70*PI/180/(PI/1024/WRIST_GEAR_RATIO);

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
  public static double TRACK_WIDTH_METERS = 0.4953;
  public static double TRACK_LENGTH_METERS = 0.4953;

//  public static double TRACK_WIDTH_METERS = 0.7112;
//  public static double TRACK_LENGTH_METERS = 0.7112;

  public static double TRACK_WIDTH_METERS_TEST = 0.4953;
  public static double TRACK_LENGTH_METERS_TEST = 0.4953;

  public static double SWERVE_FORWARD_SPEED_MAX = 6.6;
  public static double SWERVE_STRAFE_SPEED_MAX = 6.6;
  public static double SWERVE_ROT_SPEED_MAX = 4.0 / 0.4953 * 0.7112;

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

  public static boolean IS_LOGGING = false;
}
