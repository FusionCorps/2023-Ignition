// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.arm.ArmToPosition;
import frc.robot.commands.arm.ManageArm;
import frc.robot.commands.arm.RelaxArm;
import frc.robot.commands.chassis.ChassisDriveFC;
import frc.robot.commands.chassis.ChassisDriveFCFlickStick;
import frc.robot.commands.chassis.ChassisDriveToNearestTarget;
import frc.robot.commands.intake.RunVoltsTime;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static frc.robot.Constants.ArmConstants.*;
import static frc.robot.Constants.IntakeConstants.INTAKE_PCT;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Chassis m_chassis = new Chassis();
  private final Cameras m_cameras = new Cameras();
  private final ShooterTest m_shooter = new ShooterTest();
  private final Arm m_arm = new Arm();
  private final Intake mIntake = new Intake();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  public static CommandXboxController m_controller =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  public Command autoOne;
  public Command twoPieceLoadSide;
  public Command threePieceLoadSide;

  public Command relaxArm;

  private Leds leds = new Leds();


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    m_chassis.comboFR.zero();
    m_chassis.comboBR.zero();
    m_chassis.comboFL.zero();
    m_chassis.comboBL.zero();

    m_chassis.setDefaultCommand(new ChassisDriveFC(m_chassis));
    m_arm.setDefaultCommand(new ManageArm(m_arm));

    PathPlannerTrajectory examplePath = PathPlanner.loadPath("test_line", new PathConstraints(4, 3));
    autoOne = m_chassis.followTrajectoryCommand(examplePath, true);

    PathPlannerTrajectory twoPieceLoadSideA = PathPlanner.loadPath("1+1_path1", new PathConstraints(4, 3));
//    this.twoPieceLoadSide = new SequentialCommandGroup(m_chassis.runOnce(() -> {m_chassis.setGyroAngle(0);}),
//            m_chassis.followTrajectoryCommand(twoPieceLoadSide, true));
    PathPlannerTrajectory twoPieceLoadSideB = PathPlanner.loadPath("1+1_path2", new PathConstraints(4, 3));


    
 PathPlannerTrajectory threePieceLoadSideA = PathPlanner.loadPath("1+1_path1", new PathConstraints(4, 3));
 PathPlannerTrajectory threePieceLoadSideB = PathPlanner.loadPath("1+1_path2", new PathConstraints(4, 3));
 PathPlannerTrajectory threePieceLoadSideC = PathPlanner.loadPath("3 Piece Third Intake", new PathConstraints(4, 3));
 PathPlannerTrajectory threePieceLoadSideD = PathPlanner.loadPath("3 Piece Third Placement",new PathConstraints(4, 3));
    
    twoPieceLoadSide = new SequentialCommandGroup(
            m_chassis.runOnce(() -> {m_chassis.setGyroAngle(0.0);}),
            new ArmToPosition(m_arm, HIGH_BASE_POS, HIGH_WRIST_POS),
            new RunVoltsTime(mIntake, 9.0, 0.5),
//            new ArmToPosition(m_arm, 0, 0),
            new ArmToPosition(m_arm, 0, 0),
            new ParallelCommandGroup(new ArmToPosition(m_arm, INTAKE_BASE_POS_CONE, INTAKE_WRIST_POS_CONE),
                    m_chassis.followTrajectoryCommand(twoPieceLoadSideA, true),
                    new RunVoltsTime(mIntake, -9.0, twoPieceLoadSideA.getTotalTimeSeconds())),
            new ArmToPosition(m_arm, 0, 0),
            m_chassis.followTrajectoryCommand(twoPieceLoadSideB, false),
            new ChassisDriveToNearestTarget(m_chassis, m_cameras, 1.0),
            new ArmToPosition(m_arm, HIGH_BASE_POS, HIGH_WRIST_POS),
            new RunVoltsTime(mIntake, 9.0, 0.5)
         );

    threePieceLoadSide = new SequentialCommandGroup(
          m_chassis.runOnce(() -> {m_chassis.setGyroAngle(0.0);}),
          new ArmToPosition(m_arm, HIGH_BASE_POS, HIGH_WRIST_POS),
          new RunVoltsTime(mIntake, 9.0, 0.5),
    //            new ArmToPosition(m_arm, 0, 0),
          new ArmToPosition(m_arm, INTAKE_BASE_POS_CONE, INTAKE_WRIST_POS_CONE),
          new ParallelCommandGroup(m_chassis.followTrajectoryCommand(twoPieceLoadSideA, true),
                new RunVoltsTime(mIntake, -9.0, twoPieceLoadSideA.getTotalTimeSeconds())),
          new ArmToPosition(m_arm, 0, 0),
          m_chassis.followTrajectoryCommand(twoPieceLoadSideB, false),
          new ArmToPosition(m_arm, HIGH_BASE_POS, HIGH_WRIST_POS),
          new RunVoltsTime(mIntake, 9, 0.5),
          new ArmToPosition(m_arm, INTAKE_BASE_POS_CUBE, INTAKE_WRIST_POS_CUBE),
          new ParallelCommandGroup(m_chassis.followTrajectoryCommand(threePieceLoadSideC, false),
            new RunVoltsTime(mIntake, -4, threePieceLoadSideC.getTotalTimeSeconds())),
          new ArmToPosition(m_arm, 0, 0),
          m_chassis.followTrajectoryCommand(threePieceLoadSideD, false),
          new ArmToPosition(m_arm, HIGH_BASE_POS, HIGH_WRIST_POS),
          new RunVoltsTime(mIntake, 9, 0.5) );
    

    relaxArm = new RelaxArm(m_arm);

  }

  public void ledPeriodic(boolean isCube, boolean isEnabled) {
    leds.setLedColor(isCube);
    leds.setLedEnabled(isEnabled);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
    //     .onTrue(new ExampleCommand(m_exampleSubsystem));

    // // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // // cancelling on release.
    // m_controller.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

    m_controller.b().onTrue(new InstantCommand(m_chassis::resetGyro));
//    m_controller.x().whileTrue(m_chassis.run(() -> {m_chassis.crossWheels();}));
//    m_controller.a().onTrue(m_cameras.runOnce(() -> {m_cameras.togglePipeline();}));

    // m_controller.y().onTrue(m_arm.runOnce(() -> {m_arm.setTalonTargets(HIGH_BASE_POS, HIGH_WRIST_POS);}));
    m_controller.y().onTrue(m_arm.runOnce(() -> {m_arm.setArmHigh();}));
    // m_controller.x().onTrue(m_arm.runOnce(() -> {m_arm.setTalonTargets(MID_BASE_POS, MID_WRIST_POS);}));
    m_controller.x().onTrue(m_arm.runOnce(() -> {m_arm.setArmMid();}));
//    m_controller.x().onTrue(m_arm.runOnce(() -> {m_arm.setTalonTargets(MID_BASE_POS, 0);}));
//    m_controller.y().whileTrue(m_arm.run(() -> {m_arm.passSetpoints(PI/2/(PI/1024/BASE_GEAR_RATIO), 0);}));
//    m_controller.y().onFalse(m_arm.runOnce(() -> {m_arm.setTalonTargets(0, 0);}));
//    m_controller.y().whileTrue(new ManageArm(m_arm));

    m_controller.a().onTrue(m_arm.runOnce(() -> {m_arm.setTalonTargets(0, 0);}));


    m_controller.leftBumper().onTrue(m_chassis.runOnce(() -> {m_chassis.togglePrecision();}));
    //m_controller.leftBumper().whileTrue(new ChassisAutoBalance(m_chassis));
//    m_controller.leftBumper().onTrue(m_arm.runOnce(() -> {m_arm.setTalonTargets(0, 30*PI/180/(PI/1024/WRIST_GEAR_RATIO));}));
    // m_controller.rightBumper().onTrue(m_arm.runOnce(() -> {m_arm.setTalonTargets(0, 30*PI/180/(PI/1024/WRIST_GEAR_RATIO));}));
    m_controller.rightBumper().whileTrue(mIntake.run(() -> {mIntake.set(INTAKE_PCT);}));
    // m_controller.rightBumper().onTrue(m_arm.runOnce(() -> {m_arm.setTalonTargets(INTAKE_BASE_POS_CONE, INTAKE_WRIST_POS_CONE);}));
    m_controller.rightBumper().onTrue(m_arm.runOnce(() -> {m_arm.setArmConeIntake();;}));
    m_controller.rightBumper().onFalse(mIntake.runOnce(() -> {mIntake.set(0.0);}));

//    m_controller.rightBumper().whileTrue(new ChassisTargetToCone(m_chassis, m_cameras));

    m_controller.povUp().onTrue(m_arm.runOnce(() -> {m_arm.setTalonTargets(m_arm.baseTalonTarget - 5000, m_arm.wristTalonTarget);}));
    m_controller.povDown().onTrue(m_arm.runOnce(() -> {m_arm.setTalonTargets(m_arm.baseTalonTarget + 5000, m_arm.wristTalonTarget);}));

    m_controller.povRight().whileTrue(mIntake.run(() -> {mIntake.set(-1*INTAKE_PCT);}));
    m_controller.povRight().onFalse(mIntake.runOnce(() -> {mIntake.set(0.0);}));
    m_controller.povLeft().whileTrue(mIntake.run(() -> {mIntake.set(INTAKE_PCT);}));
    m_controller.povLeft().onFalse(mIntake.runOnce(() -> {mIntake.set(0.0);}));

//    m_controller.rightTrigger(0.7).onTrue(m_arm.runOnce(() -> {m_arm.setTalonTargets(30*PI/180/(PI/1024/BASE_GEAR_RATIO), -50*PI/180/(PI/1024/WRIST_GEAR_RATIO));}));
    // m_controller.rightTrigger(0.7).onTrue(m_arm.runOnce(() -> {m_arm.setTalonTargets(INTAKE_BASE_POS_CUBE, INTAKE_WRIST_POS_CUBE);}));
    m_controller.rightTrigger().onTrue(m_arm.runOnce(() -> {m_arm.setArmCubeIntake();}));
    m_controller.rightTrigger(0.7).whileTrue(mIntake.run(() -> {mIntake.set(INTAKE_PCT);}));
    m_controller.rightTrigger(0.7).onFalse(mIntake.runOnce(() -> {mIntake.set(-0.2);}));

    m_controller.leftTrigger(0.7).whileTrue(mIntake.run(() -> {mIntake.setVolts(9.0);}));
    m_controller.leftTrigger(0.7).onFalse(mIntake.runOnce(() -> {mIntake.set(0.0);}));

    m_controller.back().onTrue(m_arm.runOnce(() -> {m_arm.setTalonTargets(CHUTE_BASE_POS, CHUTE_WRIST_POS);}));

    m_controller.start().whileTrue(new ChassisDriveToNearestTarget(m_chassis, m_cameras, 99.0));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
//  public Command getAutonomousCommand() {
//    // An example command will be run in autonomous
//    return autoCommand;
//  }
}
