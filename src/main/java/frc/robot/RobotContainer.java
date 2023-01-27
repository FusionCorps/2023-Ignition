// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.cameras.UpdateOdometryBotpose;
import frc.robot.commands.chassis.ChassisAltAutoBalance;
import frc.robot.commands.chassis.ChassisDriveFC;
import frc.robot.commands.chassis.ChassisTargetToCone;
import frc.robot.subsystems.Cameras;
import frc.robot.subsystems.Chassis;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.ShooterTest;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class



RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Chassis m_chassis = new Chassis();
  private final Cameras m_cameras = new Cameras();
  private final ShooterTest m_shooter = new ShooterTest();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  public static CommandXboxController m_controller =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  public Command autoOne;
  public Command autoTwo;


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    m_chassis.comboFR.zero();
    m_chassis.comboBR.zero();
    m_chassis.comboFL.zero();
    m_chassis.comboBL.zero();

    m_chassis.setDefaultCommand(new ChassisDriveFC(m_chassis));

    PathPlannerTrajectory examplePath = PathPlanner.loadPath("test_line", new PathConstraints(4, 3));
    autoOne = m_chassis.followTrajectoryCommand(examplePath, true);

    PathPlannerTrajectory examplePathTwo = PathPlanner.loadPath("2piececombo", new PathConstraints(4, 3));
    autoTwo = new SequentialCommandGroup(m_chassis.runOnce(() -> {m_chassis.setGyroAngle(180);}),
            m_chassis.followTrajectoryCommand(examplePathTwo, true));
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
    m_controller.x().whileTrue(m_chassis.run(() -> {m_chassis.crossWheels();}));
    m_controller.a().onTrue(m_cameras.runOnce(() -> {m_cameras.togglePipeline();}));
    m_controller.y().whileTrue(new UpdateOdometryBotpose(m_chassis, m_cameras));

    //m_controller.leftBumper().whileTrue(new ChassisAutoBalance(m_chassis));
    m_controller.leftBumper().whileTrue(new ChassisAltAutoBalance(m_chassis));

    m_controller.rightBumper().whileTrue(new ChassisTargetToCone(m_chassis, m_cameras));
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
