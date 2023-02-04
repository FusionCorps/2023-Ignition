// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/*
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;*/
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.data.CSVManager;
import frc.robot.math.Tilt;

import static frc.robot.RobotContainer.m_chassis;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  // -----AUTON SELECTION INSTRUCTIONS--------
  // to add auton to auton selection, initiate auton variable here:
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  CSVManager logs;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.

     // -----AUTON SELECTION INSTRUCTIONS--------
    // to add new command to command selction follow this format:
    // m_chooser.addOption("Auton_Name", auton_variable)
    // if it is the first auton in the list, use m_chooser.setDefaultOption("Auton_Name", auton_variable)
    m_robotContainer = new RobotContainer();

    // adds the auton selection to ShuffleBoard
    SmartDashboard.putData(m_chooser);

  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    if (logs != null) logs.close();
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    /*PathPlannerTrajectory examplePath = PathPlanner.loadPath("test_line", new PathConstraints(8, 5));

    // returns the selected auton
    m_autonomousCommand = m_chooser.getSelected();
    
    //m_autonomousCommand = m_chassis.followTrajectoryCommand(examplePath, true);


    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }*/
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}
  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.

    logs = new CSVManager("rotations");

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    double yaw = m_chassis.ahrs.getYaw();
    double tilt = Tilt.calculate(m_chassis.ahrs.getPitch(), m_chassis.ahrs.getRoll());
    logs.logDataPoint(yaw, tilt);
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
    Shuffleboard.getTab("Hello").add("Toast",2);
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }
}
