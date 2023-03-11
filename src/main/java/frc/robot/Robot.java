// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import static frc.robot.Constants.IS_LOGGING;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  // For enabling the cargo LEDs
  private NetworkTableEntry isCubeEntry;
  private NetworkTableEntry isEnabledEntry;
  private NetworkTableEntry isLocked;

  private RobotContainer m_robotContainer;

  // -----AUTON SELECTION INSTRUCTIONS--------
  // to add auton to auton selection, initiate auton variable here:
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our

    // initialises the LED stuffs
    NetworkTableInstance instance = NetworkTableInstance.getDefault();
    NetworkTable ledTable = instance.getTable("led");

    NetworkTableInstance lInstance = NetworkTableInstance.getDefault();
    NetworkTable lockedTable = lInstance.getTable("locked");

    isEnabledEntry = ledTable.getEntry("isEnabled");
    isEnabledEntry.setBoolean(false);
    isCubeEntry = ledTable.getEntry("isCube");
    isCubeEntry.setBoolean(false);



    // autonomous chooser on the dashboard.

    // -----AUTON SELECTION INSTRUCTIONS--------
    // to add new command to command selction follow this format:
    // m_chooser.addOption("Auton_Name", auton_variable)
    // if it is the first auton in the list, use m_chooser.setDefaultOption("Auton_Name", auton_variable)
    m_robotContainer = new RobotContainer();

    // m_chooser.addOption("Test Line", m_robotContainer.autoOne);
    m_chooser.addOption("Two Piece Loadside", m_robotContainer.twoPieceLoadSide);

    m_chooser.addOption("Two Piece Loadside and Balance Blue", m_robotContainer.twoPieceLoadSideBalance);
    m_chooser.addOption("Two Piece Loadside and Balance Red", m_robotContainer.twoPieceLoadSideBalanceRed);

    m_chooser.addOption("Three Piece Loadside Mid Blue", m_robotContainer.threePieceLoadSideMidBlue);
    m_chooser.addOption("Three Piece Loadside Mid Red", m_robotContainer.threePieceLoadSideMidRed);

    m_chooser.addOption("Two Piece Wireside High Blue",m_robotContainer.twoPieceWireSideHighBlue);
    m_chooser.addOption("Two Piece WireSide High Red",m_robotContainer.twoPieceWireSideHighRed);

    // m_chooser.addOption("Three Piece Left", m_robotContainer.threePieceLoadSide);
    m_chooser.addOption("One Piece Balance", m_robotContainer.onePieceBalance);
    m_chooser.addOption("One Piece Farside", m_robotContainer.oneMidFarSide);
    // m_chooser.addOption("Three Piece Left Cube", m_robotContainer.threePieceLoadSideCube);

    m_chooser.addOption("Cone/Cube Loadside Blue", m_robotContainer.twoPieceLoadSideCubeBlue);
    m_chooser.addOption("Cone/Cube Loadside Red", m_robotContainer.twoPieceLoadSideCubeRed);



    m_chooser.setDefaultOption("Two Piece Loadside and Balance", m_robotContainer.twoPieceLoadSideBalance);

    // adds the auton selection to ShuffleBoard
    SmartDashboard.putData(m_chooser);
    
    CameraServer.startAutomaticCapture();

    if (IS_LOGGING) {
      DataLogManager.start();
    }
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

    if (m_robotContainer != null) {
    }
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    m_robotContainer.relaxArm.schedule();
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {

    // returns the selected auton
     m_autonomousCommand = m_chooser.getSelected();
    //m_autonomousCommand = m_robotContainer.threePieceLoadSideCube;


    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
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
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    m_robotContainer.relaxArm.schedule();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

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
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
