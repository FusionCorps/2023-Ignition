// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.arm.*;
import frc.robot.commands.autos.*;
import frc.robot.commands.chassis.*;
import frc.robot.commands.intake.RunVoltsTime;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static frc.robot.Constants.ArmConstants.*;
import static frc.robot.Constants.IntakeConstants.INTAKE_PCT;
import static frc.robot.Constants.IntakeConstants.OUTTAKE_VOLTS;
import static java.lang.Math.PI;


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
    public Command twoPieceLoadSideBalance;
    public Command twoPieceLoadSideBalanceRed;
    public Command oneMidFarSide;
    public Command twoPieceLoadSideMid;
    public Command twoPieceLoadSideVikes;
    public Command threePieceLoadSide;
    public Command threePieceLoadSideNested;
    public Command onePieceBalance;
    public Command threePieceLoadSideCube;

    public Command twoPieceLoadSideCubeBlue;
    public Command twoPieceLoadSideCubeRed;

    public Command threePieceLoadSideMidBlue;
    public Command threePieceLoadSideMidRed;

    public Command twoPieceWireSideHighBlue;
    public Command twoPieceWireSideHighRed;

    public Command twoPieceCubeLoadSideBalanceBlue;
    public Command twoPieceCubeLoadSideBalanceRed;

    public Command relaxArm;

    public Command twoPieceIntakeLoadSideBalanceBlue;
    public Command twoPieceIntakeLoadSideBalanceRed;

    public Command twoPieceCenterBalanceBlue;
    public Command twoPieceCenterBalanceRed;

    private final Leds leds = new Leds();


    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
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

        PathPlannerTrajectory twoPieceLoadSideA = PathPlanner.loadPath("1+1_path1R", new PathConstraints(4, 3));
        PathPlannerTrajectory twoPieceLoadSideB = PathPlanner.loadPath("1+1_path2R", new PathConstraints(4, 3));

        PathPlannerTrajectory twoPieceLoadSideASlow = PathPlanner.loadPath("1+1_path1R", new PathConstraints(4.5, 3)); // 4.5, 3
        PathPlannerTrajectory twoPieceLoadSideBSlow = PathPlanner.loadPath("1+1_path2R", new PathConstraints(5, 2)); // 5, 2

        PathPlannerTrajectory twoPieceLoadSideBalancePath = PathPlanner.loadPath("1+1_pathToBalance", new PathConstraints(2, 3));

        PathPlannerTrajectory onePieceFarSide = PathPlanner.loadPath("taxiAttempt", new PathConstraints(4, 3));

        PathPlannerTrajectory twoPieceLoadSideAVikes = PathPlanner.loadPath("1+1_path1RVikes", new PathConstraints(3, 3));
        PathPlannerTrajectory twoPieceLoadSideBVikes = PathPlanner.loadPath("1+1_path2RVikes", new PathConstraints(3, 3));


        PathPlannerTrajectory threePieceLoadSideA = PathPlanner.loadPath("1+1_path1R", new PathConstraints(4, 3));
        PathPlannerTrajectory threePieceLoadSideB = PathPlanner.loadPath("1+1_path2R", new PathConstraints(4, 3));
        PathPlannerTrajectory threePieceLoadSideC = PathPlanner.loadPath("1+2_path2R", new PathConstraints(4, 3));
        PathPlannerTrajectory threePieceLoadSideD = PathPlanner.loadPath("1+2_path3R", new PathConstraints(4, 3));
        PathPlannerTrajectory threePieceLoadSideCubeB = PathPlanner.loadPath("1+2Cube_2R", new PathConstraints(4, 3));
        PathPlannerTrajectory threePieceLoadSideCubeC = PathPlanner.loadPath("1+2Cube_3R", new PathConstraints(4, 3));
        PathPlannerTrajectory threePieceLoadSideCubeD = PathPlanner.loadPath("1+2Cube_4R", new PathConstraints(4, 3));

        PathPlannerTrajectory twoPieceLoadSideARed = PathPlanner.loadPath("1+1_path1Rred", new PathConstraints(4, 3));
        PathPlannerTrajectory threePieceLoadSideCubeBRed = PathPlanner.loadPath("1+2Cube_2Rred", new PathConstraints(4, 3));

        // TODO: Standardize autonomous outtake voltage
        twoPieceLoadSide = new SequentialCommandGroup(
                m_cameras.runOnce(() -> { System.out.println("Running two piece loader side"); }),
                m_chassis.runOnce(() -> { m_chassis.setGyroAngle(0.0); }),
                new TwoPartHigh(m_arm), // arm to high
                new ArmToPosition(m_arm, HIGH_BASE_POS_ALT, HIGH_WRIST_POS_ALT - 2000, 0.5),
                new RunVoltsTime(mIntake, OUTTAKE_VOLTS, 0.5),
                new ArmToPosition(m_arm, 0, 0, 0.25), // return to stow
                new ParallelCommandGroup(new ArmToPosition(m_arm, INTAKE_BASE_POS_CONE, INTAKE_WRIST_POS_CONE), // deploy intake
                        m_chassis.followTrajectoryCommand(twoPieceLoadSideA, true), // drive to piece
                        new RunVoltsTime(mIntake, -9.0, twoPieceLoadSideA.getTotalTimeSeconds())), // intake
                new ParallelCommandGroup(new ArmToPosition(m_arm, 0, 0), // stow arm
                        m_chassis.followTrajectoryCommand(twoPieceLoadSideB, false)), // return to scoring
                // new ChassisDriveToNearestTarget(m_chassis, m_cameras, 0.2), // drive forward to align
                new ChassisDriveAuton(m_chassis, 0.2, 0.0, 0.0, 0.2), // drive forward to align
                new TwoPartHigh(m_arm), // arm to high
                new ArmToPosition(m_arm, HIGH_BASE_POS_ALT, HIGH_WRIST_POS_ALT - 2000, 0.5),
                new RunVoltsTime(mIntake, OUTTAKE_VOLTS, 0.25)
        );


        twoPieceLoadSideCubeBlue = new SequentialCommandGroup(
                m_chassis.runOnce(() -> { m_chassis.setGyroAngle(0.0); }),
                new TwoPartHighAuto(m_arm), // arm to high
                new ArmToPosition(m_arm, HIGH_BASE_POS_ALT_AUTO, HIGH_WRIST_POS_ALT_AUTO, 0.75),
                new RunVoltsTime(mIntake, OUTTAKE_VOLTS, 0.75), // outtake
                new ArmToPosition(m_arm, 0, 0, 0.25), // return to stow
                new ParallelCommandGroup(m_chassis.followTrajectoryCommand(twoPieceLoadSideA, true),
                        new ArmToPosition(m_arm, INTAKE_BASE_POS_CUBE, INTAKE_WRIST_POS_CUBE),
                        new RunVoltsTime(mIntake, -11.0, twoPieceLoadSideA.getTotalTimeSeconds())),
                mIntake.runOnce(() -> {
                    mIntake.set(-0.2);
                }),// intake
                new ParallelCommandGroup(new ArmToPosition(m_arm, 0, 0), // stow arm
                        m_chassis.followTrajectoryCommand(threePieceLoadSideCubeB, false)), // return to scoring
                // new ChassisDriveToNearestTarget(m_chassis, m_cameras, 0.2), // drive forward to align
                // new ChassisDriveAuton(m_chassis, 0.2, 0.0, 0.0, 0.2), // drive forward to align
                new TwoPartHighAuto(m_arm), // arm to high
                new ArmToPosition(m_arm, HIGH_BASE_POS_ALT_AUTO, HIGH_WRIST_POS_ALT_AUTO, 0.75),
                new RunVoltsTime(mIntake, OUTTAKE_VOLTS, 0.75), // outtake
                new ArmToPosition(m_arm, 0, 0)
        );

        twoPieceLoadSideCubeRed = new SequentialCommandGroup(
                m_chassis.runOnce(() -> { m_chassis.setGyroAngle(0.0); }),
                new TwoPartHighAuto(m_arm), // arm to high
                new ArmToPosition(m_arm, HIGH_BASE_POS_ALT_AUTO, HIGH_WRIST_POS_ALT_AUTO, 0.75),
                new RunVoltsTime(mIntake, OUTTAKE_VOLTS, 0.75), // outtake
                new ArmToPosition(m_arm, 0, 0, 0.25), // return to stow
                new ParallelCommandGroup(m_chassis.followTrajectoryCommand(twoPieceLoadSideARed, true),
                        new ArmToPosition(m_arm, INTAKE_BASE_POS_CUBE, INTAKE_WRIST_POS_CUBE),
                        new RunVoltsTime(mIntake, -11.0, twoPieceLoadSideA.getTotalTimeSeconds())),
                mIntake.runOnce(() -> {
                    mIntake.set(-0.2);
                }),// intake
                new ParallelCommandGroup(new ArmToPosition(m_arm, 0, 0), // stow arm
                        m_chassis.followTrajectoryCommand(threePieceLoadSideCubeBRed, false)), // return to scoring
                // new ChassisDriveToNearestTarget(m_chassis, m_cameras, 0.2), // drive forward to align
                // new ChassisDriveAuton(m_chassis, 0.2, 0.0, 0.0, 0.2), // drive forward to align
                new TwoPartHighAuto(m_arm), // arm to high
                new ArmToPosition(m_arm, HIGH_BASE_POS_ALT_AUTO, HIGH_WRIST_POS_ALT_AUTO, 0.75),
                new RunVoltsTime(mIntake, OUTTAKE_VOLTS, 0.75), // outtake
                new ArmToPosition(m_arm, 0, 0)
        );

        // TODO: Standardize autonomous outtake voltage
//        twoPieceLoadSideBalance = new SequentialCommandGroup(
//                m_cameras.runOnce(() -> { System.out.println("Running two piece loader side"); }),
//                m_chassis.runOnce(() -> { m_chassis.setGyroAngle(0.0); }),
//                new ArmToPosition(m_arm, MID_BASE_POS, MID_WRIST_POS, 0.1),
//                new RunVoltsTime(mIntake, OUTTAKE_VOLTS, 0.25),
//                new ParallelCommandGroup(new ArmToPosition(m_arm, INTAKE_BASE_POS_CONE, INTAKE_WRIST_POS_CONE), // deploy intake
//                        m_chassis.followTrajectoryCommand(twoPieceLoadSideASlow, true), // drive to piece
//                        new RunVoltsTime(mIntake, -11.0, twoPieceLoadSideASlow.getTotalTimeSeconds())), // intake
//                mIntake.runOnce(() -> {
//                    mIntake.set(-0.2);
//                }),// intake
//                new ParallelCommandGroup(new ArmToPosition(m_arm, MID_BASE_POS, MID_WRIST_POS), // stow arm
//                        m_chassis.followTrajectoryCommand(twoPieceLoadSideBSlow, false)), // return to scoring
//                // new ChassisDriveToNearestTarget(m_chassis, m_cameras, 0.2), // drive forward to align
//                new ChassisDriveAuton(m_chassis, 0.2, 0.0, 0.0, 0.1), // drive forward to align
//                new ArmToPosition(m_arm, MID_BASE_POS, MID_WRIST_POS, 0.02),
//                new RunVoltsTime(mIntake, OUTTAKE_VOLTS, 0.25),
//                new ArmToPosition(m_arm, 0, 0, 0.25), // return to stow
//                m_chassis.followTrajectoryCommand(twoPieceLoadSideBalancePath, true), // drive to piece
//                new ChassisAutoBalance(m_chassis) // balance
//        );

        twoPieceLoadSideBalance = new TwoPieceMidBalance(m_cameras, m_chassis, m_arm, mIntake, false);
        twoPieceLoadSideBalanceRed = new TwoPieceMidBalance(m_cameras, m_chassis, m_arm, mIntake, true);

        threePieceLoadSideMidBlue = new ThreePieceMid(m_cameras,m_chassis,m_arm,mIntake,false);
        threePieceLoadSideMidRed = new ThreePieceMid(m_cameras,m_chassis,m_arm,mIntake,true);

        twoPieceWireSideHighBlue = new TwoPieceHighWire(m_chassis,m_arm,m_cameras,mIntake,false);
        twoPieceWireSideHighRed = new TwoPieceHighWire(m_chassis,m_arm,m_cameras,mIntake,true);

        twoPieceCubeLoadSideBalanceBlue = new TwoPieceCubeBalance(m_cameras, m_chassis, m_arm, mIntake,false);
        twoPieceCubeLoadSideBalanceRed = new TwoPieceCubeBalance(m_cameras, m_chassis, m_arm, mIntake,true);

        twoPieceIntakeLoadSideBalanceBlue = new TwoPieceIntakeBalance(m_chassis,m_arm,mIntake,m_cameras,false);
        twoPieceIntakeLoadSideBalanceRed = new TwoPieceIntakeBalance(m_chassis,m_arm,mIntake,m_cameras,true);

        twoPieceCenterBalanceBlue = new OnePieceTaxiBalance(m_chassis,m_arm,m_cameras,mIntake,false);
        twoPieceCenterBalanceRed = new OnePieceTaxiBalance(m_chassis,m_arm,m_cameras,mIntake,true);


        // TODO: Standardize autonomous outtake voltage
        oneMidFarSide = new SequentialCommandGroup(
                m_chassis.runOnce(() -> { m_chassis.setGyroAngle(0.0); }),
                new TwoPartHighAuto(m_arm), // arm to high
                new ArmToPosition(m_arm, HIGH_BASE_POS_ALT_AUTO, HIGH_WRIST_POS_ALT_AUTO, 0.75),
                new RunVoltsTime(mIntake, OUTTAKE_VOLTS, 0.75),
                new ArmToPosition(m_arm, 0, 0, 0.25), // return to stow
                new ParallelCommandGroup(new ArmToPosition(m_arm, INTAKE_BASE_POS_CONE, INTAKE_WRIST_POS_CONE), // deploy intake
                        m_chassis.followTrajectoryCommand(onePieceFarSide, true), // drive to piece
                        new RunVoltsTime(mIntake, -9.0, onePieceFarSide.getTotalTimeSeconds())), // intake
                new ArmToPosition(m_arm, 0, 0, 0.25)
        );

        // TODO: Standardize autonomous outtake voltage
        twoPieceLoadSideMid = new SequentialCommandGroup(
                m_cameras.runOnce(() -> {
                    System.out.println("Running two piece loader side");
                }),
                m_chassis.runOnce(() -> {
                    m_chassis.setGyroAngle(0.0);
                }),
                new ParallelCommandGroup(
                        m_chassis.runOnce(() -> {
                            m_chassis.crossWheels();
                        }),
                        new ArmToPosition(m_arm, MID_BASE_POS, MID_WRIST_POS)), // arm to high
                new RunVoltsTime(mIntake, 5.6, 1.0), // outtake
                new ArmToPosition(m_arm, 0, 0, 0.25), // return to stow
                new ParallelCommandGroup(new ArmToPosition(m_arm, INTAKE_BASE_POS_CONE, INTAKE_WRIST_POS_CONE), // deploy intake
                        m_chassis.followTrajectoryCommand(twoPieceLoadSideA, true), // drive to piece
                        new RunVoltsTime(mIntake, -9.0, twoPieceLoadSideA.getTotalTimeSeconds())), // intake
                new ParallelCommandGroup(new ArmToPosition(m_arm, 0, 0), // stow arm
                        m_chassis.followTrajectoryCommand(twoPieceLoadSideB, false)), // return to scoring
                // new ChassisDriveToNearestTarget(m_chassis, m_cameras, 0.2), // drive forward to align
                new ChassisDriveAuton(m_chassis, 0.2, 0.0, 0.0, 0.2), // drive forward to align
                new ArmToPosition(m_arm, MID_BASE_POS, MID_WRIST_POS), // arm to high
                new RunVoltsTime(mIntake, 5.6, 1.0) // outtake
        );

        twoPieceLoadSideVikes = new SequentialCommandGroup(
                m_chassis.runOnce(() -> {
                    m_chassis.setGyroAngle(0.0);
                }),
                new ArmToPosition(m_arm, HIGH_BASE_POS_VIKES, HIGH_WRIST_POS_VIKES),
                new RunVoltsTime(mIntake, 9.0, 0.5),
//            new ArmToPosition(m_arm, 0, 0),
                new ArmToPosition(m_arm, 0, 0),
                new ParallelCommandGroup(new ArmToPosition(m_arm, INTAKE_BASE_POS_CONE, INTAKE_WRIST_POS_CONE),
                        m_chassis.followTrajectoryCommand(twoPieceLoadSideAVikes, true),
                        new RunVoltsTime(mIntake, -9.0, twoPieceLoadSideAVikes.getTotalTimeSeconds())),
                new ParallelCommandGroup(new ArmToPosition(m_arm, 0, 0),
                        m_chassis.followTrajectoryCommand(twoPieceLoadSideBVikes, false)),
                new ChassisDriveToNearestTarget(m_chassis, m_cameras, 0.2),
                new ArmToPosition(m_arm, HIGH_BASE_POS_VIKES, HIGH_WRIST_POS_VIKES),
                new RunVoltsTime(mIntake, 9.0, 0.5)
        );

        onePieceBalance = new SequentialCommandGroup(
                m_chassis.runOnce(() -> {
                    m_chassis.setGyroAngle(0.0);
                }), // reset gyro
                new TwoPartHighAuto(m_arm), // arm to high
                new ArmToPosition(m_arm, HIGH_BASE_POS_ALT_AUTO, HIGH_WRIST_POS_ALT_AUTO, 0.75),
                new RunVoltsTime(mIntake, OUTTAKE_VOLTS, 0.75), // outtake
                new ArmToPosition(m_arm, 0, 0), // stow
                new ChassisDriveAuton(m_chassis, -0.2, 0.0, 0.0, 3.0), // drive forward
                new ChassisAutoBalanceNew(m_chassis) // balance
        );

        threePieceLoadSide = new SequentialCommandGroup(
                m_chassis.runOnce(() -> {
                    m_chassis.setGyroAngle(0.0);
                }), // reset gyro
                new ArmToPosition(m_arm, HIGH_BASE_POS, HIGH_WRIST_POS), // arm to high
                new RunVoltsTime(mIntake, 9.0, 0.5),
                new ParallelCommandGroup(m_chassis.followTrajectoryCommand(twoPieceLoadSideA, true),
                        new ArmToPosition(m_arm, INTAKE_BASE_POS_CONE, INTAKE_WRIST_POS_CONE),
                        new RunVoltsTime(mIntake, -9.0, twoPieceLoadSideA.getTotalTimeSeconds())),
                new ParallelCommandGroup(new ArmToPosition(m_arm, 0, 0),
                        m_chassis.followTrajectoryCommand(twoPieceLoadSideB, false)),
                new ChassisDriveToNearestTarget(m_chassis, m_cameras, 0.2),
                new ArmToPosition(m_arm, HIGH_BASE_POS, HIGH_WRIST_POS),
                new RunVoltsTime(mIntake, 9, 0.5),

                new ParallelCommandGroup(m_chassis.followTrajectoryCommand(threePieceLoadSideC, false),
                        new ArmToPosition(m_arm, INTAKE_BASE_POS_CUBE, INTAKE_WRIST_POS_CUBE),
                        new RunVoltsTime(mIntake, -4, threePieceLoadSideC.getTotalTimeSeconds())),
                new ParallelCommandGroup(new ArmToPosition(m_arm, 0, 0),
                        m_chassis.followTrajectoryCommand(threePieceLoadSideD, false)),
                new ArmToPosition(m_arm, MID_BASE_POS, MID_WRIST_POS),
                new RunVoltsTime(mIntake, 9, 0.5));

        // REMEMBER THE OTHER AUTOS ARE COMMANDS TOO - AO
//      threePieceLoadSideNested = new SequentialCommandGroup(
//              twoPieceLoadSide,
//              new ArmToPosition(m_arm, 0.0, 0.0, 0.0), // stow before you move
//              new ParallelCommandGroup(m_chassis.followTrajectoryCommand(threePieceLoadSideC, false),
//                      new ArmToPosition(m_arm, INTAKE_BASE_POS_CUBE, INTAKE_WRIST_POS_CUBE),
//                      new RunVoltsTime(mIntake, -9, threePieceLoadSideC.getTotalTimeSeconds())),
//              mIntake.runOnce(() -> {mIntake.set(-0.2);}), // remember cube needs to be held in
//              new ParallelCommandGroup(new ArmToPosition(m_arm, 0, 0),
//                      m_chassis.followTrajectoryCommand(threePieceLoadSideD, false)),
//              new ArmToPosition(m_arm, MID_BASE_POS, MID_WRIST_POS),
//              new RunVoltsTime(mIntake, 9, 0.5) );

        threePieceLoadSideCube = new SequentialCommandGroup(
                m_chassis.runOnce(() -> {
                    m_chassis.setGyroAngle(0.0);
                }),
                new ArmToPosition(m_arm, MID_BASE_POS, MID_WRIST_POS),
                new RunVoltsTime(mIntake, 9.0, 0.5),
                //            new ArmToPosition(m_arm, 0, 0),
                new ArmToPosition(m_arm, 0, 0, 0.25),
                new ParallelCommandGroup(m_chassis.followTrajectoryCommand(twoPieceLoadSideA, true),
                        new ArmToPosition(m_arm, INTAKE_BASE_POS_CUBE, INTAKE_WRIST_POS_CUBE),
                        new RunVoltsTime(mIntake, -10.0, twoPieceLoadSideA.getTotalTimeSeconds())),
                mIntake.runOnce(() -> {
                    mIntake.set(-0.2);
                }),
                new ParallelCommandGroup(
                        new ArmToPosition(m_arm, MID_BASE_POS, MID_WRIST_POS),
                        m_chassis.followTrajectoryCommand(threePieceLoadSideCubeB, false)),
                // new ArmToPosition(m_arm, HIGH_BASE_POS, HIGH_WRIST_POS),
                new RunVoltsTime(mIntake, 9, 0.5),
                new ArmToPosition(m_arm, 0, 0),
                new ParallelCommandGroup(m_chassis.followTrajectoryCommand(threePieceLoadSideCubeC, false),
                        new ArmToPosition(m_arm, INTAKE_BASE_POS_CONE, INTAKE_WRIST_POS_CONE),
                        new RunVoltsTime(mIntake, -9, threePieceLoadSideCubeC.getTotalTimeSeconds())),
                new ParallelCommandGroup(new ArmToPosition(m_arm, 0, 0),
                        m_chassis.followTrajectoryCommand(threePieceLoadSideCubeD, false)),
                new ChassisDriveAuton(m_chassis, 0.2, 0.0, 0.0, 0.2), // drive forward to align
                new ArmToPosition(m_arm, MID_BASE_POS, MID_WRIST_POS),
                new RunVoltsTime(mIntake, 9, 0.5));


        relaxArm = new RelaxArm(m_arm);

    }



    public void ledPeriodic(boolean isCube, boolean isEnabled, boolean isRainbow) {
        leds.setLedColor(isCube, isRainbow);
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

        // m_controller.y().onTrue(m_arm.runOnce(() -> {m_arm.setTalonTargets(HIGH_BASE_POS_VIKES, HIGH_WRIST_POS_VIKES);}));
//        m_controller.y().onTrue(m_arm.runOnce(() -> {
//            m_arm.setArmHigh();
//        }));

        // m_controller.y().onTrue(new TwoPartHigh(m_arm));

        m_controller.y().onTrue(new TwoPartHigh(m_arm));
        // m_controller.y().onTrue(m_arm.runOnce(() -> {m_arm.setTalonTargets(MID_BASE_POS, -190*PI/180/(PI/1024/WRIST_GEAR_RATIO));}));

        // m_controller.x().onTrue(m_arm.runOnce(() -> {m_arm.setTalonTargets(MID_BASE_POS, MID_WRIST_POS);}));
        m_controller.x().onTrue(m_arm.runOnce(() -> {
            m_arm.setArmMid();
        }));
//    m_controller.x().onTrue(m_arm.runOnce(() -> {m_arm.setTalonTargets(MID_BASE_POS, 0);}));
//    m_controller.y().whileTrue(m_arm.run(() -> {m_arm.passSetpoints(PI/2/(PI/1024/BASE_GEAR_RATIO), 0);}));
//    m_controller.y().onFalse(m_arm.runOnce(() -> {m_arm.setTalonTargets(0, 0);}));
//    m_controller.y().whileTrue(new ManageArm(m_arm));

        m_controller.a().onTrue(m_arm.runOnce(() -> {
            m_arm.setArmStow();
        }));


        //m_controller.leftBumper().whileTrue(new ChassisAutoBalanceNew(m_chassis));
        //m_controller.leftBumper().whileTrue(new ChassisAutoBalanceFast(m_chassis));
        m_controller.leftBumper().onTrue(m_arm.runOnce(() -> {m_arm.setTalonTargets(LOW_BASE_POS_CUBE, LOW_WRIST_POS_CUBE);}));
//    m_controller.leftBumper().whileTrue(m_chassis.run(() -> {m_chassis.crossWheels();}));
//    m_controller.leftBumper().onTrue(m_arm.runOnce(() -> {m_arm.setTalonTargets(0, 30*PI/180/(PI/1024/WRIST_GEAR_RATIO));}));
        // m_controller.rightBumper().onTrue(m_arm.runOnce(() -> {m_arm.setTalonTargets(0, 30*PI/180/(PI/1024/WRIST_GEAR_RATIO));}));
        m_controller.rightBumper().whileTrue(mIntake.run(() -> {
            mIntake.set(INTAKE_PCT);
        }));
        // m_controller.rightBumper().onTrue(m_arm.runOnce(() -> {m_arm.setTalonTargets(INTAKE_BASE_POS_CONE, INTAKE_WRIST_POS_CONE);}));
        m_controller.rightBumper().onTrue(m_arm.runOnce(() -> {
            m_arm.setArmConeIntake();
        }));
        m_controller.rightBumper().onFalse(mIntake.runOnce(() -> {
            mIntake.set(-0.075);
        }));

//    m_controller.rightBumper().whileTrue(new ChassisTargetToCone(m_chassis, m_cameras));

        m_controller.povUp().onTrue(m_arm.runOnce(() -> {
            m_arm.setTalonTargets(m_arm.baseTalonTarget - 1000, m_arm.wristTalonTarget);
        }));
        m_controller.povDown().onTrue(m_arm.runOnce(() -> {
            m_arm.setTalonTargets(m_arm.baseTalonTarget + 1000, m_arm.wristTalonTarget);
        }));

        m_controller.povRight().whileTrue(mIntake.run(() -> {
            mIntake.set(-0.2 * INTAKE_PCT);
        }));
        m_controller.povRight().onFalse(mIntake.runOnce(() -> {
            mIntake.set(0.0);
        }));
        m_controller.povLeft().whileTrue(mIntake.run(() -> {
            mIntake.set(INTAKE_PCT);
        }));
        m_controller.povLeft().onFalse(mIntake.runOnce(() -> {
            mIntake.set(-0.075);
        }));

//    m_controller.rightTrigger(0.7).onTrue(m_arm.runOnce(() -> {m_arm.setTalonTargets(30*PI/180/(PI/1024/BASE_GEAR_RATIO), -50*PI/180/(PI/1024/WRIST_GEAR_RATIO));}));
        // m_controller.rightTrigger(0.7).onTrue(m_arm.runOnce(() -> {m_arm.setTalonTargets(INTAKE_BASE_POS_CUBE, INTAKE_WRIST_POS_CUBE);}));
        m_controller.rightTrigger().onTrue(m_arm.runOnce(() -> {
            m_arm.setArmCubeIntake();
        }));
        m_controller.rightTrigger(0.7).whileTrue(mIntake.run(() -> {
            mIntake.set(INTAKE_PCT);
        }));
        m_controller.rightTrigger(0.7).onFalse(mIntake.runOnce(() -> {
            mIntake.set(-0.2);
        }));

        m_controller.leftTrigger(0.7).whileTrue(mIntake.run(() -> {
            if(m_arm.hasCone) {
                mIntake.setVolts(OUTTAKE_VOLTS);
            }else{
                mIntake.setVolts(.09);
            }
        }));
        m_controller.leftTrigger(0.7).onFalse(mIntake.runOnce(() -> {
            mIntake.set(-0.0);
        }));

        m_controller.back().onTrue(m_arm.runOnce(() -> {
            m_arm.setTalonTargets(CHUTE_BASE_POS, CHUTE_WRIST_POS);
        }));
        m_controller.back().onTrue(m_arm.runOnce(() -> {
            m_arm.hasCone = true;
        }));

        // m_controller.start().whileTrue(new ChassisDriveToNearestTarget(m_chassis, m_cameras, 99.0));
        m_controller.start().onTrue(m_chassis.runOnce(() -> {
            m_chassis.togglePrecision();
        }));
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
