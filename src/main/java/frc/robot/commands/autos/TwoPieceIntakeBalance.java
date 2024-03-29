package frc.robot.commands.autos;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.ArmToPosition;
import frc.robot.commands.chassis.ChassisAutoBalanceFast;
import frc.robot.commands.chassis.ChassisAutoBalanceNew;
import frc.robot.commands.chassis.ChassisDriveAuton;
import frc.robot.commands.intake.RunVoltsTime;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Cameras;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Intake;

import static frc.robot.Constants.ArmConstants.*;
import static frc.robot.Constants.ArmConstants.INTAKE_WRIST_POS_CONE;
import static frc.robot.Constants.IntakeConstants.OUTTAKE_VOLTS;

public class TwoPieceIntakeBalance extends SequentialCommandGroup {

    Chassis m_chassis;
    Cameras m_camera;
    Arm m_arm;
    Intake m_intake;

    PathPlannerTrajectory twoPieceIntakeBalanceA;
    PathPlannerTrajectory twoPieceIntakeBalanceB;
    PathPlannerTrajectory twoPieceIntakeBalanceC;

    PathPlannerTrajectory balance;

    public TwoPieceIntakeBalance(Chassis chassis, Arm arm, Intake intake, Cameras camera, boolean isRed){
        m_camera = camera;
        m_chassis = chassis;
        m_arm = arm;
        m_intake = intake;

        twoPieceIntakeBalanceA = PathPlanner.loadPath("MilB_3Piece_1", new PathConstraints(5, 1.75));
        twoPieceIntakeBalanceB = PathPlanner.loadPath("MilB_3Piece_2", new PathConstraints(5, 1.75));
        twoPieceIntakeBalanceC = PathPlanner.loadPath("MilB_3Piece_3", new PathConstraints(5, 2));

        balance = PathPlanner.loadPath("MilB_3PieceToBalance", new PathConstraints(4,3));

        if(isRed){
            twoPieceIntakeBalanceA = PathPlannerTrajectory.transformTrajectoryForAlliance(twoPieceIntakeBalanceA, DriverStation.Alliance.Red);
            twoPieceIntakeBalanceB = PathPlannerTrajectory.transformTrajectoryForAlliance(twoPieceIntakeBalanceB, DriverStation.Alliance.Red);
            twoPieceIntakeBalanceC = PathPlannerTrajectory.transformTrajectoryForAlliance(twoPieceIntakeBalanceC, DriverStation.Alliance.Red);
            balance = PathPlannerTrajectory.transformTrajectoryForAlliance(balance, DriverStation.Alliance.Red);
        }

        addCommands(
                m_chassis.runOnce(() -> { m_chassis.setGyroAngle(0.0); }),
                new ArmToPosition(m_arm, MID_BASE_POS, MID_WRIST_POS, 0.25),
                new RunVoltsTime(m_intake, OUTTAKE_VOLTS, 0.2),
                new ParallelCommandGroup(
                        new ArmToPosition(m_arm,INTAKE_BASE_POS_CUBE,INTAKE_WRIST_POS_CUBE),
                        m_chassis.followTrajectoryCommand(twoPieceIntakeBalanceA,true),
                        new RunVoltsTime(m_intake,-0.45*12,twoPieceIntakeBalanceA.getTotalTimeSeconds())
                ),
                new ParallelCommandGroup(
                        m_intake.runOnce(() -> {m_intake.set(-0.2);}),
                        new ArmToPosition(m_arm, MID_BASE_POS_CUBE, MID_WRIST_POS_CUBE),
                        m_chassis.followTrajectoryCommand(twoPieceIntakeBalanceB,false)
                ),
                new ChassisDriveAuton(m_chassis, 0.25, 0.0, 0.0, 0.1),
                new RunVoltsTime(m_intake,3.5,0.1),
                new ParallelCommandGroup(
                        new ArmToPosition(m_arm,INTAKE_BASE_POS_CONE,INTAKE_WRIST_POS_CONE),
                        m_chassis.followTrajectoryCommand(twoPieceIntakeBalanceC,false),
                        new RunVoltsTime(m_intake,-11,twoPieceIntakeBalanceC.getTotalTimeSeconds())
                ),

                new ParallelCommandGroup(
                        new ArmToPosition(m_arm,BASE_START_POS,WRIST_START_POS),
                        m_chassis.followTrajectoryCommand(balance,false)
                ),
                new ChassisAutoBalanceNew(m_chassis)
        );
    }
}
