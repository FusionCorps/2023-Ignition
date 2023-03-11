package frc.robot.commands.autos;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.ArmToPosition;
import frc.robot.commands.chassis.ChassisAutoBalance;
import frc.robot.commands.chassis.ChassisDriveAuton;
import frc.robot.commands.intake.RunVoltsTime;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Cameras;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Intake;

import static frc.robot.Constants.ArmConstants.*;
import static frc.robot.Constants.ArmConstants.MID_WRIST_POS;
import static frc.robot.Constants.IntakeConstants.OUTTAKE_VOLTS;

public class TwoPieceMidBalance extends SequentialCommandGroup {

    PathPlannerTrajectory twoPieceLoadSideASlow; // 4.5, 3
    PathPlannerTrajectory twoPieceLoadSideBSlow; // 5, 2

    PathPlannerTrajectory twoPieceLoadSideBalancePath;

    Cameras m_cameras;
    Chassis m_chassis;
    Arm m_arm;
    Intake m_Intake;

    public TwoPieceMidBalance(Cameras cameras, Chassis chassis, Arm arm, Intake intake, boolean isRed) {

        m_cameras = cameras;
        m_chassis = chassis;
        m_arm = arm;
        m_Intake = intake;

        twoPieceLoadSideASlow = PathPlanner.loadPath("1+1_path1R", new PathConstraints(4.5, 3)); // 4.5, 3
        twoPieceLoadSideBSlow = PathPlanner.loadPath("1+1_path2R", new PathConstraints(5, 2)); // 5, 2

        twoPieceLoadSideBalancePath = PathPlanner.loadPath("1+1_pathToBalance", new PathConstraints(2, 3));


        if (isRed) {
            twoPieceLoadSideASlow = PathPlannerTrajectory.transformTrajectoryForAlliance(twoPieceLoadSideASlow, DriverStation.Alliance.Red);
            twoPieceLoadSideBSlow = PathPlannerTrajectory.transformTrajectoryForAlliance(twoPieceLoadSideBSlow, DriverStation.Alliance.Red);

            twoPieceLoadSideBalancePath = PathPlannerTrajectory.transformTrajectoryForAlliance(twoPieceLoadSideBalancePath, DriverStation.Alliance.Red);
        }

        addCommands(
                m_cameras.runOnce(() -> { System.out.println("Running two piece loader side"); }),
                m_chassis.runOnce(() -> { m_chassis.setGyroAngle(0.0); }),
                new ArmToPosition(m_arm, MID_BASE_POS, MID_WRIST_POS, 0.1),
                new RunVoltsTime(m_Intake, OUTTAKE_VOLTS, 0.25),
                new ParallelCommandGroup(new ArmToPosition(m_arm, INTAKE_BASE_POS_CONE, INTAKE_WRIST_POS_CONE), // deploy intake
                        m_chassis.followTrajectoryCommand(twoPieceLoadSideASlow, true), // drive to piece
                        new RunVoltsTime(m_Intake, -11.0, twoPieceLoadSideASlow.getTotalTimeSeconds())), // intake
                m_Intake.runOnce(() -> {
                    m_Intake.set(-0.2);
                }),// intake
                new ParallelCommandGroup(new ArmToPosition(m_arm, MID_BASE_POS, MID_WRIST_POS), // stow arm
                        m_chassis.followTrajectoryCommand(twoPieceLoadSideBSlow, false)), // return to scoring
                // new ChassisDriveToNearestTarget(m_chassis, m_cameras, 0.2), // drive forward to align
                new ChassisDriveAuton(m_chassis, 0.2, 0.0, 0.0, 0.1), // drive forward to align
                new ArmToPosition(m_arm, MID_BASE_POS, MID_WRIST_POS, 0.02),
                new RunVoltsTime(m_Intake, OUTTAKE_VOLTS, 0.25),
                new ArmToPosition(m_arm, 0, 0, 0.25), // return to stow
                m_chassis.followTrajectoryCommand(twoPieceLoadSideBalancePath, true), // drive to piece
                new ChassisAutoBalance(m_chassis)// balance
                // new ChassisAutoBalanceFast(m_chassis)
        );
    }

}
