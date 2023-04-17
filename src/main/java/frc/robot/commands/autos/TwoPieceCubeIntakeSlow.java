package frc.robot.commands.autos;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.ArmToPosition;
import frc.robot.commands.chassis.ChassisAutoBalanceFast;
import frc.robot.commands.chassis.ChassisDriveAuton;
import frc.robot.commands.intake.RunVoltsTime;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Cameras;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Intake;

import static frc.robot.Constants.ArmConstants.*;
import static frc.robot.Constants.ArmConstants.MID_WRIST_POS_CUBE;
import static frc.robot.Constants.IntakeConstants.OUTTAKE_VOLTS;

public class TwoPieceCubeIntakeSlow extends SequentialCommandGroup {

    Cameras m_cameras;
    Chassis m_chassis;
    Arm m_arm;
    Intake m_Intake;

    PathPlannerTrajectory twoPieceLoadsideA;
    PathPlannerTrajectory twoPieceLoadsideB;
    PathPlannerTrajectory twoPieceLoadsideC;


    public TwoPieceCubeIntakeSlow(Cameras cameras, Chassis chassis, Arm arm, Intake intake, boolean isRed){
        m_cameras = cameras;
        m_chassis = chassis;
        m_arm = arm;
        m_Intake = intake;

        twoPieceLoadsideA = PathPlanner.loadPath("MilB_3Piece_1", new PathConstraints(5, 1.5));
        twoPieceLoadsideB = PathPlanner.loadPath("MilB_3Piece_2", new PathConstraints(5, 1.5));
        twoPieceLoadsideC = PathPlanner.loadPath("MilB_3Piece_3", new PathConstraints(5, 1.5));


        if(isRed){
            twoPieceLoadsideA = PathPlannerTrajectory.transformTrajectoryForAlliance(twoPieceLoadsideA, DriverStation.Alliance.Red);
            twoPieceLoadsideB = PathPlannerTrajectory.transformTrajectoryForAlliance(twoPieceLoadsideB, DriverStation.Alliance.Red);
            twoPieceLoadsideC = PathPlannerTrajectory.transformTrajectoryForAlliance(twoPieceLoadsideC, DriverStation.Alliance.Red);
        }

        addCommands(
                m_chassis.runOnce(() -> { m_chassis.setGyroAngle(0.0); }),
                new ArmToPosition(m_arm, MID_BASE_POS, MID_WRIST_POS, 0.1),
                new RunVoltsTime(m_Intake, OUTTAKE_VOLTS, 0.25),
                new ParallelCommandGroup(
                        new ArmToPosition(m_arm,INTAKE_BASE_POS_CUBE,INTAKE_WRIST_POS_CUBE),
                        m_chassis.followTrajectoryCommand(twoPieceLoadsideA,true),
                        new SequentialCommandGroup(new RunVoltsTime(m_Intake,0,1),
                                new RunVoltsTime(m_Intake,-6,twoPieceLoadsideA.getTotalTimeSeconds()-1)
                        )
                ),
                m_chassis.runOnce(() -> {m_Intake.set(-0.2);}),
                new ParallelCommandGroup(
                        new ArmToPosition(m_arm, MID_BASE_POS_CUBE, MID_WRIST_POS_CUBE),
                        m_chassis.followTrajectoryCommand(twoPieceLoadsideB,false)
                ),
                new ChassisDriveAuton(m_chassis, 0.2, 0.0, 0.0, 0.1),
                new RunVoltsTime(m_Intake,3,0.3),
                new ParallelCommandGroup(
                        new ArmToPosition(m_arm,INTAKE_BASE_POS_CONE,INTAKE_WRIST_POS_CONE),
                        m_chassis.followTrajectoryCommand(twoPieceLoadsideC,false),
                        new RunVoltsTime(m_Intake,-11,twoPieceLoadsideC.getTotalTimeSeconds())
                ),
                new ArmToPosition(m_arm, 0, 0)
        );
    }
}
