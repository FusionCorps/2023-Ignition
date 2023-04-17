package frc.robot.commands.autos;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.ArmToPosition;
import frc.robot.commands.chassis.ChassisDriveAuton;
import frc.robot.commands.intake.RunVoltsTime;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Cameras;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Intake;

import java.nio.file.Path;

import static frc.robot.Constants.ArmConstants.*;
import static frc.robot.Constants.IntakeConstants.OUTTAKE_VOLTS;

public class ThreePieceMid extends SequentialCommandGroup {

    PathPlannerTrajectory threePieceLoadSideA;
    PathPlannerTrajectory threePieceLoadSideB;
    PathPlannerTrajectory threePieceLoadSideC;
    PathPlannerTrajectory threePieceLoadSideD;

    Cameras m_cameras;
    Chassis m_chassis;
    Arm m_arm;
    Intake m_intake;

    public ThreePieceMid(Cameras camera, Chassis chassis, Arm arm, Intake intake, boolean isRed){

        m_cameras = camera;
        m_chassis = chassis;
        m_arm = arm;
        m_intake = intake;

        threePieceLoadSideA = PathPlanner.loadPath("MilB_3Piece_1", new PathConstraints(5, 1.75));
        threePieceLoadSideB = PathPlanner.loadPath("MilB_3Piece_2", new PathConstraints(5, 1.75));
        threePieceLoadSideC = PathPlanner.loadPath("MilB_3Piece_3", new PathConstraints(5, 2.0));
        threePieceLoadSideD = PathPlanner.loadPath("MilB_3Piece_4", new PathConstraints(5, 3.25));

        if(isRed){
            threePieceLoadSideA = PathPlanner.loadPath("MilR_3Piece_1", new PathConstraints(5, 1.75));
            threePieceLoadSideB = PathPlanner.loadPath("MilR_3Piece_2", new PathConstraints(5, 1.75));
            threePieceLoadSideC = PathPlanner.loadPath("MilR_3Piece_3", new PathConstraints(5, 2.0));
            threePieceLoadSideD = PathPlanner.loadPath("MilR_3Piece_4", new PathConstraints(5, 3.25));

            threePieceLoadSideA = PathPlannerTrajectory.transformTrajectoryForAlliance(threePieceLoadSideA, DriverStation.Alliance.Red);
            threePieceLoadSideB = PathPlannerTrajectory.transformTrajectoryForAlliance(threePieceLoadSideB, DriverStation.Alliance.Red);
            threePieceLoadSideC = PathPlannerTrajectory.transformTrajectoryForAlliance(threePieceLoadSideC, DriverStation.Alliance.Red);
            threePieceLoadSideD = PathPlannerTrajectory.transformTrajectoryForAlliance(threePieceLoadSideD, DriverStation.Alliance.Red);
        }

        addCommands(
                m_cameras.runOnce(() -> { System.out.println("Running two piece loader side"); }),
                m_chassis.runOnce(() -> { m_chassis.setGyroAngle(0.0); }),
                new ArmToPosition(m_arm, MID_BASE_POS, MID_WRIST_POS, 0.25),
                new RunVoltsTime(m_intake, OUTTAKE_VOLTS, 0.2),
                new ParallelCommandGroup(
                        new ArmToPosition(m_arm,INTAKE_BASE_POS_CUBE,INTAKE_WRIST_POS_CUBE),
                        m_chassis.followTrajectoryCommand(threePieceLoadSideA,true),
                        new SequentialCommandGroup(new RunVoltsTime(m_intake,0,1),
                                new RunVoltsTime(m_intake,-0.45*12,threePieceLoadSideA.getTotalTimeSeconds()-1)
                        )
                ),
                new ParallelCommandGroup(
                        m_intake.runOnce(() -> {m_intake.set(-0.2);}),
                        new ArmToPosition(m_arm, MID_BASE_POS_CUBE, MID_WRIST_POS_CUBE),
                        m_chassis.followTrajectoryCommand(threePieceLoadSideB,false)
                ),
                new ChassisDriveAuton(m_chassis, 0.25, 0.0, 0.0, 0.1),
                new RunVoltsTime(m_intake,3.5,0.1),
                new ParallelCommandGroup(
                        new ArmToPosition(m_arm,INTAKE_BASE_POS_CONE,INTAKE_WRIST_POS_CONE),
                        m_chassis.followTrajectoryCommand(threePieceLoadSideC,false),
                        new RunVoltsTime(m_intake,-11,threePieceLoadSideC.getTotalTimeSeconds())
                ),
                m_intake.runOnce(() -> {m_intake.set(-0.2);}),
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                // delay putting arm up to avoid rocking robot
                                new ArmToPosition(m_arm,0,0),
                                new RunVoltsTime(m_intake,-1.0,threePieceLoadSideD.getTotalTimeSeconds()*0.4),
                                new ArmToPosition(m_arm,MID_BASE_POS,MID_WRIST_POS)
                        ),
                        m_chassis.followTrajectoryCommand(threePieceLoadSideD,false)
                ),
                new ChassisDriveAuton(m_chassis,0.1,0,0,0.13),
                new RunVoltsTime(m_intake,OUTTAKE_VOLTS,0.25)


        );

    }


}
