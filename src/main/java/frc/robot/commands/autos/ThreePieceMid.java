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

public class ThreePieceMid extends AllianceCommand {

    PathPlannerTrajectory threePieceLoadSideA;
    PathPlannerTrajectory threePieceLoadSideB;
    PathPlannerTrajectory threePieceLoadSideC;
    PathPlannerTrajectory threePieceLoadSideD;

    Cameras m_cameras;
    Chassis m_chassis;
    Arm m_arm;
    Intake m_intake;

    boolean isRed;

    public ThreePieceMid(Cameras camera, Chassis chassis, Arm arm, Intake intake, boolean isRed){
        this.isRed = isRed;

        m_cameras = camera;
        m_chassis = chassis;
        m_arm = arm;
        m_intake = intake;

        threePieceLoadSideA = PathPlanner.loadPath("1+1_path1R_NP", new PathConstraints(5, 2.25));
        threePieceLoadSideB = PathPlanner.loadPath("1+2Cube_2R Copy", new PathConstraints(5, 3));
        threePieceLoadSideC = PathPlanner.loadPath("1+2Cube_3R", new PathConstraints(4, 3));
        threePieceLoadSideD = PathPlanner.loadPath("1+2Cube_4R_NP", new PathConstraints(4, 3));

        if(isRed){
            threePieceLoadSideA = PathPlannerTrajectory.transformTrajectoryForAlliance(threePieceLoadSideA, DriverStation.Alliance.Red);
            threePieceLoadSideB = PathPlannerTrajectory.transformTrajectoryForAlliance(threePieceLoadSideB, DriverStation.Alliance.Red);
            threePieceLoadSideC = PathPlannerTrajectory.transformTrajectoryForAlliance(threePieceLoadSideC, DriverStation.Alliance.Red);
            threePieceLoadSideD = PathPlannerTrajectory.transformTrajectoryForAlliance(threePieceLoadSideD, DriverStation.Alliance.Red);
        }

        addCommands(
                m_cameras.runOnce(() -> { System.out.println("Running two piece loader side"); }),
                m_chassis.runOnce(() -> { m_chassis.setGyroAngle(0.0); }),
                new ArmToPosition(m_arm, MID_BASE_POS, MID_WRIST_POS, 0.1),
                new RunVoltsTime(m_intake, OUTTAKE_VOLTS, 0.25),
                new ParallelCommandGroup(
                        new ArmToPosition(m_arm,INTAKE_BASE_POS_CUBE,INTAKE_WRIST_POS_CUBE),
                        m_chassis.followTrajectoryCommand(threePieceLoadSideA,true),
                        new RunVoltsTime(m_intake,-6,threePieceLoadSideA.getTotalTimeSeconds())
                ),
                m_chassis.runOnce(() -> {m_intake.set(-0.2);}),
                new ParallelCommandGroup(
                        new ArmToPosition(m_arm, MID_BASE_POS_CUBE, MID_WRIST_POS_CUBE),
                        m_chassis.followTrajectoryCommand(threePieceLoadSideB,false)
                ),
                new ChassisDriveAuton(m_chassis, 0.25, 0.0, 0.0, 0.1),
                new RunVoltsTime(m_intake,3.5,0.3),
                new ParallelCommandGroup(
                        new ArmToPosition(m_arm,INTAKE_BASE_POS_CONE,INTAKE_WRIST_POS_CONE),
                        m_chassis.followTrajectoryCommand(threePieceLoadSideC,false),
                        new RunVoltsTime(m_intake,-11,threePieceLoadSideC.getTotalTimeSeconds())
                ),
                m_chassis.runOnce(() -> {m_intake.set(-0.2);}),
                new ParallelCommandGroup(
                        new ArmToPosition(m_arm,MID_BASE_POS,MID_WRIST_POS),
                        m_chassis.followTrajectoryCommand(threePieceLoadSideD,false)
                ),
                new ChassisDriveAuton(m_chassis,0.2,0,0,0.23),
                new RunVoltsTime(m_intake,OUTTAKE_VOLTS,0.25)


        );

    }

    private void init() {
        if(isRed){
            threePieceLoadSideA = PathPlannerTrajectory.transformTrajectoryForAlliance(threePieceLoadSideA, DriverStation.Alliance.Red);
            threePieceLoadSideB = PathPlannerTrajectory.transformTrajectoryForAlliance(threePieceLoadSideB, DriverStation.Alliance.Red);
            threePieceLoadSideC = PathPlannerTrajectory.transformTrajectoryForAlliance(threePieceLoadSideC, DriverStation.Alliance.Red);
            threePieceLoadSideD = PathPlannerTrajectory.transformTrajectoryForAlliance(threePieceLoadSideD, DriverStation.Alliance.Red);
        }

        addCommands(
                m_cameras.runOnce(() -> { System.out.println("Running two piece loader side"); }),
                m_chassis.runOnce(() -> { m_chassis.setGyroAngle(0.0); }),
                new ArmToPosition(m_arm, MID_BASE_POS, MID_WRIST_POS, 0.1),
                new RunVoltsTime(m_intake, OUTTAKE_VOLTS, 0.25),
                new ParallelCommandGroup(
                        new ArmToPosition(m_arm,INTAKE_BASE_POS_CUBE,INTAKE_WRIST_POS_CUBE),
                        m_chassis.followTrajectoryCommand(threePieceLoadSideA,true),
                        new RunVoltsTime(m_intake,-6,threePieceLoadSideA.getTotalTimeSeconds())
                ),
                m_chassis.runOnce(() -> {m_intake.set(-0.2);}),
                new ParallelCommandGroup(
                        new ArmToPosition(m_arm, MID_BASE_POS_CUBE, MID_WRIST_POS_CUBE),
                        m_chassis.followTrajectoryCommand(threePieceLoadSideB,false)
                ),
                new ChassisDriveAuton(m_chassis, 0.25, 0.0, 0.0, 0.1),
                new RunVoltsTime(m_intake,3.5,0.3),
                new ParallelCommandGroup(
                        new ArmToPosition(m_arm,INTAKE_BASE_POS_CONE,INTAKE_WRIST_POS_CONE),
                        m_chassis.followTrajectoryCommand(threePieceLoadSideC,false),
                        new RunVoltsTime(m_intake,-11,threePieceLoadSideC.getTotalTimeSeconds())
                ),
                m_chassis.runOnce(() -> {m_intake.set(-0.2);}),
                new ParallelCommandGroup(
                        new ArmToPosition(m_arm,MID_BASE_POS,MID_WRIST_POS),
                        m_chassis.followTrajectoryCommand(threePieceLoadSideD,false)
                ),
                new ChassisDriveAuton(m_chassis,0.2,0,0,0.23),
                new RunVoltsTime(m_intake,OUTTAKE_VOLTS,0.25)


        );
    }

    public void changeAlliance(boolean isRed) {

    }


}
