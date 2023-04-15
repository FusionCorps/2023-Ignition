package frc.robot.commands.autos;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.ArmToPosition;
import frc.robot.commands.arm.TwoPartHighAuto;
import frc.robot.commands.chassis.ChassisDriveAuton;
import frc.robot.commands.intake.RunVoltsTime;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Cameras;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Intake;

import static frc.robot.Constants.ArmConstants.*;
import static frc.robot.Constants.ArmConstants.WRIST_START_POS;
import static frc.robot.Constants.IntakeConstants.OUTTAKE_VOLTS;

public class ThreePieceWireSide extends SequentialCommandGroup {
    // cone -> cube -> cone
    // might switch cone and cube order since cube is more forgiving

    Chassis m_chassis;
    Arm m_arm;
    Cameras m_camera;
    Intake m_intake;

    PathPlannerTrajectory twoPieceWireSideA;
    PathPlannerTrajectory twoPieceWireSideB;
    PathPlannerTrajectory threePieceWireSideC;
    PathPlannerTrajectory threePieceWireSideD;


    public ThreePieceWireSide(Chassis chassis, Arm arm, Cameras camera, Intake intake, boolean isRed){
        m_chassis = chassis;
        m_arm = arm;
        m_intake = intake;
        m_camera = camera;

        // need to push these to 5/3 in order to make time
        twoPieceWireSideA = PathPlanner.loadPath("Titan_1+1_wire_path1", new PathConstraints(5,3));
        twoPieceWireSideB = PathPlanner.loadPath("Why_1+1_wire_path2", new PathConstraints(5,3));
        threePieceWireSideC = PathPlanner.loadPath("Titan_1+2_wire_path3", new PathConstraints(5,3));
        threePieceWireSideD = PathPlanner.loadPath("Titan_1+2_wire_path4", new PathConstraints(5,3));

        if(isRed){
            twoPieceWireSideA = PathPlannerTrajectory.transformTrajectoryForAlliance(twoPieceWireSideA, DriverStation.Alliance.Red);
            twoPieceWireSideB = PathPlannerTrajectory.transformTrajectoryForAlliance(twoPieceWireSideB,DriverStation.Alliance.Red);
            threePieceWireSideC = PathPlannerTrajectory.transformTrajectoryForAlliance(threePieceWireSideC,DriverStation.Alliance.Red);
            threePieceWireSideD = PathPlannerTrajectory.transformTrajectoryForAlliance(threePieceWireSideD,DriverStation.Alliance.Red);
        }

        addCommands(
                m_chassis.runOnce(() -> { m_chassis.setGyroAngle(0.0); }),
                // arm to mid and score
                new ArmToPosition(m_arm, MID_BASE_POS, MID_WRIST_POS, 0.25),
                new RunVoltsTime(m_intake, OUTTAKE_VOLTS, 0.25),
                // go intake cube
                new ParallelCommandGroup(
                        new ArmToPosition(m_arm,INTAKE_BASE_POS_CUBE,INTAKE_WRIST_POS_CUBE),
                        m_chassis.followTrajectoryCommand(twoPieceWireSideA,true),
                        new SequentialCommandGroup(
                                new RunVoltsTime(m_intake,1,0.3),
                                new RunVoltsTime(m_intake,-5,twoPieceWireSideA.getTotalTimeSeconds()-0.3)
                        )
                ),
                // hold cube in intake
                m_intake.runOnce(() -> {m_intake.set(-0.2);}),
                // go cube mid and head back
                new ParallelCommandGroup(
                        new ArmToPosition(m_arm,MID_BASE_POS_CUBE, MID_WRIST_POS_CUBE),
                        m_chassis.followTrajectoryCommand(twoPieceWireSideB,false)
                ),
                // pop cube in
                new ParallelCommandGroup(
                    new ChassisDriveAuton(m_chassis, 0.25, 0.0, 0.0, 0.1),
                    new RunVoltsTime(m_intake,3.5,0.1)
                ),
                new ChassisDriveAuton(m_chassis, -0.25, 0.0, 0.0, 0.1),
                // grab next cube
                new ParallelCommandGroup(
                        // avoid crashing intake into wiretrace
                        new SequentialCommandGroup(
                            new RunVoltsTime(m_intake,0.0,1.0).andThen(
                                    new ArmToPosition(m_arm,INTAKE_BASE_POS_CUBE,INTAKE_WRIST_POS_CUBE)
                            ),
                            new RunVoltsTime(m_intake,-8,threePieceWireSideC.getTotalTimeSeconds() - 1.25)
                        ),
                        m_chassis.followTrajectoryCommand(threePieceWireSideC,false)
                ),
                // hold cube in intake
                m_intake.runOnce(() -> {m_intake.set(-0.2);}),
                // bowl for score
                new ParallelCommandGroup(
                        new SequentialCommandGroup(new ArmToPosition(m_arm,LOW_BASE_POS_CUBE,LOW_WRIST_POS_CUBE),
                                new RunVoltsTime(m_intake,-8, 0.5)),
                        m_chassis.followTrajectoryCommand(threePieceWireSideD,false)
                )
                // depending on time we could go for a score with this one too
        );

    }


}



