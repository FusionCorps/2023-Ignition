package frc.robot.commands.autos;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.ArmToPosition;
import frc.robot.commands.arm.TwoPartHigh;
import frc.robot.commands.arm.TwoPartHighAuto;
import frc.robot.commands.chassis.ChassisDriveAuton;
import frc.robot.commands.intake.RunVoltsTime;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Cameras;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Intake;

import static frc.robot.Constants.ArmConstants.*;
import static frc.robot.Constants.IntakeConstants.OUTTAKE_VOLTS;

public class TwoPieceHighWire extends SequentialCommandGroup {

    Chassis m_chassis;
    Arm m_arm;
    Cameras m_camera;
    Intake m_intake;

    PathPlannerTrajectory twoPieceWireSideA;
    PathPlannerTrajectory twoPieceWireSideB;

    public TwoPieceHighWire(Chassis chassis, Arm arm, Cameras camera, Intake intake, boolean isRed){
        m_chassis = chassis;
        m_arm = arm;
        m_intake = intake;
        m_camera = camera;

        twoPieceWireSideA = PathPlanner.loadPath("1+1_wire_path1", new PathConstraints(2,1));
        twoPieceWireSideB = PathPlanner.loadPath("1+1_wire_path2", new PathConstraints(2,1));

        if(isRed){
            twoPieceWireSideA = PathPlannerTrajectory.transformTrajectoryForAlliance(twoPieceWireSideA, DriverStation.Alliance.Red);
            twoPieceWireSideB = PathPlannerTrajectory.transformTrajectoryForAlliance(twoPieceWireSideB,DriverStation.Alliance.Red);
        }

        addCommands(
                m_chassis.runOnce(() -> { m_chassis.setGyroAngle(0.0); }),
                new TwoPartHighAuto(m_arm),
                new ArmToPosition(m_arm,HIGH_BASE_POS_ALT_AUTO,HIGH_WRIST_POS_ALT_AUTO),
                new RunVoltsTime(m_intake,OUTTAKE_VOLTS,0.5),
                new ArmToPosition(m_arm,BASE_START_POS,WRIST_START_POS),
                m_chassis.followTrajectoryCommand(twoPieceWireSideA,true),
                m_chassis.followTrajectoryCommand(twoPieceWireSideB,false),
                new ArmToPosition(m_arm,HIGH_BASE_POS,HIGH_WRIST_POS),
                new ChassisDriveAuton(m_chassis,0.2,0,0,0.1),
                new RunVoltsTime(m_intake,OUTTAKE_VOLTS,0.2),
                new ChassisDriveAuton(m_chassis,-0.2,0,0,0.1),
                new ArmToPosition(m_arm,BASE_START_POS,WRIST_START_POS)
        );

    }


}
