package frc.robot.commands.autos;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.ArmToPosition;
import frc.robot.commands.arm.TwoPartHighAuto;
import frc.robot.commands.chassis.ChassisAutoBalanceFast;
import frc.robot.commands.chassis.ChassisDriveAuton;
import frc.robot.commands.intake.RunVoltsTime;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Cameras;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Intake;

import static frc.robot.Constants.ArmConstants.*;
import static frc.robot.Constants.IntakeConstants.OUTTAKE_VOLTS;

// Score one taxi then balance.
// Might be good to curve this one towards the middle to avoid falling off charge station edge.
public class OnePieceTaxiBalance extends SequentialCommandGroup {

    Chassis m_chassis;
    Arm m_arm;
    Intake m_intake;
    Cameras m_camera;

    PathPlannerTrajectory twoPieceHighBalanceA;
    PathPlannerTrajectory twoPieceHighBalanceB;
    PathPlannerTrajectory balance;

    public OnePieceTaxiBalance(Chassis chassis, Arm arm, Cameras camera, Intake intake, boolean isRed){
        m_arm = arm;
        m_intake = intake;
        m_camera = camera;
        m_chassis = chassis;

        twoPieceHighBalanceA = PathPlanner.loadPath("1+taxi+balance1",new PathConstraints(1.5,2));
        twoPieceHighBalanceB = PathPlanner.loadPath("1+taxi+balance2",new PathConstraints(4,3));
        balance = PathPlanner.loadPath("1+taxi+balance3",new PathConstraints(4,3));

        if(isRed){
            twoPieceHighBalanceA = PathPlannerTrajectory.transformTrajectoryForAlliance(twoPieceHighBalanceA, DriverStation.Alliance.Red);
            twoPieceHighBalanceB = PathPlannerTrajectory.transformTrajectoryForAlliance(twoPieceHighBalanceB, DriverStation.Alliance.Red);
            balance = PathPlannerTrajectory.transformTrajectoryForAlliance(balance, DriverStation.Alliance.Red);
        }

        addCommands(
                // reset gyro
                m_chassis.runOnce(() -> { m_chassis.setGyroAngle(0.0); }),
                // score first cone
                new TwoPartHighAuto(m_arm),
                new ArmToPosition(m_arm,HIGH_BASE_POS_ALT_AUTO,HIGH_WRIST_POS_ALT_AUTO),
                new RunVoltsTime(m_intake,OUTTAKE_VOLTS,0.5),
                new ArmToPosition(m_arm,BASE_START_POS, WRIST_START_POS),
                // go over the charge station (note maybe deprecated check the path b4 running)
                new ParallelCommandGroup(
                        //new RunVoltsTime(m_intake,-6,twoPieceHighBalanceA.getTotalTimeSeconds()),
                        m_chassis.followTrajectoryCommand(twoPieceHighBalanceA,true)
                        // new ArmToPosition(m_arm,INTAKE_BASE_POS_CUBE,INTAKE_WRIST_POS_CUBE,2.3)
                ),
                // drive back and balance
                new ChassisDriveAuton(m_chassis,0.3,0,0,2.0),
                new ChassisAutoBalanceFast(m_chassis)
//                new ParallelCommandGroup(
//                        m_chassis.followTrajectoryCommand(twoPieceHighBalanceB,false),
//                        new ArmToPosition(m_arm,BASE_START_POS,WRIST_START_POS)
//                ),
                // new ArmToPosition(m_arm,HIGH_BASE_POS,HIGH_WRIST_POS),
//                new ChassisDriveAuton(m_chassis,0.2,0,0,0.15),
//                new RunVoltsTime(m_intake,OUTTAKE_VOLTS,0.2),
//                new ParallelCommandGroup(
//                        m_chassis.followTrajectoryCommand(balance,false),
//                        new ArmToPosition(m_arm,BASE_START_POS,WRIST_START_POS,0.2)
//                ),
//                new ChassisAutoBalanceFast(m_chassis)
        );
    }


}
