package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.ArmToPosition;
import frc.robot.commands.arm.TwoPartHighAuto;
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

public class OneHighIntakeBalance extends SequentialCommandGroup {

    Chassis m_chassis;
    Arm m_arm;
    Intake m_intake;

    // using dead reckoning
    public OneHighIntakeBalance(Chassis chassis, Arm arm, Intake intake) {

        m_chassis = chassis;
        m_arm = arm;
        m_intake = intake;

        addCommands(
                m_chassis.runOnce(() -> {
                    m_chassis.setGyroAngle(0.0);
                }), // reset gyro
                new TwoPartHighAuto(m_arm), // arm to high
                new ArmToPosition(m_arm, HIGH_BASE_POS_ALT_AUTO, HIGH_WRIST_POS_ALT_AUTO, 0.25),
                new RunVoltsTime(m_intake, OUTTAKE_VOLTS, 0.75), // outtake
                new ArmToPosition(m_arm, 0, 0), // stow
                new ChassisDriveAuton(m_chassis, -0.5, 0.0, 0.0, 3.0), // drive forward
                // drive forward + intake
                new ParallelCommandGroup(
                        new ChassisDriveAuton(m_chassis, -0.2, 0.0, 0.0, 2.0),
                        new RunVoltsTime(m_intake,-11,2.0),
                        new ArmToPosition(m_arm,INTAKE_BASE_POS_CONE,INTAKE_WRIST_POS_CONE)
                ),
                // stow
                new ArmToPosition(m_arm, 0, 0),
                // drive back
                new ChassisDriveAuton(m_chassis, 0.5, 0.0, 0.0, 4.0),
                // balance
                new ChassisAutoBalanceNew(m_chassis) // balance
        );
    }

}
