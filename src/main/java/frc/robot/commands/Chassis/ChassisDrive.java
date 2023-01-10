package frc.robot.commands.Chassis;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;

import static frc.robot.RobotContainer.m_controller;

public class ChassisDrive extends CommandBase {

    Chassis mChassis;

    public ChassisDrive(Chassis chassis) {
        mChassis = chassis;

        addRequirements(mChassis);
    }

    @Override
    public void execute() {
        // pass args to swerve modules
        try {
            mChassis.runSwerve(m_controller.getRawAxis(1),
                    -m_controller.getRawAxis(0),
                    m_controller.getRawAxis(4));
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

}
