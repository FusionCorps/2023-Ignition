package frc.robot.commands.chassis;

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

        // getRawAxis returns from -1 to 1 joystick magnitude
        // 0 - LJoystick X Axis
        // 1 - LJoystick Y Axis
        // 4 - RJoystick X Axis
        try {
            mChassis.runSwerve(m_controller.getRawAxis(1),
                    -m_controller.getRawAxis(0),
                    m_controller.getRawAxis(4));
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

}
