package frc.robot.commands.Chassis;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;

import static frc.robot.RobotContainer.m_controller;
import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.StrictMath.PI;

public class ChassisDriveFC extends CommandBase {
    Chassis mChassis;

    private SlewRateLimiter fwdLimiter = new SlewRateLimiter(6.5);
    private SlewRateLimiter strLimiter = new SlewRateLimiter(6.5);
    private SlewRateLimiter rotLimiter = new SlewRateLimiter(6.5);

    public ChassisDriveFC(Chassis chassis) {
        mChassis = chassis;

        addRequirements(mChassis);
    }

    @Override
    public void execute() {
        // pass args to swerve modules

        double angle = -(mChassis.ahrs.getAngle() % 360);

        angle = 0;

        double axis0 = m_controller.getRawAxis(0);
        double axis1 = m_controller.getRawAxis(1);
        double axis4 = m_controller.getRawAxis(4);

        angle -= 40*axis4;

        try {
            mChassis.runSwerve(fwdLimiter.calculate(axis1*cos(angle/360*(2*PI)) + axis0*sin(angle/360*(2*PI))),
                    strLimiter.calculate(axis1*sin(angle/360*(2*PI)) - axis0*cos(angle/360*(2*PI))),
                    rotLimiter.calculate(axis4));
        } catch (Exception e) {

        }
    }

}
