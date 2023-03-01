package frc.robot.commands.Chassis;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;

import static frc.robot.RobotContainer.m_chassis;
import static frc.robot.RobotContainer.m_controller;
import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.StrictMath.PI;

public class ChassisDriveFC extends CommandBase {

    Chassis mChassis;

    boolean isController = true;

    double fwd = 0;
    double rot = 0;
    double str = 0;
    double time = 0;

    Timer timer = new Timer();

    public void setAutoDrive(double f, double r, double s, double t) {
        fwd = f;
        rot = r;
        str = s;
        time = t;
    }

    public void startAuto() {
        isController = false;
        timer.reset();
        timer.start();
    }

    public void endAuto() {
        isController = true;
    }


    private SlewRateLimiter fwdLimiter = new SlewRateLimiter(6.5);
    private SlewRateLimiter strLimiter = new SlewRateLimiter(6.5);
    private SlewRateLimiter rotLimiter = new SlewRateLimiter(6.5);

    public ChassisDriveFC(Chassis chassis) {
        mChassis = m_chassis;

        addRequirements(m_chassis);
    }

    public void execute() {
        // pass args to swerve modules

        double angle = -(mChassis.ahrs.getAngle() % 360);
        double axis0;
        double axis1;
        double axis4;
        if (isController) {
            axis0 = m_controller.getRawAxis(0);
            axis1 = m_controller.getRawAxis(1);
            axis4 = m_controller.getRawAxis(4);
        } else if (!timer.hasElapsed(time) || time > 0) {
            axis0 = str;
            axis1 = -fwd;
            axis4 = rot;
        } else {
            axis0 = 0;
            axis1 = 0;
            axis4 = 0;
        }

        angle -= 40*axis4;

        try {
            mChassis.runSwerve(fwdLimiter.calculate(axis1*cos(angle/360*(2*PI)) + axis0*sin(angle/360*(2*PI))),
                    strLimiter.calculate(axis1*sin(angle/360*(2*PI)) - axis0*cos(angle/360*(2*PI))),
                    rotLimiter.calculate(axis4));
        } catch (Exception e) {

        }
    }

}
