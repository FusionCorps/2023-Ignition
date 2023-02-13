package frc.robot.commands.chassis;

import com.fasterxml.jackson.core.sym.NameN;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;

import static edu.wpi.first.math.MathUtil.clamp;
import static frc.robot.RobotContainer.m_controller;
import static java.lang.Math.*;
import static java.lang.StrictMath.PI;

public class ChassisDriveFCFlickStick extends CommandBase {

    Chassis mChassis;


    // limit stick accel
    private SlewRateLimiter fwdLimiter = new SlewRateLimiter(4.5);
    private SlewRateLimiter strLimiter = new SlewRateLimiter(4.5);
    private SlewRateLimiter rotLimiter = new SlewRateLimiter(4.5);

//    ProfiledPIDController rotController = new ProfiledPIDController(0.05, 0, 0,
//            new TrapezoidProfile.Constraints(5, 10));

    PIDController rotController = new PIDController(0.01, 0, 0.0);


    public ChassisDriveFCFlickStick(Chassis chassis) {
        mChassis = chassis;

        addRequirements(mChassis);

    }

    @Override
    public void initialize() {
        rotController.enableContinuousInput(-180, 180);
    }

    @Override
    public void execute() {
        // pass args to swerve modules

        double angle = -(mChassis.ahrs.getAngle() % 360);

        // toggle gyro off
//        angle = 0;

        // getRawAxis returns from -1 to 1 joystick magnitude
        // 0 - LJoystick X Axis
        // 1 - LJoystick Y Axis
        // 4 - RJoystick X Axis
        double axis0 = m_controller.getRawAxis(0);
        double axis1 = m_controller.getRawAxis(1);
        double axis4 = m_controller.getRawAxis(4);

        if (pow(m_controller.getRightX(),2) + pow(m_controller.getRightY(),2) > 0.25 ) {
            mChassis.desiredHeading = Math.atan2(-m_controller.getRightX(), -m_controller.getRightY()) / PI * 180;
        }



        // skew correction
        // angle -= 40*axis4;

        try {
            mChassis.runSwerve(fwdLimiter.calculate(axis1*cos(angle/360*(2*PI)) + axis0*sin(angle/360*(2*PI))),
                    strLimiter.calculate(axis1*sin(angle/360*(2*PI)) - axis0*cos(angle/360*(2*PI))),
                    rotLimiter.calculate(clamp(-1*rotController.calculate(angle, mChassis.desiredHeading), -0.75, 0.75)));
        } catch (Exception e) {

        }
    }

}
