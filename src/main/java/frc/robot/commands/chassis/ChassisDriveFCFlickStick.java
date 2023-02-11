package frc.robot.commands.chassis;

import com.fasterxml.jackson.core.sym.NameN;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;

import static frc.robot.RobotContainer.m_controller;
import static java.lang.Math.*;
import static java.lang.StrictMath.PI;

public class ChassisDriveFCFlickStick extends CommandBase {

    Chassis mChassis;

    double desiredHeading = 0;

    // limit stick accel
    private SlewRateLimiter fwdLimiter = new SlewRateLimiter(4.5);
    private SlewRateLimiter strLimiter = new SlewRateLimiter(4.5);
    private SlewRateLimiter rotLimiter = new SlewRateLimiter(4.5);

//    ProfiledPIDController rotController = new ProfiledPIDController(0.001, 0, 0,
//            new TrapezoidProfile.Constraints(5, 10));

    PIDController rotController = new PIDController(0.001, 0, 0);


    public ChassisDriveFCFlickStick(Chassis chassis) {
        mChassis = chassis;

        addRequirements(mChassis);

        rotController.enableContinuousInput(0, 360);
    }

    @Override
    public void execute() {
        // pass args to swerve modules

        double angle = -(mChassis.ahrs.getAngle() % 360);

        double angleTrue = angle;

        // toggle gyro off
//        angle = 0;

        // getRawAxis returns from -1 to 1 joystick magnitude
        // 0 - LJoystick X Axis
        // 1 - LJoystick Y Axis
        // 4 - RJoystick X Axis
        double axis0 = m_controller.getRawAxis(0);
        double axis1 = m_controller.getRawAxis(1);
        double axis4 = m_controller.getRawAxis(4);


        if (!Double.isNaN(atan(m_controller.getRightY() / m_controller.getRightX()))) {
            desiredHeading = 180 / PI * atan(m_controller.getRightY() / m_controller.getRightX()) + 90;

            if (m_controller.getRightX() > 0) {
                desiredHeading += 180;
            }
        }


        System.out.println(desiredHeading);

        // skew correction
        angle -= 40*axis4;

        try {
            mChassis.runSwerve(fwdLimiter.calculate(axis1*cos(angle/360*(2*PI)) + axis0*sin(angle/360*(2*PI))),
                    strLimiter.calculate(axis1*sin(angle/360*(2*PI)) - axis0*cos(angle/360*(2*PI))),
                    rotController.calculate(angleTrue, desiredHeading));
        } catch (Exception e) {

        }
    }

}
