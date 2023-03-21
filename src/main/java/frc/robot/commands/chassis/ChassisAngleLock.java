package frc.robot.commands.chassis;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Cameras;
import frc.robot.subsystems.Chassis;

import static edu.wpi.first.math.MathUtil.clamp;
import static frc.robot.RobotContainer.m_controller;
import static java.lang.Math.*;
import static java.lang.StrictMath.PI;

public class ChassisAngleLock extends CommandBase {

    Chassis mChassis;
    Cameras mCameras;

    Timer timer = new Timer();
    double runTime;

    PIDController str_controller = new PIDController(0.04, 0, 0);
    PIDController rot_controller = new PIDController(0.01, 0, 0);

    public ChassisAngleLock(Chassis chassis, Cameras cameras, double time) {
        mChassis = chassis;
        mCameras = cameras;

        runTime = time;
        rot_controller.enableContinuousInput(-180, 180);

        addRequirements(mChassis);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {

        // maintain FC control
        double angle = mChassis.ahrs.getAngle() % 360;


        double tx = -1*mCameras.tx();

        double axis0 = m_controller.getRawAxis(0);
        double axis1 = m_controller.getRawAxis(1);

        System.out.println(tx);

        // deadzone/min error
//        if (abs(tx) <= 2.0) {
//            tx = 0;
//        }

//        if (tx == 0) {
//            try {
//                mChassis.runSwerve(0.05,
//                        clamp(str_controller.calculate(tx, 6), -0.35, 0.35),
//                        clamp(rot_controller.calculate(angle, 0), -0.75, 0.75));
//            } catch (Exception e) {
//                e.printStackTrace();
//            }
//        } else
        if (abs((angle + 180) % 360 - 180) < 3) {
            try {
                mChassis.runSwerve(axis1 * cos(angle / 360 * (2 * PI)) + axis0 * sin(angle / 360 * (2 * PI)),
                        clamp(str_controller.calculate(tx, 6), -0.35, 0.35),
                        clamp(rot_controller.calculate(angle, 0), -0.75, 0.75));
            } catch (Exception e) {
                e.printStackTrace();
            }
        } else {
            try {
                mChassis.runSwerve(0,
                        0,
                        clamp(rot_controller.calculate(angle, 0), -0.75, 0.75));
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(runTime);
    }

    @Override
    public void end(boolean isFinished) {
        try {
            mChassis.runSwerve(0,
                    0,
                    0);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

}
