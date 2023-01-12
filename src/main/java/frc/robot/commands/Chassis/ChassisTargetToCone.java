package frc.robot.commands.Chassis;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Cameras;
import frc.robot.subsystems.Chassis;

import static frc.robot.RobotContainer.m_controller;

public class ChassisTargetToCone extends CommandBase {

    Chassis mChassis;
    Cameras mCameras;

    PIDController str_controller = new PIDController(0.04, 0, 0);
    PIDController rot_controller = new PIDController(0.01, 0, 0);

    double starting_angle;

    public ChassisTargetToCone(Chassis chassis, Cameras cameras) {
        mChassis = chassis;
        mCameras = cameras;

        addRequirements(mChassis);

        rot_controller.enableContinuousInput(0, 360);

        starting_angle = mChassis.ahrs.getAngle() % 360;
    }

    @Override
    public void initialize() {
        starting_angle = mChassis.ahrs.getAngle() % 360;
    }

    @Override
    public void execute() {
        // pass args to swerve modules
        double tx = mCameras.ll_table.getEntry("tx").getDouble(0.0);
        double angle = mChassis.ahrs.getAngle() % 360;

        System.out.println(angle);

        try {
            mChassis.runSwerve(m_controller.getRawAxis(1),
                    str_controller.calculate(tx, 0),
                    rot_controller.calculate(angle, starting_angle));
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

}
