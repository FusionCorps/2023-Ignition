package frc.robot.commands.Chassis;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;

import static frc.robot.RobotContainer.m_controller;

public class ChassisDrive extends CommandBase {

    Chassis mChassis;

    boolean isController = true;

    double fwd = 0;
    double rot = 0;
    double str = 0;
    double time = 0;


    public void setAutoDrive(double f, double r, double s, double t) {
        fwd = f;
        rot = r;
        str = s;
        time = t;
    }

    public ChassisDrive(Chassis chassis) {
        mChassis = chassis;

        addRequirements(mChassis);
    }

    @Override
    public void execute() {
        // pass args to swerve modules
        if (isController) {
            try {
                mChassis.runSwerve(m_controller.getRawAxis(1),
                        -m_controller.getRawAxis(0),
                        m_controller.getRawAxis(4));
            } catch (Exception e) {
                e.printStackTrace();
            }
        } else {

        }
    }

}
