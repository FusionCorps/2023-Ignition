package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

import static frc.robot.RobotContainer.m_controller;

public class IntakeCube extends CommandBase {

    Intake mIntake;
    double pct;

    public IntakeCube(Intake intake, double percent) {
        mIntake = intake;
        pct = percent;

        addRequirements(mIntake);
    }

    @Override
    public void execute() {
        mIntake.set(pct);

        if (-1*mIntake.getSpeed() < 500) {
            m_controller.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0.1);
        }

    }

    @Override
    public void end(boolean isFinished) {
        m_controller.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0.0);
    }

}
