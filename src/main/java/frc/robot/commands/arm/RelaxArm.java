package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

// command to make arm not try to reset on wakeup
public class RelaxArm extends CommandBase {

    Timer mTimer;
    Arm mArm;

    public RelaxArm(Arm arm) {
        mArm = arm;
        mTimer = new Timer();

        addRequirements(mArm);
    }

    @Override
    public void initialize() {
        mTimer.reset();
        mTimer.start();
    }

    @Override
    public void execute() {
        mArm.setHoldCurrentPos();
        mArm.setMotorsBrake();
    }

    @Override
    public boolean isFinished() {
        return mTimer.hasElapsed(0.5);
    }

    @Override
    public void end(boolean interrupted) {
        mArm.setMotorsBrake();
    }

}
