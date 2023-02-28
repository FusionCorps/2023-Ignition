package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ArmToPositionOverride extends CommandBase {

    Arm mArm;

    double baseTarget;
    double wristTarget;

    Timer timer = new Timer();
    boolean timerStarted = false;

    double delay;

    public ArmToPositionOverride(Arm arm, double basePos, double wristPos) {
        mArm = arm;

        baseTarget = basePos;
        wristTarget = wristPos;

        delay = 0.25;
    }

    public ArmToPositionOverride(Arm arm, double basePos, double wristPos, double delayTime) {
        mArm = arm;

        baseTarget = basePos;
        wristTarget = wristPos;

        delay = delayTime;
    }

    @Override
    public void initialize() {
        mArm.setTalonTargets(baseTarget, wristTarget);
        timer.reset();
        timer.stop();
        timerStarted = false;

        mArm.overriding = true;
    }

    @Override
    public void execute() {
        if (!timerStarted && (mArm.armAtTarget() && mArm.wristAtTarget())) {
            timerStarted = true;
            timer.reset();
            timer.start();
        }
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(delay) || mArm.baseTalonTarget != baseTarget || mArm.wristTalonTarget != wristTarget;
    }

    @Override
    public void end(boolean isFinished) {
        System.out.println("ArmToPosition Finished");
        mArm.overriding = false;
    }

}
