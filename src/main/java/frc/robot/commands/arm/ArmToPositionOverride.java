package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

// arm to position but I turned off the safety features
// really for testing only DO NOT USE UNLESS YOU KNOW WHAT YOU ARE DOING
// SERIOUSLY THIS COULD BREAK !@$#
// I don't even know if this works anymore either
public class ArmToPositionOverride extends CommandBase {

    Arm mArm;

    double baseTarget;
    double wristTarget;

    Timer timer = new Timer();
    boolean timerStarted = false;

    double delay;

    // this is basically the same thing but minus the safety checks
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
