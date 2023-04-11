package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

import static frc.robot.Constants.ArmConstants.*;

public class ArmToPosition extends CommandBase {

    Arm mArm;

    double baseTarget;
    double wristTarget;

    Timer timer = new Timer();
    boolean timerStarted = false;

    double delay;

    public ArmToPosition(Arm arm, double basePos, double wristPos) {
        mArm = arm;

        baseTarget = basePos;
        wristTarget = wristPos;

        delay = 0.25;
    }

    public ArmToPosition(Arm arm, double basePos, double wristPos, double delayTime) {
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
    }

    @Override
    public void execute() {

//        mArm.baseTalonTarget = HIGH_BASE_POS_ALT_PREP;
//        mArm.wristTalonTarget = HIGH_WRIST_POS_ALT;

        if (mArm.armAtTarget()) {
            mArm.passSetpoints(mArm.baseTalonTarget, mArm.wristTalonTarget);
        } else if (mArm.wristStowed()) {
            mArm.passSetpoints(mArm.baseTalonTarget, WRIST_STOWED_POS);
        } else {
            mArm.stowWrist();
        }

        if (!timerStarted && (mArm.armAtTarget() && mArm.wristAtTarget())) {
            timerStarted = true;
            timer.reset();
            timer.start();
        }
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(delay);
    }

    @Override
    public void end(boolean isFinished) {
        System.out.println("ArmToPosition Finished");
    }

}
